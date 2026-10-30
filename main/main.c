/*
 * Lab 5 - Control en discreto de un sistema mecanico (pinon-cremallera), ESP32
 *
 *  Posicion y en mm con derecha positiva (u > 0 mueve a la derecha).
 *
 *  TELEMETRIA (ESP32 -> PC), cada T = 10 ms:
 *      D,t_ms,r_mm,y_mm,e_mm,u_ef_pct,u_pwm_pct,v_mm_s,y_mod_mm,modo
 *      u_ef: salida del controlador (sin zona muerta); u_pwm: PWM aplicado al L298N
 *      y_mod: salida del modelo discreto G(z) con el mismo PWM
 *      modo: 0 manual, 1 continuo-discreto, 2 discreto-discreto
 *  CFG,T_ms,q0,q1,q2,s0 (CD),q0,q1,q2,s0 (DD),u0+,u0-,u_ef_max,u_ef_min,dz,lim
 *  COMANDOS (PC -> ESP32), una linea terminada en '\n':
 *      R <mm>          referencia
 *      C <0|1|2>       modo: manual (motor parado), continuo-discreto, discreto-discreto
 *      Q <1|2> q0 q1 q2 s0   coeficientes del controlador 1 (CD) o 2 (DD)
 *      DZ <0|1>        compensacion de zona muerta
 *      ZERO            la posicion actual pasa a ser 0 mm
 *      U <pct>         PWM manual con signo (-100..100), + = derecha
 *      P <pct> <ms>    pulso de PWM manual durante <ms> y luego STOP
 *      S               STOP (modo manual)
 *      LIM <mm>        limite de seguridad |y|: apaga el control si se supera
 *      CFG             reenvia la configuracion
 *  Respuesta: ACK,<texto> o ERR,<texto>
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"

// Puente H L298N (canal B)
#define PIN_ENB         19      // PWM de velocidad
#define PIN_IN1         5       // Direccion
#define PIN_IN2         18      // Direccion

#define PWM_FREQ_HZ     1000
#define PWM_RES         LEDC_TIMER_10_BIT   // 0..1023
#define PWM_MAX         1023

// Encoder del motor (cuadratura)
#define ENCODER_GPIO_A          34
#define ENCODER_GPIO_B          35
#define PCNT_GLITCH_FILTER_NS   1000
#define PCNT_HIGH_LIMIT         30000
#define PCNT_LOW_LIMIT          -30000

// Parametros mecanicos (tomados de ../control, lab 4)
#define ENCODER_PPR         11.0f   // Pulsos por revolucion motor
#define GEAR_RATIO          20.0f   // Reduccion 20:1
#define QUAD_MULTIPLIER     4.0f    // Modo x4
#define COUNTS_PER_REV      (ENCODER_PPR * GEAR_RATIO * QUAD_MULTIPLIER)
#define PINION_RADIUS_MM    9.4f
#define MM_PER_COUNT        ((2.0f * (float)M_PI * PINION_RADIUS_MM) / COUNTS_PER_REV)

// Muestreo
#define SAMPLE_PERIOD_MS    10
#define LOG_DECIMATION      1       // telemetria cada 10 ms
#define VEL_EMA_ALPHA       0.3f

#define X_LIMIT_DEFAULT_MM  40.0f

// Planta identificada (ver identificacion/ y discretizacion/)
// Entrada: PWM efectivo u_ef [%] (sin zona muerta). Salida: x [mm], derecha +
#define PLANT_DIR           -1.0f   // u > 0 (derecha) da cuentas negativas del encoder
#define DEADZONE_POS_PCT    42.1f   // zona muerta hacia la derecha
#define DEADZONE_NEG_PCT    55.3f   // zona muerta hacia la izquierda

// G(s) = GS_B0 / (s^2 + GS_A1 s + GS_A0)
#define GS_B0               168.4f
#define GS_A1               46.87f
#define GS_A0               622.9f

// G(z) = (GZ_B1 z^-1 + GZ_B2 z^-2) / (1 + GZ_A1 z^-1 + GZ_A2 z^-2), ZOH con T = 0.01 s
#define GZ_T_MS             10
#define GZ_B1               0.007209f
#define GZ_B2               0.006166f
#define GZ_A1               (-1.576343f)
#define GZ_A2               0.625815f

_Static_assert(SAMPLE_PERIOD_MS == GZ_T_MS, "G(z) se discretizo con T = 10 ms");

// Controladores (ver simulacion/parametros.m). Lazo cerrado deseado: zeta = 1, ts = 0.3 s
// C(z) = (q0 + q1 z^-1 + q2 z^-2) / ((1 - z^-1)(1 + s0 z^-1))
// u(k) = (1 - s0) u(k-1) + s0 u(k-2) + q0 e(k) + q1 e(k-1) + q2 e(k-2)
// Continuo-discreto: PID (Kp=7.9136, Ki=70.3792, Kd=0.2759) con Euler hacia atras, s0 = 0
#define CD_Q0               36.208476f
#define CD_Q1               (-63.095738f)
#define CD_Q2               27.591053f
#define CD_S0               0.0f
// Discreto-discreto: asignacion de polos sobre G(z)
#define DD_Q0               8.523371f
#define DD_Q1               (-14.371015f)
#define DD_Q2               6.123475f
#define DD_S0               (-0.262282f)

#define U_EF_MAX            (100.0f - DEADZONE_POS_PCT)    // saturacion de u_ef
#define U_EF_MIN            (-(100.0f - DEADZONE_NEG_PCT))
#define U_MEM_LIMIT         300.0f  // cota de u sin saturar guardado (evita crecer sin limite)
#define DZ_EPS_PCT          1.0f    // |u_ef| menor a esto no compensa la zona muerta

typedef enum {
    CTRL_MANUAL = 0,
    CTRL_CD = 1,            // continuo-discreto
    CTRL_DD = 2,            // discreto-discreto
} ctrl_mode_t;

typedef struct {
    float q0, q1, q2, s0;
} ctrl_coef_t;

static const char *TAG = "lab5";

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_DERECHA,
    MOTOR_IZQUIERDA,
} motor_dir_t;

static pcnt_unit_handle_t s_enc = NULL;
static volatile int s_zero_counts = 0;
static volatile float s_u_pct = 0.0f;         // PWM con signo aplicado
static volatile int s_pulse_ms_left = 0;      // >0: pulso en curso
static volatile float s_x_limit_mm = X_LIMIT_DEFAULT_MM;

static void motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num = PIN_ENB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

// pct: 0..100
static void motor_set(motor_dir_t dir, int pct)
{
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    switch (dir) {
    case MOTOR_DERECHA:
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
        break;
    case MOTOR_IZQUIERDA:
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
        break;
    default:
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 0);
        pct = 0;
        break;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (PWM_MAX * pct) / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// u: -100..100, + = derecha
static void motor_apply(float u)
{
    if (u > 100.0f) u = 100.0f;
    if (u < -100.0f) u = -100.0f;
    s_u_pct = u;
    if (u > 0.0f) {
        motor_set(MOTOR_DERECHA, (int)lroundf(u));
    } else if (u < 0.0f) {
        motor_set(MOTOR_IZQUIERDA, (int)lroundf(-u));
    } else {
        motor_set(MOTOR_STOP, 0);
    }
}

static esp_err_t encoder_init(void)
{
    // accum_count: el driver acumula al llegar a los limites, get_count da el total
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &s_enc), TAG, "Error creando unidad PCNT");

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_GLITCH_FILTER_NS,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_enc, &filter_config), TAG, "Error filtro glitch");

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_enc, &chan_a_config, &chan_a), TAG, "Error canal A");

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_enc, &chan_b_config, &chan_b), TAG, "Error canal B");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "Error edge A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "Error level A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE), TAG, "Error edge B");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "Error level B");

    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_enc, PCNT_HIGH_LIMIT), TAG, "Error watch point alto");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_enc, PCNT_LOW_LIMIT), TAG, "Error watch point bajo");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_enc), TAG, "Error habilitando PCNT");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_enc), TAG, "Error limpiando contador");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_enc), TAG, "Error iniciando PCNT");
    return ESP_OK;
}

static int encoder_counts(void)
{
    int c = 0;
    pcnt_unit_get_count(s_enc, &c);
    return c - s_zero_counts;
}

static void encoder_zero(void)
{
    int c = 0;
    pcnt_unit_get_count(s_enc, &c);
    s_zero_counts = c;
}

// PWM con signo -> PWM efectivo (descuenta la zona muerta de cada lado)
static float plant_u_ef(float u)
{
    if (u > DEADZONE_POS_PCT) return u - DEADZONE_POS_PCT;
    if (u < -DEADZONE_NEG_PCT) return u + DEADZONE_NEG_PCT;
    return 0.0f;
}

// Ecuacion en diferencias de G(z): x(k) = -a1 x(k-1) - a2 x(k-2) + b1 u(k-1) + b2 u(k-2)
typedef struct {
    float x1, x2;   // x(k-1), x(k-2) [mm, derecha +]
    float u1, u2;   // u_ef(k-1), u_ef(k-2) [%]
} plant_model_t;

static float plant_model_step(plant_model_t *m, float u_pct)
{
    float x = -GZ_A1 * m->x1 - GZ_A2 * m->x2 + GZ_B1 * m->u1 + GZ_B2 * m->u2;
    m->x2 = m->x1;
    m->x1 = x;
    m->u2 = m->u1;
    m->u1 = plant_u_ef(u_pct);   // u aplicado durante el siguiente periodo
    return x;
}

static volatile bool s_model_reset = false;

// Control en lazo cerrado
static volatile ctrl_mode_t s_mode = CTRL_MANUAL;
static volatile bool s_ctrl_reset = false;
static volatile bool s_dz_comp = true;      // compensar la zona muerta
static volatile float s_r_mm = 0.0f;        // referencia [mm, derecha +]
static ctrl_coef_t s_coef[3] = {
    [CTRL_CD] = { CD_Q0, CD_Q1, CD_Q2, CD_S0 },
    [CTRL_DD] = { DD_Q0, DD_Q1, DD_Q2, DD_S0 },
};

typedef struct {
    float e1, e2;   // e(k-1), e(k-2)
    float u1, u2;   // u(k-1), u(k-2) sin saturar
} ctrl_state_t;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Ecuacion en diferencias del controlador. Devuelve u_ef saturado.
// Guarda u sin saturar: el controlador es lineal y solo se satura la salida (igual que en Simulink).
static float ctrl_step(ctrl_state_t *st, const ctrl_coef_t *c, float e)
{
    float u = (1.0f - c->s0) * st->u1 + c->s0 * st->u2
            + c->q0 * e + c->q1 * st->e1 + c->q2 * st->e2;
    u = clampf(u, -U_MEM_LIMIT, U_MEM_LIMIT);
    st->e2 = st->e1;
    st->e1 = e;
    st->u2 = st->u1;
    st->u1 = u;
    return clampf(u, U_EF_MIN, U_EF_MAX);
}

// u_ef -> PWM con signo, sumando la zona muerta del lado correspondiente
static float dz_compensate(float u_ef)
{
    if (!s_dz_comp) return u_ef;
    if (u_ef > DZ_EPS_PCT) return u_ef + DEADZONE_POS_PCT;
    if (u_ef < -DZ_EPS_PCT) return u_ef - DEADZONE_NEG_PCT;
    return 0.0f;
}

static void sample_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    plant_model_t model = {0};
    ctrl_state_t st = {0};
    int prev_counts = encoder_counts();
    float v_filt = 0.0f;
    float u_ef = 0.0f;
    int log_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        int counts = encoder_counts();
        float y_mm = PLANT_DIR * counts * MM_PER_COUNT;      // derecha +
        float v_raw = PLANT_DIR * (counts - prev_counts) * MM_PER_COUNT * (1000.0f / SAMPLE_PERIOD_MS);
        prev_counts = counts;
        v_filt += VEL_EMA_ALPHA * (v_raw - v_filt);

        float r = s_r_mm;
        float e = r - y_mm;
        ctrl_mode_t mode = s_mode;

        if (s_ctrl_reset) {
            s_ctrl_reset = false;
            st = (ctrl_state_t){ .e1 = e, .e2 = e };     // sin salto en la derivada al activar
        }

        if (mode != CTRL_MANUAL) {
            u_ef = ctrl_step(&st, &s_coef[mode], e);
            motor_apply(dz_compensate(u_ef));
        } else {
            u_ef = plant_u_ef(s_u_pct);
        }

        if (s_model_reset) {
            s_model_reset = false;
            model = (plant_model_t){ .x1 = y_mm, .x2 = y_mm };
        }
        float y_mod_mm = plant_model_step(&model, s_u_pct);

        // Fin de pulso
        if (s_pulse_ms_left > 0) {
            s_pulse_ms_left -= SAMPLE_PERIOD_MS;
            if (s_pulse_ms_left <= 0) {
                s_pulse_ms_left = 0;
                motor_apply(0.0f);
            }
        }

        // Limite de seguridad: solo deja mover hacia el centro
        if (fabsf(y_mm) > s_x_limit_mm && y_mm * s_u_pct > 0.0f) {
            s_mode = CTRL_MANUAL;
            s_pulse_ms_left = 0;
            motor_apply(0.0f);
            printf("ERR,limite %.2f mm: control apagado\n", y_mm);
        }

        if (++log_div >= LOG_DECIMATION) {
            log_div = 0;
            printf("D,%lld,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%.3f,%d\n",
                   esp_timer_get_time() / 1000, r, y_mm, e, u_ef, s_u_pct, v_filt, y_mod_mm, (int)mode);
        }
    }
}

static void print_cfg(void)
{
    const ctrl_coef_t *a = &s_coef[CTRL_CD], *b = &s_coef[CTRL_DD];
    printf("CFG,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%d,%.1f\n",
           SAMPLE_PERIOD_MS, a->q0, a->q1, a->q2, a->s0, b->q0, b->q1, b->q2, b->s0,
           DEADZONE_POS_PCT, DEADZONE_NEG_PCT, U_EF_MAX, U_EF_MIN, (int)s_dz_comp, s_x_limit_mm);
}

static void set_mode(ctrl_mode_t m)
{
    s_pulse_ms_left = 0;
    if (m == CTRL_MANUAL) motor_apply(0.0f);
    s_ctrl_reset = true;
    s_mode = m;
}

static void command_task(void *arg)
{
    char line[64];
    int len = 0;

    while (1) {
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY) != 1) continue;
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (len < (int)sizeof(line) - 1) line[len++] = (char)ch;
            continue;
        }
        line[len] = '\0';
        len = 0;

        float a = 0, b = 0, c = 0, d = 0;
        int n = 0;
        if (strcmp(line, "ZERO") == 0) {
            encoder_zero();
            s_model_reset = true;
            s_ctrl_reset = true;
            printf("ACK,ZERO\n");
        } else if (strcmp(line, "S") == 0) {
            set_mode(CTRL_MANUAL);
            printf("ACK,STOP\n");
        } else if (strcmp(line, "CFG") == 0) {
            print_cfg();
        } else if (sscanf(line, "R %f", &a) == 1) {
            s_r_mm = a;
            printf("ACK,R,%.3f\n", a);
        } else if (sscanf(line, "C %d", &n) == 1 && n >= CTRL_MANUAL && n <= CTRL_DD) {
            set_mode((ctrl_mode_t)n);
            printf("ACK,C,%d\n", n);
        } else if (sscanf(line, "Q %d %f %f %f %f", &n, &a, &b, &c, &d) == 5 && (n == CTRL_CD || n == CTRL_DD)) {
            s_coef[n] = (ctrl_coef_t){ a, b, c, d };
            s_ctrl_reset = true;
            printf("ACK,Q,%d\n", n);
            print_cfg();
        } else if (sscanf(line, "DZ %d", &n) == 1) {
            s_dz_comp = n != 0;
            printf("ACK,DZ,%d\n", (int)s_dz_comp);
        } else if (sscanf(line, "U %f", &a) == 1) {
            set_mode(CTRL_MANUAL);
            motor_apply(a);
            printf("ACK,U,%.1f\n", s_u_pct);
        } else if (sscanf(line, "P %f %f", &a, &b) == 2 && b > 0) {
            set_mode(CTRL_MANUAL);
            motor_apply(a);
            s_pulse_ms_left = (int)b;
            printf("ACK,P,%.1f,%d\n", s_u_pct, (int)b);
        } else if (sscanf(line, "LIM %f", &a) == 1 && a > 0) {
            s_x_limit_mm = a;
            printf("ACK,LIM,%.1f\n", a);
        } else {
            printf("ERR,cmd '%s'\n", line);
        }
    }
}

void app_main(void)
{
    // UART0 con driver: printf y lectura de comandos por el mismo puerto
    // Buffer de TX: printf no bloquea la tarea de muestreo
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 4096, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);
    setvbuf(stdout, NULL, _IONBF, 0);

    motor_init();
    ESP_ERROR_CHECK(encoder_init());
    vTaskDelay(pdMS_TO_TICKS(100));
    encoder_zero();     // posicion inicial = 0 mm

    ESP_LOGI(TAG, "ENB=%d IN1=%d IN2=%d | ENC A=%d B=%d | %.4f mm/cuenta",
             PIN_ENB, PIN_IN1, PIN_IN2, ENCODER_GPIO_A, ENCODER_GPIO_B, MM_PER_COUNT);

    xTaskCreate(sample_task, "sample", 4096, NULL, 10, NULL);
    xTaskCreate(command_task, "cmd", 4096, NULL, 5, NULL);
    print_cfg();
}
