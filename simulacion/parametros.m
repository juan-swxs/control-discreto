% Parametros del Lab 5: planta, especificaciones y controladores.
% Lo ejecuta el modelo de Simulink al iniciar (InitFcn) y lo usan diseno.m y simular.m.

%% Planta identificada: G(s) = b0 / (s^2 + a1 s + a0)   [mm / % PWM efectivo]
b0 = 168.4;  a1 = 46.87;  a0 = 622.9;
Gs = tf(b0, [1 a1 a0]);
T  = 0.01;                          % tiempo de muestreo [s]
Gz = c2d(Gs, T, 'zoh');
[numGz, denGz] = tfdata(Gz, 'v');   % [0 b1 b2], [1 a1z a2z]

% Actuador y sensor
U0_POS = 42.1;  U0_NEG = 55.3;      % zona muerta [%]
U_MAX  = 100 - U0_POS;              % u_ef maximo hacia la derecha
U_MIN  = -(100 - U0_NEG);           % u_ef minimo hacia la izquierda
Q_ENC  = 2*pi*9.4/880;              % resolucion del encoder [mm/cuenta]

%% Especificaciones del lazo cerrado (iguales para ambas tecnicas)
zeta_d = 1;                         % sin sobrepico
ts_d   = 0.30;                      % tiempo de establecimiento deseado (2 %) [s]
wn_d   = 4/(zeta_d*ts_d);
% Polo no dominante: 5*zeta*wn. Con 10*zeta*wn (practica 3) p3*T = 1.33 y Euler
% deforma el lazo (Mp = 28.6 %); con 5*zeta*wn, p3*T = 0.67 y el discreto conserva
% la respuesta continua con la mitad de ganancia.
p3     = 5*zeta_d*wn_d;

%% 1) Continuo-discreto: PID continuo por asignacion de polos + Euler hacia atras
% s(s^2+a1 s+a0) + b0(Kd s^2+Kp s+Ki) = (s^2+2 zeta wn s+wn^2)(s+p3)
Pd_s = conv([1 2*zeta_d*wn_d wn_d^2], [1 p3]);
Kd = (Pd_s(2) - a1)/b0;
Kp = (Pd_s(3) - a0)/b0;
Ki =  Pd_s(4)/b0;
Cs = tf([Kd Kp Ki], [1 0]);
% s = (1 - z^-1)/T  ->  C(z) = (q0 + q1 z^-1 + q2 z^-2)/(1 - z^-1)
q0_cd = Kd/T + Kp + Ki*T;
q1_cd = -2*Kd/T - Kp;
q2_cd = Kd/T;
s0_cd = 0;                          % denominador (1 - z^-1)(1 + s0 z^-1) con s0 = 0
Cz_cd = tf([q0_cd q1_cd q2_cd], [1 -1 0], T, 'Variable', 'z^-1');

%% 2) Discreto-discreto: C(z) = (q0+q1 z^-1+q2 z^-2)/((1-z^-1)(1+s0 z^-1)) sobre G(z)
% Polinomio deseado en z: par dominante mapeado con z = e^{sT} y dos polos en e^{-p3 T}
P1 = -2*exp(-zeta_d*wn_d*T)*cos(wn_d*sqrt(max(0,1-zeta_d^2))*T);
P2 =  exp(-2*zeta_d*wn_d*T);
alfa = exp(-p3*T);
Pd_z = conv([1 P1 P2], conv([1 -alfa], [1 -alfa]));     % 1 + d1 z^-1 + ... + d4 z^-4
% A(z)(1-z^-1)(1+s0 z^-1) + B(z)Q(z) = Pd(z), lineal en [q0 q1 q2 s0]
gb1 = numGz(2); gb2 = numGz(3); ga1 = denGz(2); ga2 = denGz(3);
AD0 = conv([1 ga1 ga2], [1 -1]);                        % A(z)(1-z^-1)
%            q0    q1    q2    s0
M = [       gb1,    0,    0,   AD0(1);                  % z^-1
            gb2,  gb1,    0,   AD0(2);                  % z^-2
              0,  gb2,  gb1,   AD0(3);                  % z^-3
              0,    0,  gb2,   AD0(4)];                 % z^-4
rhs = Pd_z(2:5).' - [AD0(2); AD0(3); AD0(4); 0];
sol = M \ rhs;
q0_dd = sol(1); q1_dd = sol(2); q2_dd = sol(3); s0_dd = sol(4);
Cz_dd = tf([q0_dd q1_dd q2_dd], conv([1 -1], [1 s0_dd]), T, 'Variable', 'z^-1');

%% Simulacion
R1 = 5;  R2 = -5;                   % escalones de referencia [mm]
T_R1 = 0.1;  T_R2 = 1.6;  T_FIN = 3.2;
