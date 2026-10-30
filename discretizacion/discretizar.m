% Discretizacion de la planta pinon-cremallera por retenedor de orden cero (ZOH)
% G(s) = b0 / (s^2 + a1 s + a0)   [mm / %PWM efectivo], identificada en ../identificacion
here = fileparts(mfilename('fullpath'));
fmt = @(x) sprintf('%.6f', x);

b0 = 168.4; a1 = 46.87; a0 = 622.9;
Gs = tf(b0, [1 a1 a0]);

%% 1. Parametros de la planta continua
wn   = sqrt(a0);
zeta = a1/(2*wn);
K    = b0/a0;
a    = a1/2;                 % parte real de los polos (s = -a +- j w)
w    = sqrt(a0 - a^2);       % parte imaginaria
ts   = 4/(zeta*wn);          % tiempo de establecimiento 2 %
fprintf('wn=%s zeta=%s K=%s a=%s w=%s ts=%s\n', fmt(wn), fmt(zeta), fmt(K), fmt(a), fmt(w), fmt(ts));

%% 2. Tiempo de muestreo
fprintf('Landau: %s <= T <= %s\n', fmt(0.25/wn), fmt(1.5/wn));
fprintf('Lazo cerrado ts/20..ts/10: %s .. %s\n', fmt(ts/20), fmt(ts/10));
fprintf('Lazo abierto ts/80..ts/40: %s .. %s\n', fmt(ts/80), fmt(ts/40));
T = 0.01;
fprintf('T=%.3f -> wn*T=%s  ts/T=%s\n', T, fmt(wn*T), fmt(ts/T));

%% 3. Fracciones parciales de G(s)/s = A/s + B(s+a)/((s+a)^2+w^2) + C w/((s+a)^2+w^2)
A = b0/a0;  B = -A;  C = -a*A/w;
fprintf('A=%s B=%s C=%s\n', fmt(A), fmt(B), fmt(C));

%% 4. Terminos de la tabla
e  = exp(-a*T); c = cos(w*T); s = sin(w*T);
p1 = -2*e*c;  p2 = e^2;
fprintf('e^-aT=%s  wT=%s  cos=%s  sin=%s\n', fmt(e), fmt(w*T), fmt(c), fmt(s));
fprintf('e*cos=%s  e*sin=%s  2e*cos=%s  e^-2aT=%s\n', fmt(e*c), fmt(e*s), fmt(2*e*c), fmt(p2));

%% 5. G(z) = (b1 z^-1 + b2 z^-2) / (1 + p1 z^-1 + p2 z^-2)
b1 = A*(1 - e*c) + C*e*s;
b2 = A*(e^2 - e*c) - C*e*s;
fprintf('A(1-e cos)=%s  C e sin=%s  A(e^2-e cos)=%s\n', fmt(A*(1-e*c)), fmt(C*e*s), fmt(A*(e^2-e*c)));
fprintf('b1=%s b2=%s p1=%s p2=%s\n', fmt(b1), fmt(b2), fmt(p1), fmt(p2));
Gz = tf([0 b1 b2], [1 p1 p2], T, 'Variable', 'z^-1');

%% 6. Verificacion
Gc = c2d(Gs, T, 'zoh');
[nc, dc] = tfdata(Gc, 'v');
fprintf('c2d: num=[%s] den=[%s]\n', num2str(nc, '%.6f '), num2str(dc, '%.6f '));
fprintf('Ganancia DC: G(s)=%s  G(z=1)=%s\n', fmt(dcgain(Gs)), fmt((b1+b2)/(1+p1+p2)));
pz = roots([1 p1 p2]);
fprintf('Polos z: %s  |z|=%s  (exp(sT): %s)\n', num2str(pz.', '%.6f '), fmt(abs(pz(1))), num2str(exp(pole(Gs)*T).', '%.6f '));
fprintf('Cero z: %s\n', fmt(-b2/b1));

%% Ejemplo de clase (F(s)=3/(s^2+8s+20), T=0.15) con la misma formula
Ae = 3/20; ae = 4; we = 2; Ce = -ae*Ae/we; Te = 0.15;
ee = exp(-ae*Te); ce = cos(we*Te); se = sin(we*Te);
fprintf('Ejemplo: b1=%s b2=%s p1=%s p2=%s\n', fmt(Ae*(1-ee*ce)+Ce*ee*se), fmt(Ae*(ee^2-ee*ce)-Ce*ee*se), fmt(-2*ee*ce), fmt(ee^2));
[ne, de] = tfdata(c2d(tf(3,[1 8 20]), Te, 'zoh'), 'v');
fprintf('Ejemplo c2d: num=[%s] den=[%s]\n', num2str(ne, '%.6f '), num2str(de, '%.6f '));

%% Figuras
t = 0:T:0.4;
f = figure('Visible', 'off', 'Position', [0 0 760 420]);
[yc, tc] = step(Gs, 0.4);
yd = step(Gz, t);
plot(tc, yc, 'b-', 'LineWidth', 1.4); hold on
stairs(t, yd, 'r-', 'LineWidth', 1.2); plot(t, yd, 'r.', 'MarkerSize', 10);
grid on; xlabel('t [s]'); ylabel('x [mm] por 1 % de u_{ef}');
legend('G(s) continua', 'G(z) ZOH, T = 0.01 s', 'Location', 'southeast');
title('Respuesta al escalon unitario');
exportgraphics(f, fullfile(here, 'escalon_continuo_discreto.pdf'), 'ContentType', 'vector');

f2 = figure('Visible', 'off', 'Position', [0 0 520 480]);
zplane([b1 b2], [1 p1 p2]); grid on; title('Polos y ceros de G(z)');
exportgraphics(f2, fullfile(here, 'polos_ceros_z.pdf'), 'ContentType', 'vector');

save(fullfile(here, 'discretizacion.mat'), 'Gs', 'Gz', 'T', 'b1', 'b2', 'p1', 'p2');
