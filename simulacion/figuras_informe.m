% Regenera todas las figuras del informe IEEE con fondo blanco en ../informe_ieee/figs.
% Usa los datos guardados: identificacion/ident.mat, simulacion/resultados_sim.mat y los CSV.
here = fileparts(mfilename('fullpath'));
raiz = fullfile(here, '..');
figs = fullfile(raiz, 'informe_ieee', 'figs');
run(fullfile(here, 'parametros.m'));

C = struct('r', [0 0 0], 'c', [0.45 0.45 0.45], 'cd', [0.85 0.40 0.00], 'dd', [0.00 0.45 0.70], ...
           'u', [0.10 0.55 0.35], 'pwm', [0.55 0.55 0.55], 'm', [0.80 0.10 0.10]);
nombres = {'PID continuo (ideal)', 'Continuo-discreto', 'Discreto-discreto'};
tags = {'c', 'cd', 'dd'};
guardar = @(f, nombre) exportgraphics(f, fullfile(figs, nombre), 'ContentType', 'vector', 'BackgroundColor', 'white');

%% 1. Caracterizacion en lazo abierto
D = readmatrix(fullfile(raiz, 'identificacion', 'caracterizacion.csv'));   % t_ms,counts,x_mm,v,u
t = (D(:,1) - D(1,1))/1000;
f = figura_blanca([900 520]);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile; plot(t, -D(:,3), 'Color', C.cd, 'LineWidth', 1.2); grid on
ylabel('y [mm]'); title('Desplazamiento medido (derecha +)');
nexttile; stairs(t, D(:,5), 'Color', C.u, 'LineWidth', 1.1); grid on
ylabel('u [% PWM]'); xlabel('t [s]'); ylim([-110 110]); title('Pulsos de PWM aplicados');
title(tl, 'Caracterizacion en lazo abierto', 'FontWeight', 'bold');
guardar(f, 'caracterizacion.pdf');

%% 2. Identificacion
S = load(fullfile(raiz, 'identificacion', 'ident.mat'));
Gid = tf(b0, [1 a1 a0]);
f = figura_blanca([1100 560]);
tl = tiledlayout(f, 2, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:numel(S.U)
    e = getexp(S.E, i);
    y = e.y; tt = (0:numel(y)-1)'*T;
    nexttile; plot(tt, y, '.', 'Color', C.cd, 'MarkerSize', 6); hold on
    plot(tt, lsim(Gid, e.u, tt), '-', 'Color', C.dd, 'LineWidth', 1.3); grid on
    title(sprintf('u = %+g %%', S.U(i))); xlabel('t [s]');
    if mod(i-1, 5) == 0, ylabel('y [mm]'); end
end
lg = legend('medido', 'modelo G(s)', 'Orientation', 'horizontal'); lg.Layout.Tile = 'south';
title(tl, 'Escalones de identificacion y respuesta del modelo', 'FontWeight', 'bold');
guardar(f, 'ajuste_escalones.pdf');

f = figura_blanca([620 420]);
uu = linspace(0, 100, 200);
plot(abs(S.U(S.U>0)), S.XSS(S.U>0), 'o', 'Color', C.cd, 'MarkerFaceColor', C.cd); hold on
plot(abs(S.U(S.U<0)), -S.XSS(S.U<0), 's', 'Color', C.dd, 'MarkerFaceColor', C.dd);
plot(uu, max(S.Kss(1)*(uu - S.u0(1)), 0), '--', 'Color', C.cd, 'LineWidth', 1.2);
plot(uu, max(S.Kss(2)*(uu - S.u0(2)), 0), '--', 'Color', C.dd, 'LineWidth', 1.2);
xline(S.u0(1), ':', sprintf('u_0^+ = %.1f %%', S.u0(1)), 'Color', C.cd, 'LabelVerticalAlignment', 'middle');
xline(S.u0(2), ':', sprintf('u_0^- = %.1f %%', S.u0(2)), 'Color', C.dd, 'LabelVerticalAlignment', 'middle');
grid on; xlabel('|u| [% PWM]'); ylabel('|y_{ss}| [mm]');
legend('derecha', 'izquierda', 'ajuste derecha', 'ajuste izquierda', 'Location', 'northwest');
title('Equilibrio contra los resortes y zona muerta');
guardar(f, 'equilibrio_pwm.pdf');

%% 3. Discretizacion
f = figura_blanca([700 400]);
[yc, tc] = step(Gs, 0.4); td = 0:T:0.4; yd = step(Gz, td);
plot(tc, yc, '-', 'Color', C.dd, 'LineWidth', 1.5); hold on
stairs(td, yd, '-', 'Color', C.cd, 'LineWidth', 1.1); plot(td, yd, '.', 'Color', C.cd, 'MarkerSize', 10);
grid on; xlabel('t [s]'); ylabel('y [mm] por 1 % de u_{ef}');
legend('G(s) continua', 'G(z) con ZOH, T = 0.01 s', '', 'Location', 'southeast');
title('Respuesta al escalon unitario: planta continua y discreta');
guardar(f, 'escalon_continuo_discreto.pdf');

f = figura_blanca([480 440]);
zplane(numGz, denGz); grid on; title('Polos y cero de G(z)');
guardar(f, 'polos_ceros_z.pdf');

%% 4. Simulacion
R = load(fullfile(here, 'resultados_sim.mat'));
out = R.out; ts_ = out.r.Time; r = out.r.Data(:);
sig = @(n, k) out.([n '_' k]).Data(:);

% 4a. Comparacion de posicion: completa y zoom en cada escalon
f = figura_blanca([1000 640]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
nexttile([1 2]);
plot(ts_, r, '--', 'Color', C.r, 'LineWidth', 1); hold on
for i = 1:3, plot(ts_, sig('y', tags{i}), 'Color', C.(tags{i}), 'LineWidth', 1.4); end
grid on; ylabel('y [mm]'); title('Secuencia completa: +5 mm en t = 0.1 s y -5 mm en t = 1.6 s');
legend(['referencia r', nombres], 'Location', 'northeast');
zooms = {[0.05 0.75], [1.55 2.25]}; tit = {'Zoom escalon 0 \rightarrow +5 mm', 'Zoom escalon +5 \rightarrow -5 mm'};
for z = 1:2
    nexttile; k = ts_ >= zooms{z}(1) & ts_ <= zooms{z}(2);
    plot(ts_(k), r(k), '--', 'Color', C.r); hold on
    for i = 1:3
        yy = sig('y', tags{i});
        plot(ts_(k), yy(k), 'Color', C.(tags{i}), 'LineWidth', 1.4);
    end
    grid on; xlabel('t [s]'); ylabel('y [mm]'); title(tit{z});
end
title(tl, 'Simulacion: posicion de los tres lazos', 'FontWeight', 'bold');
guardar(f, 'sim_posicion.pdf');

% 4b. Una figura por controlador discreto: r e y, u_ef con saturacion, error
for i = 2:3
    f = figura_blanca([900 700]);
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
    nexttile; plot(ts_, r, '--', 'Color', C.r); hold on
    plot(ts_, sig('y', tags{i}), 'Color', C.(tags{i}), 'LineWidth', 1.4); grid on
    ylabel('y [mm]'); legend('r', 'y', 'Location', 'northeast'); title('Posicion');
    nexttile; stairs(ts_, sig('u', tags{i}), 'Color', C.u, 'LineWidth', 1.2); hold on
    yline(U_MAX, ':', sprintf('u_{max} = %.1f %%', U_MAX), 'Color', C.m);
    yline(U_MIN, ':', sprintf('u_{min} = %.1f %%', U_MIN), 'Color', C.m, 'LabelVerticalAlignment', 'bottom');
    grid on; ylabel('u_{ef} [%]'); ylim([U_MIN-12 U_MAX+12]); title('Accion de control (salida saturada)');
    nexttile; stairs(ts_, sig('e', tags{i}), 'Color', C.m, 'LineWidth', 1.1); grid on
    ylabel('e [mm]'); xlabel('t [s]'); title('Error e = r - y');
    title(tl, ['Simulacion del controlador ' lower(nombres{i})], 'FontWeight', 'bold');
    guardar(f, ['sim_' tags{i} '.pdf']);
end

% 4c. Accion de control comparada (PID ideal sin saturar en eje propio)
f = figura_blanca([900 520]);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile; hold on
for i = 2:3, stairs(ts_, sig('u', tags{i}), 'Color', C.(tags{i}), 'LineWidth', 1.2); end
yline(U_MAX, ':', 'Color', C.m); yline(U_MIN, ':', 'Color', C.m);
grid on; ylabel('u_{ef} [%]'); ylim([U_MIN-12 U_MAX+12]); legend(nombres(2:3), 'Location', 'northeast');
title('Lazos discretos (saturados a la zona util del PWM)');
nexttile; plot(ts_, sig('u', 'c'), 'Color', C.c, 'LineWidth', 1.2); grid on
ylabel('u [%]'); xlabel('t [s]'); title('PID continuo ideal (sin saturacion): picos por el derivativo');
title(tl, 'Simulacion: accion de control', 'FontWeight', 'bold');
guardar(f, 'sim_control.pdf');

% 4d. ISE acumulado
f = figura_blanca([800 400]); hold on
for i = 1:3, plot(ts_, sig('ise', tags{i}), 'Color', C.(tags{i}), 'LineWidth', 1.5); end
xline(T_R2, ':', 'escalon a -5 mm', 'LabelVerticalAlignment', 'bottom');
grid on; xlabel('t [s]'); ylabel('ISE [mm^2 s]'); legend(nombres, 'Location', 'northwest');
title('ISE acumulado = \int e^2 dt');
guardar(f, 'sim_ise.pdf');

%% 5. Diagramas de Simulink (modelo y cada subsistema)
mdl = 'lab5_control';
load_system(fullfile(here, [mdl '.slx']));
open_system(mdl);
bloques = {mdl, [mdl '/PID continuo'], [mdl '/Continuo-discreto'], [mdl '/Discreto-discreto']};
arch = {'simulink_modelo.pdf', 'simulink_pid_continuo.pdf', 'simulink_lazo_cd.pdf', 'simulink_lazo_dd.pdf'};
for i = 1:4
    print(['-s' bloques{i}], '-dpdf', fullfile(figs, arch{i}));
end
close_system(mdl, 0);

%% 6. Planta real
D = readmatrix(fullfile(raiz, 'resultados', 'prueba_lazo_cerrado_preliminar.csv'));
f = figura_blanca([1000 620]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
for m = 1:2
    d = D(D(:,1) == m, :); tt = (d(:,2) - d(1,2))/1000;
    nexttile(m); stairs(tt, d(:,3), '--', 'Color', C.r); hold on
    plot(tt, d(:,4), 'Color', C.(tags{m+1}), 'LineWidth', 1.4); grid on
    ylabel('y [mm]'); title(nombres{m+1}); legend('r', 'y medida', 'Location', 'southwest');
    nexttile(m + 2); stairs(tt, d(:,7), ':', 'Color', C.pwm, 'LineWidth', 1); hold on
    stairs(tt, d(:,6), 'Color', C.u, 'LineWidth', 1.2); grid on
    ylabel('[%]'); xlabel('t [s]'); ylim([-105 105]); legend('PWM aplicado', 'u_{ef}', 'Location', 'southwest');
end
title(tl, 'Prueba en la planta real (T = 10 ms)', 'FontWeight', 'bold');
guardar(f, 'real_lazo.pdf');
fprintf('Figuras listas en %s\n', figs);

function f = figura_blanca(pos)
    f = figure('Visible', 'off', 'Position', [0 0 pos], 'Color', 'w');
    try, theme(f, 'light'); catch, end
end
