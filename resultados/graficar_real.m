% Grafica la prueba en lazo cerrado sobre la planta real (prueba_lazo.py)
here = fileparts(mfilename('fullpath'));
D = readmatrix(fullfile(here, 'prueba_lazo_cerrado_preliminar.csv'));
% modo_prueba,t_ms,r_mm,y_mm,e_mm,u_ef,u_pwm,v_mm_s,y_mod_mm,modo
nombres = {'Continuo-discreto', 'Discreto-discreto'};
f = figure('Visible', 'off', 'Position', [0 0 900 620]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
for m = 1:2
    d = D(D(:,1) == m, :);
    t = (d(:,2) - d(1,2))/1000;
    nexttile(m);
    stairs(t, d(:,3), 'k--'); hold on; plot(t, d(:,4), 'LineWidth', 1.3, 'Color', [0.85 0.45 0.05]);
    grid on; ylabel('mm'); title(nombres{m}); legend('r', 'y medida', 'Location', 'southwest');
    nexttile(m + 2);
    stairs(t, d(:,6), 'LineWidth', 1.1, 'Color', [0.05 0.55 0.6]); hold on
    stairs(t, d(:,7), ':', 'Color', [0.4 0.4 0.4]);
    grid on; ylabel('%'); xlabel('t [s]'); legend('u_{ef}', 'PWM', 'Location', 'southwest'); ylim([-105 105]);
end
title(tl, 'Prueba en la planta real (T = 10 ms)');
exportgraphics(f, fullfile(here, '..', 'discretizacion', 'real_lazo.pdf'), 'ContentType', 'vector');
