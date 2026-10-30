% Identificacion con zona muerta compensada: u_eff = sign(u)*(|u|-u0) -> x [mm], derecha = +
warning('off', 'all');
here = fileparts(mfilename('fullpath'));
D = readmatrix(fullfile(here, 'escalones.csv'));   % test,u_step,t_ms,counts,x_mm,v_mm_s,u_pct
Ts = 0.01;
tests = unique(D(:,1))';

% Equilibrio vs PWM y zona muerta por lado
U = []; XSS = [];
for k = tests
    d = D(D(:,1)==k,:);
    t = (d(:,3)-d(1,3))/1000; x = -(d(:,5)-d(1,5)); on = d(:,7) ~= 0;
    tOn = t(find(on,1));
    U(end+1) = d(1,2); XSS(end+1) = mean(x(on & t > tOn+1.1)); %#ok<SAGROW>
end
u0 = zeros(1,2); Kss = zeros(1,2); side = [1 -1];
for i = 1:2
    m = sign(U)==side(i);
    p = polyfit(abs(U(m)), abs(XSS(m)), 1);
    Kss(i) = p(1); u0(i) = -p(2)/p(1);
end
fprintf('Zona muerta: derecha u0 = %.1f %%, izquierda u0 = %.1f %%\n', u0(1), u0(2));
fprintf('Ganancia estatica: derecha %.4f mm/%%, izquierda %.4f mm/%%\n', Kss(1), Kss(2));

% Datos por escalon: 0.3 s previos en cero + fase energizada
ed = {}; lbl = {};
for k = tests
    d = D(D(:,1)==k,:);
    x = -(d(:,5)-d(1,5)); u = d(:,7);
    last = find(u ~= 0, 1, 'last');
    i = 1 + (d(1,2) < 0);
    ue = sign(u) .* max(abs(u) - u0(i), 0);
    ed{end+1} = iddata(x(1:last), ue(1:last), Ts); %#ok<SAGROW>
    lbl{end+1} = sprintf('%+d%%', d(1,2)); %#ok<SAGROW>
end
E = merge(ed{:});

opt = tfestOptions('InitialCondition', 'zero', 'EnforceStability', true);
G1  = tfest(E, 1, 0, NaN, opt);   % primer orden + retardo
G2  = tfest(E, 2, 0, opt);        % segundo orden sin ceros (modelo fisico)
G2d = tfest(E, 2, 0, NaN, opt);   % segundo orden + retardo

fprintf('\n--- G2(s): segundo orden (u_eff -> x) ---\n'); G2 = tf(G2); display(G2)
[wn, z] = damp(G2);
fprintf('K = %.4f mm/%%   wn = %.2f rad/s   zeta = %.3f   polos = %s\n', dcgain(G2), wn(1), z(1), num2str(pole(G2).', '%.2f '));
fprintf('\n--- G2d(s): segundo orden + retardo L = %.3f s ---\n', G2d.IODelay); display(tf(G2d))
[wn, z] = damp(G2d);
fprintf('K = %.4f   wn = %.2f   zeta = %.3f   polos = %s\n', dcgain(G2d), wn(1), z(1), num2str(pole(G2d).', '%.2f '));
fprintf('\n--- G1(s): primer orden + retardo ---\n'); display(tf(G1))
fprintf('K = %.4f  tau = %.4f s  L = %.3f s\n', dcgain(G1), -1/pole(G1), G1.IODelay);

fprintf('\nFIT %% por escalon        G1     G2    G2d\n');
for i = 1:numel(ed)
    [~,f1] = compare(ed{i}, G1); [~,f2] = compare(ed{i}, G2); [~,f3] = compare(ed{i}, G2d);
    fprintf('  %-6s               %5.1f  %5.1f  %5.1f\n', lbl{i}, f1, f2, f3);
end

% Graficas para el informe
f = figure('Visible','off','Position',[0 0 1400 800]);
tl = tiledlayout(f, 2, 5, 'TileSpacing','compact');
for i = 1:numel(ed)
    nexttile; y = ed{i}.y; t = (0:numel(y)-1)'*Ts;
    ym = lsim(G2d, ed{i}.u, t);
    plot(t, y, 'k.', t, ym, 'r-', 'LineWidth', 1.2); grid on
    title(['u = ' lbl{i}]); xlabel('t [s]'); ylabel('x [mm]');
end
title(tl, 'Escalones medidos (puntos) vs modelo G2d (linea)');
exportgraphics(f, fullfile(here, 'ajuste_escalones.png'), 'Resolution', 110);

f2 = figure('Visible','off');
uu = linspace(0,100,200);
plot(abs(U(U>0)), XSS(U>0), 'bo', abs(U(U<0)), -XSS(U<0), 'rs', ...
     uu, max(Kss(1)*(uu-u0(1)),0), 'b--', uu, max(Kss(2)*(uu-u0(2)),0), 'r--'); grid on
legend('derecha','izquierda','ajuste der.','ajuste izq.','Location','northwest');
xlabel('|u| [%]'); ylabel('|x_{ss}| [mm]'); title('Equilibrio vs PWM (zona muerta)');
exportgraphics(f2, fullfile(here, 'equilibrio_pwm.png'), 'Resolution', 110);

save(fullfile(here, 'ident.mat'), 'G1', 'G2', 'G2d', 'u0', 'Kss', 'U', 'XSS', 'E');
