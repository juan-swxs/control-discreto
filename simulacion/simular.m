% Simula lab5_control.slx, calcula metricas e ISE, guarda figuras y valores para el LaTeX
here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'discretizacion');
run(fullfile(here, 'parametros.m'));
mdl = 'lab5_control';
load_system(fullfile(here, [mdl '.slx']));
out = sim(mdl);

tags = {'c', 'cd', 'dd'};
nombres = {'PID continuo', 'Continuo-discreto', 'Discreto-discreto'};
rr = out.r; t = rr.Time; r = rr.Data(:);
M = struct();
for i = 1:3
    y = out.(['y_' tags{i}]).Data(:); u = out.(['u_' tags{i}]).Data(:);
    e = out.(['e_' tags{i}]).Data(:); ise = out.(['ise_' tags{i}]).Data(:);
    for s = 1:2
        if s == 1, ta = T_R1; tb = T_R2; x0 = 0;  rf = R1;
        else,      ta = T_R2; tb = T_FIN; x0 = R1; rf = R2; end
        k = t >= ta & t < tb; tk = t(k) - ta; yk = y(k);
        A = rf - x0; sg = sign(A);
        mp = max(0, max(sg*(yk - rf)))/abs(A)*100;
        fuera = find(abs(yk - rf) > 0.02*abs(A), 1, 'last');
        if isempty(fuera), ts = 0; elseif fuera == numel(yk), ts = NaN; else, ts = tk(fuera+1); end
        ess = rf - mean(yk(end-50:end));
        ise_s = interp1(t, ise, tb - 1e-3) - interp1(t, ise, ta);
        M(i,s).mp = mp; M(i,s).ts = ts; M(i,s).ess = ess; M(i,s).ise = ise_s;
        M(i,s).umax = max(abs(u(k)));
    end
    fprintf('%-18s | +5 mm: ts=%.3f Mp=%5.2f%% ess=%+.3f ISE=%.3f umax=%.1f | -10 mm: ts=%.3f Mp=%5.2f%% ess=%+.3f ISE=%.3f | ISE total=%.3f\n', ...
        nombres{i}, M(i,1).ts, M(i,1).mp, M(i,1).ess, M(i,1).ise, M(i,1).umax, ...
        M(i,2).ts, M(i,2).mp, M(i,2).ess, M(i,2).ise, ise(end));
end

%% Figuras
col = {[0.15 0.15 0.15], [0.85 0.45 0.05], [0.05 0.55 0.6]};
fig = figure('Visible', 'off', 'Position', [0 0 900 760]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact');
nexttile; plot(t, r, 'k--', 'LineWidth', 1); hold on
for i = 1:3, plot(t, out.(['y_' tags{i}]).Data(:), 'Color', col{i}, 'LineWidth', 1.3); end
ylabel('x [mm]'); grid on; legend(['r', nombres], 'Location', 'northeast'); title('Posicion');
nexttile; hold on
for i = 1:3, stairs(t, out.(['u_' tags{i}]).Data(:), 'Color', col{i}, 'LineWidth', 1.1); end
yline(U_MAX, ':'); yline(U_MIN, ':'); ylabel('u_{ef} [%]'); grid on; title('Accion de control (saturada en los lazos discretos)');
ylim([U_MIN-10 U_MAX+10]);
nexttile; hold on
for i = 1:3, stairs(t, out.(['e_' tags{i}]).Data(:), 'Color', col{i}, 'LineWidth', 1.1); end
ylabel('e [mm]'); xlabel('t [s]'); grid on; title('Error');
exportgraphics(fig, fullfile(outDir, 'sim_respuestas.pdf'), 'ContentType', 'vector');

fig2 = figure('Visible', 'off', 'Position', [0 0 700 380]);
hold on
for i = 1:3, plot(t, out.(['ise_' tags{i}]).Data(:), 'Color', col{i}, 'LineWidth', 1.4); end
grid on; xlabel('t [s]'); ylabel('ISE [mm^2 s]'); legend(nombres, 'Location', 'southeast'); title('ISE acumulado');
exportgraphics(fig2, fullfile(outDir, 'sim_ise.pdf'), 'ContentType', 'vector');

% Diagramas de Simulink
open_system(mdl);
print(['-s' mdl], '-dpdf', fullfile(outDir, 'simulink_modelo.pdf'));
print(['-s' mdl '/Discreto-discreto'], '-dpdf', fullfile(outDir, 'simulink_lazo_discreto.pdf'));
close_system(mdl, 0);

%% Valores para el LaTeX
fid = fopen(fullfile(outDir, 'valores.tex'), 'w');
w = @(n, v, d) fprintf(fid, '\\newcommand{\\%s}{%s}\n', n, sprintf(['%.' num2str(d) 'f'], v));
w('tsd', ts_d, 2); w('zetad', zeta_d, 0); w('wnd', wn_d, 4); w('pthree', p3, 4);
w('PdsA', Pd_s(2), 4); w('PdsB', Pd_s(3), 4); w('PdsC', Pd_s(4), 4);
w('Kp', Kp, 4); w('Ki', Ki, 4); w('Kd', Kd, 4);
w('KdT', Kd/T, 4); w('KiT', Ki*T, 4);
w('qcdA', q0_cd, 4); w('qcdB', q1_cd, 4); w('qcdC', q2_cd, 4);
w('PzA', P1, 6); w('PzB', P2, 6); w('alfa', alfa, 6);
w('ezwT', exp(-zeta_d*wn_d*T), 6);
w('dA', Pd_z(2), 6); w('dB', Pd_z(3), 6); w('dC', Pd_z(4), 6); w('dD', Pd_z(5), 6);
w('ADA', AD0(2), 6); w('ADB', AD0(3), 6); w('ADC', AD0(4), 6);
w('qddA', q0_dd, 4); w('qddB', q1_dd, 4); w('qddC', q2_dd, 4); w('sdd', s0_dd, 6);
w('oneMinusSdd', 1 - s0_dd, 6);
w('Umax', U_MAX, 1); w('Umin', U_MIN, 1);
let = 'abc';
for i = 1:3
    for s = 1:2
        pre = sprintf('m%s%s', let(i), char('A' + s - 1));
        w([pre 'ts'], M(i,s).ts, 3); w([pre 'mp'], M(i,s).mp, 2);
        w([pre 'ess'], M(i,s).ess, 3); w([pre 'ise'], M(i,s).ise, 3);
    end
end
fclose(fid);
save(fullfile(here, 'resultados_sim.mat'), 'M', 'out');
fprintf('Figuras y valores.tex en %s\n', outDir);
