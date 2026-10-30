% Diseno y verificacion lineal de los dos controladores (sin saturacion ni cuantizacion)
here = fileparts(mfilename('fullpath'));
run(fullfile(here, 'parametros.m'));

fprintf('Especificacion: zeta=%.2f ts=%.3f s wn=%.4f p3=%.4f\n', zeta_d, ts_d, wn_d, p3);
fprintf('\n== Continuo-discreto ==\n');
fprintf('Pd(s) = s^3 + %.4f s^2 + %.4f s + %.4f\n', Pd_s(2), Pd_s(3), Pd_s(4));
fprintf('Kp=%.6f Ki=%.6f Kd=%.6f\n', Kp, Ki, Kd);
fprintf('q0=%.6f q1=%.6f q2=%.6f\n', q0_cd, q1_cd, q2_cd);

fprintf('\n== Discreto-discreto ==\n');
fprintf('P1=%.6f P2=%.6f alfa=%.6f\n', P1, P2, alfa);
fprintf('Pd(z) = [%s]\n', num2str(Pd_z, '%.6f '));
fprintf('q0=%.6f q1=%.6f q2=%.6f s0=%.6f\n', q0_dd, q1_dd, q2_dd, s0_dd);

Lc = feedback(Cs*Gs, 1);
L1 = feedback(Cz_cd*Gz, 1);
L2 = feedback(Cz_dd*Gz, 1);
fprintf('\nPolos LC discreto-discreto: %s\n', num2str(pole(L2).', '%.5f '));
fprintf('Polos LC continuo-discreto: %s\n', num2str(pole(L1).', '%.5f '));
for c = {{'PID continuo', Lc}, {'Continuo-discreto', L1}, {'Discreto-discreto', L2}}
    S = stepinfo(c{1}{2}, 'SettlingTimeThreshold', 0.02);
    fprintf('%-18s ts=%.3f s  Mp=%.2f %%  tr=%.3f s  ess=%.2e\n', c{1}{1}, S.SettlingTime, S.Overshoot, S.RiseTime, 1-dcgain(c{1}{2}));
end
