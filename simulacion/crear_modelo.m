% Construye lab5_control.slx: PID continuo de referencia + lazos continuo-discreto
% y discreto-discreto con la ecuacion en diferencias implementada en el ESP32.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, 'parametros.m'));
mdl = 'lab5_control';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if exist(f, 'file'), delete(f); end
new_system(mdl);

initCmd = 'run(fullfile(fileparts(get_param(bdroot,''FileName'')),''parametros.m''));';
set_param(mdl, 'PreLoadFcn', initCmd, 'InitFcn', initCmd, ...
    'Solver', 'ode4', 'FixedStep', '1e-4', 'StopTime', 'T_FIN', ...
    'ReturnWorkspaceOutputs', 'on');

L = @(sys, a, b) add_line(sys, a, b, 'autorouting', 'smart');

%% Referencia: dos escalones (+R1 y luego R2)
add_block('simulink/Sources/Step', [mdl '/Escalon 1'], 'Time', 'T_R1', 'Before', '0', 'After', 'R1', ...
    'Position', [30 60 60 90]);
add_block('simulink/Sources/Step', [mdl '/Escalon 2'], 'Time', 'T_R2', 'Before', '0', 'After', 'R2-R1', ...
    'Position', [30 130 60 160]);
add_block('simulink/Math Operations/Sum', [mdl '/r'], 'Inputs', '++', 'Position', [100 95 120 125]);
L(mdl, 'Escalon 1/1', 'r/1'); L(mdl, 'Escalon 2/1', 'r/2');

%% Lazos
lazos = {
    'PID continuo',       'c',  [];
    'Continuo-discreto',  'cd', '_cd';
    'Discreto-discreto',  'dd', '_dd'};
for i = 1:size(lazos, 1)
    nm = lazos{i,1}; tag = lazos{i,2}; suf = lazos{i,3};
    sub = [mdl '/' nm];
    y0 = 40 + (i-1)*170;
    add_block('simulink/Ports & Subsystems/Subsystem', sub, 'Position', [220 y0 340 y0+90]);
    delete_line(sub, 'In1/1', 'Out1/1');
    delete_block([sub '/In1']); delete_block([sub '/Out1']);
    if isempty(suf), lazo_continuo(sub); else, lazo_discreto(sub, suf); end
    lh = L(mdl, 'r/1', [nm '/1']);
    if i == 1, set_param(lh, 'Name', 'r'); end

    % ISE = integral de e^2
    add_block('simulink/Math Operations/Math Function', [mdl '/e2_' tag], 'Operator', 'square', ...
        'Position', [400 y0+70 430 y0+95]);
    add_block('simulink/Continuous/Integrator', [mdl '/ISE_' tag], 'Position', [460 y0+70 490 y0+95]);
    add_block('simulink/Sinks/Display', [mdl '/Display ISE ' tag], 'Position', [520 y0+70 610 y0+95]);
    L(mdl, [nm '/3'], ['e2_' tag '/1']); L(mdl, ['e2_' tag '/1'], ['ISE_' tag '/1']);
    L(mdl, ['ISE_' tag '/1'], ['Display ISE ' tag '/1']);

    % A workspace
    vars = {['y_' tag], ['u_' tag], ['e_' tag], ['ise_' tag]};
    src  = {[nm '/1'], [nm '/2'], [nm '/3'], ['ISE_' tag '/1']};
    for k = 1:4
        add_block('simulink/Sinks/To Workspace', [mdl '/' vars{k}], 'VariableName', vars{k}, ...
            'SaveFormat', 'Timeseries', 'SampleTime', '1e-3', ...
            'Position', [660 y0-20+k*25 740 y0-5+k*25]);
        L(mdl, src{k}, [vars{k} '/1']);
    end
end
add_block('simulink/Sinks/To Workspace', [mdl '/r_ws'], 'VariableName', 'r', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '1e-3', 'Position', [140 20 200 40]);
L(mdl, 'r/1', 'r_ws/1');

%% Scopes
scopes = {'Posicion', 1, {'y_c','y_cd','y_dd'}; 'Control', 2, {'u_c','u_cd','u_dd'}; 'Error', 3, {'e_c','e_cd','e_dd'}};
for s = 1:3
    nm = ['Scope ' scopes{s,1}];
    n = 3 + (s == 1);
    add_block('simulink/Sinks/Scope', [mdl '/' nm], 'NumInputPorts', num2str(n), ...
        'Position', [860 60+(s-1)*170 900 60+(s-1)*170+20*n]);
    p = 1;
    if s == 1, h = L(mdl, 'r/1', [nm '/1']); p = 2; end %#ok<NASGU>
    for i = 1:3
        h = L(mdl, [lazos{i,1} '/' num2str(scopes{s,2})], [nm '/' num2str(p)]);
        set_param(h, 'Name', scopes{s,3}{i});
        p = p + 1;
    end
end

save_system(mdl, f);
fprintf('Modelo guardado: %s\n', f);

%% ----------------------------------------------------------------------
function lazo_continuo(sub)
    % PID continuo ideal (diseno de referencia): C(s) = (Kd s^2 + Kp s + Ki)/s con filtro 1/(s/N+1)
    L = @(a, b) add_line(sub, a, b, 'autorouting', 'smart');
    add_block('simulink/Sources/In1',  [sub '/r'], 'Position', [20 100 50 114]);
    add_block('simulink/Math Operations/Sum', [sub '/Sum'], 'Inputs', '+-', 'Position', [90 97 110 117]);
    add_block('simulink/Continuous/Transfer Fcn', [sub '/C(s)'], ...
        'Numerator', '[Kd Kp Ki]', 'Denominator', '[1/1000 1 0]', 'Position', [150 85 260 130]);
    add_block('simulink/Continuous/Transfer Fcn', [sub '/G(s)'], ...
        'Numerator', 'b0', 'Denominator', '[1 a1 a0]', 'Position', [320 85 420 130]);
    add_block('simulink/Sinks/Out1', [sub '/y'], 'Position', [500 100 530 114]);
    add_block('simulink/Sinks/Out1', [sub '/u'], 'Position', [500 160 530 174]);
    add_block('simulink/Sinks/Out1', [sub '/e'], 'Position', [500 220 530 234]);
    L('r/1', 'Sum/1'); L('Sum/1', 'C(s)/1'); L('C(s)/1', 'G(s)/1'); L('G(s)/1', 'y/1');
    L('G(s)/1', 'Sum/2'); L('C(s)/1', 'u/1'); L('Sum/1', 'e/1');
end

function lazo_discreto(sub, suf)
    % u(k) = sat[ (1-s0) u(k-1) + s0 u(k-2) + q0 e(k) + q1 e(k-1) + q2 e(k-2) ]
    % Los retardos guardan u sin saturar: el controlador es lineal y solo se satura la salida.
    L = @(a, b) add_line(sub, a, b, 'autorouting', 'smart');
    add_block('simulink/Sources/In1', [sub '/r'], 'Position', [20 60 50 74]);
    add_block('simulink/Discrete/Zero-Order Hold', [sub '/Muestreo r'], 'SampleTime', 'T', 'Position', [80 52 115 82]);
    add_block('simulink/Math Operations/Sum', [sub '/Sum e'], 'Inputs', '+-', 'Position', [150 57 170 77]);
    % ramas del error
    add_block('simulink/Math Operations/Gain', [sub '/q0'], 'Gain', ['q0' suf], 'Position', [260 50 310 80]);
    add_block('simulink/Discrete/Unit Delay', [sub '/e(k-1)'], 'SampleTime', 'T', 'Position', [200 110 235 140]);
    add_block('simulink/Math Operations/Gain', [sub '/q1'], 'Gain', ['q1' suf], 'Position', [260 110 310 140]);
    add_block('simulink/Discrete/Unit Delay', [sub '/e(k-2)'], 'SampleTime', 'T', 'Position', [200 170 235 200]);
    add_block('simulink/Math Operations/Gain', [sub '/q2'], 'Gain', ['q2' suf], 'Position', [260 170 310 200]);
    % ramas del control
    add_block('simulink/Discrete/Unit Delay', [sub '/u(k-1)'], 'SampleTime', 'T', 'Position', [520 240 555 270], ...
        'Orientation', 'left');
    add_block('simulink/Math Operations/Gain', [sub '/1-s0'], 'Gain', ['1-s0' suf], 'Position', [260 240 310 270], ...
        'Orientation', 'left');
    add_block('simulink/Discrete/Unit Delay', [sub '/u(k-2)'], 'SampleTime', 'T', 'Position', [520 300 555 330], ...
        'Orientation', 'left');
    add_block('simulink/Math Operations/Gain', [sub '/s0'], 'Gain', ['s0' suf], 'Position', [260 300 310 330], ...
        'Orientation', 'left');
    add_block('simulink/Math Operations/Sum', [sub '/Sum u'], 'Inputs', '+++++', 'IconShape', 'rectangular', ...
        'Position', [370 40 395 210]);
    add_block('simulink/Discontinuities/Saturation', [sub '/Saturacion'], 'UpperLimit', 'U_MAX', 'LowerLimit', 'U_MIN', ...
        'Position', [430 110 470 140]);
    % planta y sensor
    add_block('simulink/Continuous/Transfer Fcn', [sub '/G(s)'], 'Numerator', 'b0', 'Denominator', '[1 a1 a0]', ...
        'Position', [620 105 720 145]);
    add_block('simulink/Discrete/Zero-Order Hold', [sub '/Muestreo y'], 'SampleTime', 'T', ...
        'Position', [760 175 795 205], 'Orientation', 'left');
    add_block('simulink/Discontinuities/Quantizer', [sub '/Encoder'], 'QuantizationInterval', 'Q_ENC', ...
        'Position', [680 175 715 205], 'Orientation', 'left');
    add_block('simulink/Sinks/Out1', [sub '/y'], 'Position', [820 118 850 132]);
    add_block('simulink/Sinks/Out1', [sub '/u'], 'Position', [820 30 850 44]);
    add_block('simulink/Sinks/Out1', [sub '/e'], 'Position', [820 0 850 14]);

    L('r/1', 'Muestreo r/1'); L('Muestreo r/1', 'Sum e/1');
    L('Sum e/1', 'q0/1'); L('Sum e/1', 'e(k-1)/1'); L('e(k-1)/1', 'q1/1');
    L('e(k-1)/1', 'e(k-2)/1'); L('e(k-2)/1', 'q2/1');
    L('q0/1', 'Sum u/1'); L('q1/1', 'Sum u/2'); L('q2/1', 'Sum u/3');
    L('1-s0/1', 'Sum u/4'); L('s0/1', 'Sum u/5');
    L('Sum u/1', 'Saturacion/1'); L('Saturacion/1', 'G(s)/1');
    L('Sum u/1', 'u(k-1)/1'); L('u(k-1)/1', '1-s0/1');
    L('u(k-1)/1', 'u(k-2)/1'); L('u(k-2)/1', 's0/1');
    L('G(s)/1', 'y/1'); L('G(s)/1', 'Muestreo y/1'); L('Muestreo y/1', 'Encoder/1');
    L('Encoder/1', 'Sum e/2'); L('Saturacion/1', 'u/1'); L('Sum e/1', 'e/1');
end
