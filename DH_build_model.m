% DH_build_model.m
% =============================================================
% Roda trim + linearizacao para AMBAS as calibracoes (Sato e Ana)
% e grava em
%   linear/sato/MATRIZES_DH.m
%   linear/ana/MATRIZES_DH.m
%
% A inicializacao (DH_inicializacao.m) escolhe qual carregar
% via flag coef_choice = 'sato' | 'ana'.
%
% Recompile sempre que mudar:
%   - Coeficientes em coef_DH.m
%   - Condicoes de trim (Ve, he, gammae)
%   - Massa, S, b, c, inercia (em modelo_DH.m / obs_rigidbody_DH.m)
% =============================================================

clear; clc; bdclose all;

%% Paths
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'comum'));
addpath(fullfile(rootDir, 'linear'));
addpath(fullfile(rootDir, 'nao_linear'));

%% Configuracao do trim (comum aos dois modelos aerodinamicos)
Ve     = 12;          % m/s
he     = 600;         % m (MSL)
gammae = 0*pi/180;    % rad

surfaces = 24;
R        = 6371000;

%% Build dos dois conjuntos
variantes = struct( ...
    'name',  {'sato',     'ana'    }, ...
    'sato',  {1,          0        }, ...
    'ana',   {0,          1        });

for k = 1:numel(variantes)
    name      = variantes(k).name;
    coef_Sato = variantes(k).sato;
    coef_Ana  = variantes(k).ana;

    fprintf('\n=================== Variante: %s ===================\n', upper(name));
    fprintf('Trimando em Ve=%.1f m/s, he=%.0f m, gammae=%.1f deg ...\n', ...
        Ve, he, gammae*180/pi);
    [Xe, Ue] = trimagem_DH(Ve, he, gammae, coef_Sato, coef_Ana);

    fprintf('Linearizando ...\n');
    [A, B, C, D] = lin_DH(Xe, Ue, coef_Sato, coef_Ana);

    %% Build sys_DH, sys_long, sys_lat (decoupling)
    plot_figure = 0;
    abrir_modelo_simulink = 0;
    modelo_linear_DH;

    %% Trim em coordenadas longitudinais e laterais
    Xe_long = [Ve*cos(Xe(8)-gammae), Xe(8)-gammae, 0, Xe(8), he];
    Xe_lat  = [0 0 0 0 0];

    %% Grava linear/<name>/MATRIZES_DH.m
    out_dir = fullfile(rootDir, 'linear', name);
    if ~isfolder(out_dir), mkdir(out_dir); end
    out_file = fullfile(out_dir, 'MATRIZES_DH.m');

    fid = fopen(out_file, 'w');
    fprintf(fid, '%% MATRIZES_DH.m  (variante: %s)\n', name);
    fprintf(fid, '%% =================================================================\n');
    fprintf(fid, '%% Gerado automaticamente por DH_build_model.m em %s\n', datestr(now));
    fprintf(fid, '%% NAO EDITAR a mao - re-rode DH_build_model.m para regenerar.\n');
    fprintf(fid, '%% =================================================================\n\n');

    fprintf(fid, '%%%% Condicoes de trim utilizadas\n');
    fprintf(fid, 'coef_choice = ''%s'';\n', name);
    fprintf(fid, 'Ve        = %g;\n', Ve);
    fprintf(fid, 'he        = %g;\n', he);
    fprintf(fid, 'gammae    = %g;\n', gammae);
    fprintf(fid, 'coef_Sato = %d;\n', coef_Sato);
    fprintf(fid, 'coef_Ana  = %d;\n', coef_Ana);
    fprintf(fid, 'surfaces  = %g;\n', surfaces);
    fprintf(fid, 'R         = %g;\n\n', R);

    fprintf(fid, '%%%% Equilibrio (resolvido por fsolve)\n');
    fprintf(fid, 'Xe = %s'';\n',      mat2str(Xe(:)', 12));
    fprintf(fid, 'Ue = %s'';\n\n',    mat2str(Ue(:)', 12));
    fprintf(fid, 'Xe_long = %s;\n',   mat2str(Xe_long, 12));
    fprintf(fid, 'Xe_lat  = %s;\n\n', mat2str(Xe_lat,  12));

    fprintf(fid, '%%%% Modelo linear completo (14 estados x 7 inputs x 18 outputs)\n');
    fprintf(fid, 'A = %s;\n\n', mat2str(A, 12));
    fprintf(fid, 'B = %s;\n\n', mat2str(B, 12));
    fprintf(fid, 'C = %s;\n\n', mat2str(C, 12));
    fprintf(fid, 'D = %s;\n\n', mat2str(D, 12));

    fprintf(fid, '%%%% Modo longitudinal desacoplado: [u alpha q theta h] x [throttle elevator]\n');
    fprintf(fid, 'A_long = %s;\n\n', mat2str(A_long, 12));
    fprintf(fid, 'B_long = %s;\n\n', mat2str(B_long, 12));
    fprintf(fid, 'C_long = %s;\n\n', mat2str(C_long, 12));
    fprintf(fid, 'D_long = %s;\n\n', mat2str(D_long, 12));

    fprintf(fid, '%%%% Modo latero-direcional: [beta p r phi psi] x [aileron rudder]\n');
    fprintf(fid, 'A_lat = %s;\n\n', mat2str(A_lat, 12));
    fprintf(fid, 'B_lat = %s;\n\n', mat2str(B_lat, 12));
    fprintf(fid, 'C_lat = %s;\n\n', mat2str(C_lat, 12));
    fprintf(fid, 'D_lat = %s;\n\n', mat2str(D_lat, 12));

    fprintf(fid, '%%%% Reconstroi state-space objects com nomes\n');
    fprintf(fid, 'sys_DH = ss(A, B, C, D, ...\n');
    fprintf(fid, '    ''StateName'', {''1 u'';''2 v'';''3 w'';''4 p'';''5 q'';''6 r'';''7 phi'';''8 theta'';''9 psi'';''10 xN'';''11 xE'';''12 xD'';''13 mi'';''14 lambda''}, ...\n');
    fprintf(fid, '    ''InputName'', {''1 throttle'';''2 elevator'';''3 aileron'';''4 rudder'';''5 wind_x'';''6 wind_y'';''7 wind_z''}, ...\n');
    fprintf(fid, '    ''OutputName'',{''1 VT'';''2 alpha'';''3 beta'';''4 gamma'';''5 p'';''6 q'';''7 r'';''8 phi'';''9 theta'';''10 psi'';''11 ax'';''12 ay'';''13 az'';''14 xN'';''15 h'';''16 mi'';''17 lambda'';''18 qdot''});\n\n');

    fprintf(fid, 'sys_long = ss(A_long, B_long, C_long, D_long, ...\n');
    fprintf(fid, '    ''StateName'',{''u'';''alpha'';''q'';''theta'';''h''}, ...\n');
    fprintf(fid, '    ''InputName'',{''throttle'';''elevator''}, ...\n');
    fprintf(fid, '    ''OutputName'',{''u'';''alpha'';''q'';''theta'';''h''});\n\n');

    fprintf(fid, 'sys_lat = ss(A_lat, B_lat, C_lat, D_lat, ...\n');
    fprintf(fid, '    ''StateName'',{''beta'';''p'';''r'';''phi'';''psi''}, ...\n');
    fprintf(fid, '    ''InputName'',{''aileron'';''rudder''}, ...\n');
    fprintf(fid, '    ''OutputName'',{''beta'';''p'';''r'';''phi'';''psi''});\n');
    fclose(fid);

    fprintf('  alpha_e = %.4f rad (%.2f deg)\n', Xe(8)-gammae, (Xe(8)-gammae)*180/pi);
    fprintf('  delta_t = %.4f (%.1f%%)\n',       Ue(1), Ue(1)*100);
    fprintf('  delta_e = %.4f rad (%.3f deg)\n', Ue(2), Ue(2)*180/pi);
    fprintf('  -> %s\n', out_file);
end

%% Limpa MATRIZES_DH.m antigo (raiz de linear/) se existir
old_root = fullfile(rootDir, 'linear', 'MATRIZES_DH.m');
if isfile(old_root), delete(old_root); end

fprintf('\n=== OK. Variantes geradas. Proximo passo: DH_inicializacao.m ===\n');
fprintf('    Em DH_inicializacao.m, ajuste coef_choice = ''sato'' ou ''ana''.\n');
