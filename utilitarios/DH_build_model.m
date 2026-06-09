% DH_build_model.m
% =============================================================
% Roda trim + linearizacao do Drone Hibrido (coeficientes da Ana)
% e grava as matrizes em linear/MATRIZES_DH.m.
%
% Recompile sempre que mudar:
%   - Coeficientes em utilitarios/coef_DH.m
%   - Condicoes de trim (Ve, he, gammae)
%   - Massa, S, b, c, inercia (em modelo_DH.m / obs_rigidbody_DH.m)
% =============================================================

clear; clc; bdclose all;

%% Paths
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(rootDir, 'utilitarios'));
addpath(fullfile(rootDir, 'linear', 'utilitarios'));
addpath(fullfile(rootDir, 'nao_linear'));

%% Condicoes de trim
Ve     = 12;          % m/s
he     = 600;         % m (MSL)
gammae = 0*pi/180;    % rad

surfaces = 24;
R        = 6371000;

fprintf('\nTrimando em Ve=%.1f m/s, he=%.0f m, gammae=%.1f deg ...\n', ...
    Ve, he, gammae*180/pi);
[Xe, Ue] = trimagem_DH(Ve, he, gammae);

fprintf('Linearizando ...\n');
[A, B, C, D] = lin_DH(Xe, Ue);

%% Build sys_DH, sys_long, sys_lat (decoupling)
plot_figure = 0;
abrir_modelo_simulink = 0;
modelo_linear_DH;

%% Trim em coordenadas longitudinais e laterais
% Xe_long = offsets das 6 saidas de sys_long: [u_e alpha_e q_e theta_e h_e VT_e]
Xe_long = [Ve*cos(Xe(8)-gammae), Xe(8)-gammae, 0, Xe(8), he, Ve];
Xe_lat  = [0 0 0 0 0];

%% Grava linear/MATRIZES_DH.m
out_dir = fullfile(rootDir, 'linear');
if ~isfolder(out_dir), mkdir(out_dir); end
out_file = fullfile(out_dir, 'MATRIZES_DH.m');

fid = fopen(out_file, 'w');
fprintf(fid, '%% MATRIZES_DH.m\n');
fprintf(fid, '%% =================================================================\n');
fprintf(fid, '%% Gerado automaticamente por DH_build_model.m em %s\n', datestr(now));
fprintf(fid, '%% NAO EDITAR a mao - re-rode DH_build_model.m para regenerar.\n');
fprintf(fid, '%% =================================================================\n\n');

fprintf(fid, '%%%% Condicoes de trim utilizadas\n');
fprintf(fid, 'Ve        = %g;\n', Ve);
fprintf(fid, 'he        = %g;\n', he);
fprintf(fid, 'gammae    = %g;\n', gammae);
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

fprintf(fid, '%%%% Modo longitudinal desacoplado: x=[u alpha q theta h], u=[throttle elevator], y=[u alpha q theta h VT]\n');
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
fprintf(fid, '    ''OutputName'',{''u'';''alpha'';''q'';''theta'';''h'';''VT''});\n\n');

fprintf(fid, 'sys_lat = ss(A_lat, B_lat, C_lat, D_lat, ...\n');
fprintf(fid, '    ''StateName'',{''beta'';''p'';''r'';''phi'';''psi''}, ...\n');
fprintf(fid, '    ''InputName'',{''aileron'';''rudder''}, ...\n');
fprintf(fid, '    ''OutputName'',{''beta'';''p'';''r'';''phi'';''psi''});\n');
fclose(fid);

fprintf('  alpha_e = %.4f rad (%.2f deg)\n', Xe(8)-gammae, (Xe(8)-gammae)*180/pi);
fprintf('  delta_t = %.4f (%.1f%%)\n',       Ue(1), Ue(1)*100);
fprintf('  delta_e = %.4f rad (%.3f deg)\n', Ue(2), Ue(2)*180/pi);
fprintf('  -> %s\n', out_file);

fprintf('\n=== OK. Proximo passo: rodar DH_inicializacao.m ===\n');
