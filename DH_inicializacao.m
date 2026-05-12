% DH_inicializacao.m
% =============================================================
% Carrega trim/linearizacao pre-computados (Sato OU Ana) e
% configura o workspace para usar modelo_linear_DH_CL.slx e
% Comparation_L_and_NL_*.slx.
%
% Pre-requisito: rodar DH_build_model.m UMA VEZ. Isso gera:
%   linear/sato/MATRIZES_DH.m
%   linear/ana/MATRIZES_DH.m
% Recompile sempre que mudar coeficientes ou trim.
% =============================================================

clear; clc; bdclose all; close all;

%% =================== ESCOLHA DO MODELO ===================
coef_choice = 'sato';     % <-- 'sato' ou 'ana'

%% Paths
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'comum'));
addpath(fullfile(rootDir, 'linear'));            % utilitarios (lin_DH, modelo_linear_DH)
addpath(fullfile(rootDir, 'linear', coef_choice));  % MATRIZES_DH.m da variante escolhida
addpath(fullfile(rootDir, 'nao_linear'));

%% Carrega matrizes lineares
matrizes_file = fullfile(rootDir, 'linear', coef_choice, 'MATRIZES_DH.m');
if ~isfile(matrizes_file)
    error(['MATRIZES_DH.m nao encontrado em linear/%s/.\n' ...
           'Rode DH_build_model.m primeiro para gerar trim/linearizacao.'], coef_choice);
end
run(matrizes_file);
fprintf('Matrizes carregadas: variante=%s, Ve=%.1f m/s, he=%.0f m\n', ...
    coef_choice, Ve, he);

%% Sample times
SampleTime     = 0.01;
TimeSimulation = 4;
TimeCloseloop  = 60;

%% ===================== GANHOS DO PILOTO AUTOMATICO =====================
% Tunados via pidtune para o DH (cascata inner-first).

% Altitude Hold: h_ref -> theta_ref (BW=0.3 rad/s, mais lento que C_vel)
C_alt.Kp = 0.0220; C_alt.Ki = 0.0085; C_alt.Kd = 0.0;     C_alt.N = 20;

% Pitch Attitude: theta_ref -> delta_e (BW=2 rad/s)
C_theta.Kp = 0.2731; C_theta.Ki = 0.3821; C_theta.Kd = 0.0406; C_theta.N = 229;

% Velocity Hold: VT_ref -> delta_T (BW=1.0 rad/s, PIDF, PM=70, com pitch fechado)
C_vel.Kp = 0.177;    C_vel.Ki = 0.084;    C_vel.Kd = -0.002;  C_vel.N = 114.3;

% Roll (Bank Angle Hold): phi_ref -> delta_a (BW=2 rad/s, PM=75)
% Gains negativos pq B_lat(p,aileron)<0 (Cl_da<0 nas duas calibracoes)
C_phi.Kp = -0.8300; C_phi.Ki = -0.1712; C_phi.Kd = 0.3230; C_phi.N = 2;

% SAS dampers + heading
Kq        = 0;
Kp        = 0;
Kr        = 0;
K_heading = 0.1712;

% Trim absoluto dos atuadores (usado pelo modelo NL para somar PID + trim)
TrimInput = Ue(1:4);     % [throttle elevator aileron rudder]

%% ===================== REFERENCIAS =====================
% Default: refs no trim (sem step). Edite no workspace para excitar:
%   h_ref  = he + 50; VT_ref = Ve + 6; psi_ref_final = deg2rad(20);
h_ref  = he;
VT_ref = Ve;

psi_ref_init  = 0;
psi_ref_final = 0;
psi_ref_t     = 5;

%% ===================== Pronto =====================
fprintf('Workspace configurado (%s). Pode abrir/simular:\n', coef_choice);
fprintf('  - linear/modelo_linear_DH_CL.slx       (closed-loop linear)\n');
fprintf('  - nao_linear/modelo_NL_DH_CL.slx       (closed-loop nao-linear)\n');
fprintf('  - linear/Comparation_L_and_NL_Long.slx (validacao L vs NL)\n');
fprintf('  - linear/Comparation_L_and_NL_Lat.slx  (validacao L vs NL)\n');
