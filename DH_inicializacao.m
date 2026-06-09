% DH_inicializacao.m
% =============================================================
% Carrega trim/linearizacao pre-computados e configura o workspace
% para usar modelo_linear_DH_CL.slx, modelo_NL_DH_CL.slx e
% Comparation_L_and_NL_*.slx.
%
% Pre-requisito: rodar utilitarios/DH_build_model.m UMA VEZ.
% Isso gera linear/MATRIZES_DH.m. Recompile sempre que mudar
% coeficientes ou condicoes de trim.
% =============================================================

clear; clc; bdclose all; close all;

%% Paths
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'utilitarios'));
addpath(fullfile(rootDir, 'linear', 'utilitarios'));
addpath(fullfile(rootDir, 'linear'));
addpath(fullfile(rootDir, 'nao_linear'));

%% Carrega matrizes lineares
matrizes_file = fullfile(rootDir, 'linear', 'MATRIZES_DH.m');
if ~isfile(matrizes_file)
    error(['MATRIZES_DH.m nao encontrado em linear/.\n' ...
           'Rode utilitarios/DH_build_model.m primeiro para gerar trim/linearizacao.']);
end
run(matrizes_file);
fprintf('Matrizes carregadas: Ve=%.1f m/s, he=%.0f m\n', Ve, he);

%% Sample times
SampleTime     = 0.01;
TimeSimulation = 4;
TimeCloseloop  = 60;

%% ===================== GANHOS DO PILOTO AUTOMATICO =====================
% Retuning 2026-06-09 sobre o modelo linear (coef Ana, Ve=12, he=600),
% metodologia da dissertacao (cap. 3-4):
%   - SAS via LGR: Kq p/ zeta_SP=0.70 (estabiliza fugoide de quebra);
%     Kr p/ zeta_DR=0.707 sobre washout s/(s+1); rolamento ja e' Nivel 1
%     com folga -> Kp=0.
%   - Compensadores PI (Kd=0; amortecimento vem dos dampers explicitos),
%     pidtune com fechamento sequencial inner-first:
%       C_theta : theta/de   com Kq fechado          (wc=2.0,  PM=65)
%       C_vel   : VT/dT      com theta fechada       (wc=0.8,  zero em 0.3)
%       C_alt   : h/thetaref com theta E VT fechadas (wc=0.35, PM=55, GM=26dB)
%       C_phi   : phi/da     com yaw damper fechado  (wc=2.0,  PM=65)
%                 (ganhos negativos pq Cl_da<0)
%   - K_heading = u_e/(g*tau_psi), curva coordenada (Eq. 4.21 dissertacao).
% Malha fechada linear: long zeta_min=0.57; latero todos os polos no SPE
% (espiral estabilizada em -0.14).

% SAS dampers
Kq        = 0.0524;
Kp        = 0;
Kr        = 0.1481;

% Pitch Attitude: theta_ref -> delta_e
C_theta.Kp = 0.4007;  C_theta.Ki = 0.3918;  C_theta.Kd = 0;  C_theta.N = 100;

% Velocity Hold: VT_ref -> delta_T
C_vel.Kp = 0.2899;    C_vel.Ki = 0.0870;    C_vel.Kd = 0;    C_vel.N = 100;

% Altitude Hold: h_ref -> theta_ref
C_alt.Kp = 0.02875;   C_alt.Ki = 0.00259;   C_alt.Kd = 0;    C_alt.N = 100;

% Limite de saida do C_alt (delta theta_ref, rad) — protecao de alpha:
% com +10 deg o pico de alpha na captura fica ~16 deg (trim 14.4, sem
% modelo de estol nas plantas — limite e' imposto por projeto).
% Referenciado pelos blocos PID do C_alt nos modelos linear e NL.
theta_ref_clamp = [-0.1745  0.1745];   % [-10 +10] deg

% Roll (Bank Angle Hold): phi_ref -> delta_a
C_phi.Kp = -0.2831;   C_phi.Ki = -0.2716;   C_phi.Kd = 0;    C_phi.N = 100;

% Heading Select: phi_ref = K_heading*(psi_ref - psi)
tau_psi   = 6;                          % [s] cte de tempo desejada de psi
K_heading = Xe(1)/(9.80665*tau_psi);    % = 0.1975 (curva coordenada)

% Trim absoluto dos atuadores (usado pelo modelo NL para somar PID + trim)
TrimInput = Ue(1:4);     % [throttle elevator aileron rudder]

% Seletor da malha longitudinal (Switch no modelo_NL_DH_CL):
%   att_alt = 0 -> theta_ref vem da malha de altitude (cascata PID completa)  <-- DEFAULT
%   att_alt = 1 -> theta_ref vem do Step direto (5 deg em t=5s)
att_alt = 0;

%% ===================== REFERENCIAS =====================
% Default: refs no trim (sem step). Edite no workspace para excitar:
%   h_ref  = he + 50; VT_ref = Ve + 6; psi_ref_final = deg2rad(20);
h_ref  = he;
VT_ref = Ve;

% Step de heading (Step1 no modelo)
psi_ref_init  = 0;
psi_ref_final = 0;
psi_ref_t     = 5;

% Step direto de theta (so atua se att_alt = 1)
theta_step_init  = 0;
theta_step_final = 0;
theta_step_t     = 5;

% Step direto de phi_ref (somado ao caminho do heading no modelo NL;
% para PA de rolamento isolado a la Fig 4.13: K_heading=0 + step aqui)
phi_step_init  = 0;
phi_step_final = 0;
phi_step_t     = 5;

% Step somado ao h_ref (nos DOIS modelos) — permite degrau de altitude
% em t arbitrario, a la Fig 4.7 do Marcelo (degrau em t=20 s):
%   h_ref = he; h_step_final = 5; h_step_t = 20;
h_step_init  = 0;
h_step_final = 0;
h_step_t     = 20;

%% ===================== Pronto =====================
fprintf('Workspace configurado. Pode abrir/simular:\n');
fprintf('  - linear/utilitarios/modelo_linear_DH_CL.slx       (closed-loop linear)\n');
fprintf('  - nao_linear/modelo_NL_DH_CL.slx                   (closed-loop nao-linear)\n');
fprintf('  - linear/utilitarios/Comparation_L_and_NL_Long.slx (validacao L vs NL)\n');
fprintf('  - linear/utilitarios/Comparation_L_and_NL_Lat.slx  (validacao L vs NL)\n');
