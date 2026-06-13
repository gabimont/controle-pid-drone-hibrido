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
% (Retune de ganho wc=0.18 REVERTIDO em 2026-06-13: o "teleporte" do profundor/
%  manete sera removido por SETPOINT WEIGHTING b=0 nos blocos PID, que tira o
%  chute proporcional sem abrir mao da banda nem das margens.)
C_alt.Kp = 0.02875;   C_alt.Ki = 0.00259;   C_alt.Kd = 0;    C_alt.N = 100;

% Limite de saida do C_alt (delta theta_ref, rad) — protecao de alpha:
% com +10 deg o pico de alpha na captura fica ~16 deg (trim 14.4, sem
% modelo de estol nas plantas — limite e' imposto por projeto).
% Referenciado pelos blocos PID do C_alt nos modelos linear e NL.
theta_ref_clamp = [-0.1745  0.1745];   % [-10 +10] deg

%% ===================== MODELO DE ATUADOR (SERVO) =====================
% Ate aqui o atuador era IDEAL: o comando do controle virava deflexao de
% superficie instantaneamente. Sob degrau de referencia isso gera um
% salto impulsivo no profundor (~400 deg/s em 1 passo) — inviavel num
% servo real e no hardware-in-the-loop. O servo real tem TAXA maxima de
% deflexao (slew) e BANDA finita. Modelamos cada superficie como
%   Rate Limiter (slew)  ->  lag de 1a ordem 1/(act.tau*s + 1)
% a jusante da saturacao de posicao (+-25 deg, ja existente nos blocos
% Sat_*). Assim o degrau produz uma rampa rapida porem REALIZAVEL, e a
% malha fechada e' validada COM o atraso de fase do servo.
%
% PENDENTE: trocar pelos numeros do servo real do HIL. Valores tipicos de
% servo de aeromodelo / mini-UAV (analogico/digital sob carga):
act.rate = deg2rad(150);   % taxa maxima de deflexao [rad/s]  (~150 deg/s)
act.bw   = 20;             % banda do servo [rad/s] (~3.2 Hz)
act.tau  = 1/act.bw;       % constante de tempo do lag [s] (=0.05 s)
% Saturacao de posicao (+-25 deg) ja esta nos blocos Sat_* dos modelos.

% --- Motor / helice (mesma ideia, no canal de throttle) ---
% O throttle tambem era ideal: saltava instantaneamente (ex.: 0.28->0.79 em
% 1 passo na aceleracao). Motor+helice tem inercia (spool-up) e taxa maxima.
% Modelado como rate limiter + lag 1/(eng.tau*s + 1) a jusante do Sat_Throttle.
% PROVISORIO: ajustar com a planta propulsiva real do HIL.
eng.rate = 1.0;            % taxa maxima do throttle [1/s] (curso 0->1 em ~1 s)
eng.tau  = 0.30;           % constante de tempo de spool [s] (banda ~3.3 rad/s)
% Saturacao de posicao do throttle ([0,1]) ja esta no bloco Sat_Throttle.

% Roll (Bank Angle Hold): phi_ref -> delta_a
C_phi.Kp = -0.2831;   C_phi.Ki = -0.2716;   C_phi.Kd = 0;    C_phi.N = 100;

% Pre-filtro de comando (Opcao B, 2026-06-13): suaviza os degraus de referencia
% com 1/(tau_ref*s+1) ANTES das malhas, removendo o "teleporte" dos atuadores
% sem mexer nos controladores nem no clamp. Aplicado ao DELTA de cada referencia
% (doublets de h e psi; degrau de VT em torno de Ve) -> sem transiente de partida.
% Referenciado pelos blocos Transfer Fcn de pre-filtro no modelo NL.
tau_ref = 3;          % [s] constante de tempo do pre-filtro de referencia

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

% Degrau de velocidade (bloco Step_VT_ref no controle): VT_ref -> VT_ref+VT_step_delta
% em t = VT_step_t. Default inerte (delta=0). Ex.: VT_step_delta = 3.2; VT_step_t = 5;
VT_step_delta = 0;
VT_step_t     = 5;

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

% DOUBLETS (modelo NL): cada canal de referencia tem 2 steps extras
% (Step_*2/Step_*3 + Add_dbl_*) que transformam o degrau em doublet
% identico ao harness do Mirko (LQRY):  0 -> +A em t1 -> -A em t2 -> 0 em t3.
% Amplitude reusa a do degrau principal. Default t2=t3=1e9 = INERTE.
phi_step_t2   = 1e9;  phi_step_t3   = 1e9;
psi_ref_t2    = 1e9;  psi_ref_t3    = 1e9;
h_step_t2     = 1e9;  h_step_t3     = 1e9;
theta_step_t2 = 1e9;  theta_step_t3 = 1e9;

%% ===================== Pronto =====================
fprintf('Workspace configurado. Pode abrir/simular:\n');
fprintf('  - linear/utilitarios/modelo_linear_DH_CL.slx       (closed-loop linear)\n');
fprintf('  - nao_linear/modelo_NL_DH_CL.slx                   (closed-loop nao-linear)\n');
fprintf('  - linear/utilitarios/Comparation_L_and_NL_Long.slx (validacao L vs NL)\n');
fprintf('  - linear/utilitarios/Comparation_L_and_NL_Lat.slx  (validacao L vs NL)\n');
