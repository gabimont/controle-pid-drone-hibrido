% manobra_latero.m
% =============================================================
% MANOBRA LATERO-DIRECIONAL: degrau de heading de 15 deg em t = 5 s
% (exercita K_heading -> C_phi/Kp -> aileron, com o yaw damper
%  Kr + washout coordenando a curva no leme).
%
% Fluxo de uso:
%   1) DH_inicializacao
%   2) manobra_latero        <-- este script
%   3) out = sim('modelo_NL_DH_CL');   (ou Play no Simulink)
%   4) plot_NL_DH
%
% Pode rodar outra manobra_* em seguida sem re-inicializar:
% cada script zera as excitacoes da anterior.
% =============================================================

if ~exist('he','var') || ~exist('Xe','var')
    error('Workspace vazio. Rode DH_inicializacao antes da manobra.');
end

%% ---- zera excitacoes anteriores (volta refs ao trim) ----
h_ref  = he;   VT_ref = Ve;
psi_ref_init   = 0;  psi_ref_final   = 0;  psi_ref_t   = 5;
theta_step_init= 0;  theta_step_final= 0;  theta_step_t= 5;
phi_step_init  = 0;  phi_step_final  = 0;  phi_step_t  = 5;
h_step_init    = 0;  h_step_final    = 0;  h_step_t    = 20;
att_alt   = 0;
K_heading = Xe(1)/(9.80665*tau_psi);   % restaura (caso manobra anterior tenha zerado)

%% ---- excitacao desta manobra ----
psi_ref_final = deg2rad(15);     % curva de 15 deg de heading
psi_ref_t     = 5;               % em t = 5 s

% --- variacoes (descomente no lugar da excitacao acima) ---
% psi_ref_final = deg2rad(90); psi_ref_t = 5;        % curva de 90 deg
% K_heading = 0;                                     % PA de rolamento puro
% phi_step_final = deg2rad(10); phi_step_t = 5;      %   (a la Fig 4.13): phi_ref direto

fprintf('Manobra LATERO configurada: psi %+g deg em t = %g s.\n', ...
        rad2deg(psi_ref_final), psi_ref_t);
fprintf('Agora rode:  out = sim(''modelo_NL_DH_CL'');  e depois  plot_NL_DH\n');
