% manobra_long.m
% =============================================================
% MANOBRA LONGITUDINAL: degrau de altitude +10 m em t = 10 s
% (exercita C_alt -> clamp -> C_theta/Kq -> profundor, e C_vel
%  segurando VT com o throttle durante a subida).
%
% Fluxo de uso:
%   1) DH_inicializacao
%   2) manobra_long          <-- este script
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
phi_step_t2 = 1e9; phi_step_t3 = 1e9; psi_ref_t2 = 1e9; psi_ref_t3 = 1e9;
h_step_t2   = 1e9; h_step_t3   = 1e9; theta_step_t2 = 1e9; theta_step_t3 = 1e9;
att_alt   = 0;
K_heading = Xe(1)/(9.80665*tau_psi);   % restaura (caso manobra anterior tenha zerado)

%% ---- excitacao desta manobra ----
h_step_final = 10;       % +10 m de altitude
h_step_t     = 10;       % em t = 10 s

% --- variacoes (descomente no lugar da excitacao acima) ---
% h_step_final = 5;  h_step_t = 20;          % degrau a la Marcelo (Fig 4.7)
% h_step_final = 30; h_step_t = 5;           % subida grande (clamp satura ~10 s)
% VT_ref = Ve + 2;                           % degrau de velocidade +2 m/s em t=0
% att_alt = 1; theta_step_init = Xe(8);      % PA de arfagem puro (malha h aberta):
% theta_step_final = Xe(8) + deg2rad(5);     %   +5 deg de atitude em torno do trim
% theta_step_t = 5;

fprintf('Manobra LONGITUDINAL configurada: h %+g m em t = %g s.\n', ...
        h_step_final, h_step_t);
fprintf('Agora rode:  out = sim(''modelo_NL_DH_CL'');  e depois  plot_NL_DH\n');
