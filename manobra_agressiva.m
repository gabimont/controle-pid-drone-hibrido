% manobra_agressiva.m
% =============================================================
% MANOBRA AGRESSIVA COMBINADA: subida de +30 m e curva de 40 deg
% de heading SIMULTANEAS em t = 5 s.
%
% E' o caso mais exigente testado no retuning: o clamp de theta_ref
% satura em +10 deg por ~10 s (protecao de alpha), o throttle chega
% perto do maximo e os eixos longitudinal e latero acoplam (perda
% de sustentacao na curva enquanto sobe). Use para ver degradacao
% L vs NL e o anti-windup trabalhando.
%
% Fluxo de uso:
%   1) DH_inicializacao
%   2) manobra_agressiva     <-- este script
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
h_step_final  = 30;             % +30 m de altitude
h_step_t      = 5;              % em t = 5 s
psi_ref_final = deg2rad(40);    % e curva de 40 deg de heading
psi_ref_t     = 5;              % ao mesmo tempo

% --- variacoes (descomente no lugar da excitacao acima) ---
% h_step_final = 30; h_step_t = 5;                    % subida + curva de 90 deg
% psi_ref_final = deg2rad(90); psi_ref_t = 5;

fprintf('Manobra AGRESSIVA configurada: h %+g m e psi %+g deg em t = %g s.\n', ...
        h_step_final, rad2deg(psi_ref_final), psi_ref_t);
fprintf('Agora rode:  out = sim(''modelo_NL_DH_CL'');  e depois  plot_NL_DH\n');
