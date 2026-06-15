% missao_perfil.m
% Perfil de missao completo no modelo NL:
%   Aceleracao        t=5s        VT: 12.0 -> 15.2 m/s (degrau)
%   Correcao de proa  t=50/100/150s  psi: 0 -> +5 -> -5 -> 0 (doublet)
%   Mudanca de altitude t=80/160/240s  h: 600 -> 605 -> 595 -> 600 m (doublet)
raiz = fileparts(fileparts(mfilename('fullpath')));  % raiz do repo (pasta acima de manobras/)
addpath(raiz);                                        % garante DH_inicializacao e plot_NL_DH no path
run(fullfile(raiz,'DH_inicializacao.m'));

% --- Aceleracao: degrau de VT em t=5s ---
VT_ref        = Ve;            % base 12 m/s
VT_step_delta = 15.2 - Ve;     % +3.2 m/s
VT_step_t     = 5;

% --- Correcao de proa: doublet de psi 0->+5->-5->0 em 50/100/150 ---
psi_ref_init  = 0;
psi_ref_final = deg2rad(5);
psi_ref_t     = 50;            % +5 deg
psi_ref_t2    = 100;           % -10 deg (net -5)
psi_ref_t3    = 150;           % +5 deg (net 0)

% --- Mudanca de altitude: doublet de h 600->605->595->600 em 80/160/240 ---
h_ref        = he;            % base 600 m
h_step_init  = 0;
h_step_final = 5;             % +5 m
h_step_t     = 80;
h_step_t2    = 160;           % -10 (net -5)
h_step_t3    = 240;           % +5 (net 0)

att_alt = 0;                  % cascata de altitude ativa

% --- simulacao ate 280 s ---
out = sim('modelo_NL_DH_CL','StopTime','280');

% plota (Fig longitudinal + Fig latero-direcional)
salvar_png = true;
plot_NL_DH;
