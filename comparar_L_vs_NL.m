% comparar_L_vs_NL.m
% =============================================================
% Compara o controle PID em malha fechada aplicado a:
%   - planta NL  (sfunction_DH em modelo_NL_DH_CL.slx)
%   - planta LINEAR (sys_long, sys_lat carregadas pelo
%     DH_inicializacao via MATRIZES_DH.m)
%
% Sob a MESMA excitacao definida no workspace, com os MESMOS
% PIDs (C_alt, C_theta, C_vel, C_phi, K_heading).
%
% Gera PNGs em PID/Imagens Apresentacao/ com prefixo
%   compLN_<excitacao>_<sinal>.png
% sobrepondo: ref (preto tracejado), NL (laranja), linear (azul -.).
%
% Pre-requisito:
%   DH_inicializacao
%   (opcional) overrides:  h_ref = he + 50;  VT_ref = Ve + 6;
%                          psi_ref_final = deg2rad(20); psi_ref_t = 5;
%
% Uso tipico:
%   DH_inicializacao
%   h_ref = he + 50;
%   comparar_L_vs_NL
%
% Notas:
%   - Linear esta em deltas-do-trim; somamos o trim de volta usando
%     o estado inicial Y_nl(1,:) do NL.
%   - SAS dampers incluidos no CL linear: pitch damper
%     (elevator = PI - Kq*q) e yaw damper (rudder = -Kr*s/(s+1)*r).
%     Kp (roll damper) nao incluido — e' 0 no projeto atual.
%   - Saturadores nao existem na versao linear — divergencia em
%     manobras grandes e' por isso (e' a graca da comparacao).
%   - O bypass att_alt=1 (step direto em theta) NAO esta tratado;
%     o linear sempre assume cascata altitude->pitch.
% =============================================================

%% -------- 1. checagens --------
required = {'sys_long','sys_lat','C_alt','C_theta','C_vel','C_phi',...
            'K_heading','Ue','he','Ve','h_ref','VT_ref',...
            'psi_ref_init','psi_ref_final','psi_ref_t'};
for k = 1:numel(required)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', required{k}))
        error(['Variavel ''%s'' nao encontrada. ' ...
               'Rode DH_inicializacao antes.'], required{k});
    end
end

R2D = 180/pi;

%% -------- 2. pega NL (reusa "out" do workspace se ja existir) --------
if evalin('base','exist(''out'',''var'')') && ...
   isa(evalin('base','out'), 'Simulink.SimulationOutput')
    fprintf('[1/5] Usando "out" ja existente do workspace (skip sim).\n');
    out_nl = evalin('base','out');
else
    fprintf('[1/5] Rodando NL closed-loop...\n');
    out_nl = sim('modelo_NL_DH_CL');
end

t_nl        = out_nl.tout;
Y_nl        = out_nl.Y.signals.values;   % [VT alpha beta gamma p q r phi theta psi ax ay az xN h ...]
U_nl        = out_nl.U.signals.values;   % [throttle elev ail rud]
h_nl        = Y_nl(:,15);
theta_nl    = Y_nl(:,9)  * R2D;
VT_nl       = Y_nl(:,1);
psi_nl      = Y_nl(:,10) * R2D;
phi_nl      = Y_nl(:,8)  * R2D;
elev_nl     = U_nl(:,2)  * R2D;
ail_nl      = U_nl(:,3)  * R2D;
throttle_nl = U_nl(:,1);

% Trim absoluto inferido do estado inicial da NL (Y_nl em t=0):
VT_trim    = Y_nl(1,1);
phi_trim   = Y_nl(1,8);
theta_trim = Y_nl(1,9);
psi_trim   = Y_nl(1,10);
h_trim     = Y_nl(1,15);

%% -------- 3. monta CL linear longitudinal --------
fprintf('[2/5] Montando CL linear longitudinal...\n');

% sys_long ja vem com InputName={'throttle','elevator'} e
% OutputName={'u','alpha','q','theta','h'} do MATRIZES_DH.m.
% Nao re-nomeia, usa esses nomes pra ligar tudo.
P_long = sys_long;

% PIDs (filtro derivativo Tf = 1/N apenas se Kd != 0)
make_pid = @(C) makePID(C);
Calt   = make_pid(C_alt);    Calt.InputName   = 'e_h';     Calt.OutputName   = 'theta_ref_int';
Ctheta = make_pid(C_theta);  Ctheta.InputName = 'e_theta'; Ctheta.OutputName = 'u_theta';
Cvel   = make_pid(C_vel);    Cvel.InputName   = 'e_VT';    Cvel.OutputName   = 'throttle';

% Pitch damper: elevator = u_theta - Kq*q
Kq_blk = ss(Kq); Kq_blk.InputName = 'q'; Kq_blk.OutputName = 'kq_q';

% Somadores (sumblk)
Sum_h     = sumblk('e_h     = h_ref         - h');
Sum_theta = sumblk('e_theta = theta_ref_int - theta');
Sum_VT    = sumblk('e_VT    = VT_ref        - VT');   % fecha em VT (saida 6 de sys_long)
Sum_de    = sumblk('elevator = u_theta - kq_q');

CL_long = connect(P_long, Calt, Ctheta, Cvel, Kq_blk, Sum_h, Sum_theta, Sum_VT, Sum_de, ...
                  {'h_ref','VT_ref'}, ...
                  {'h','theta','VT','elevator','throttle'});

%% -------- 4. monta CL linear lateral --------
fprintf('[3/5] Montando CL linear lateral...\n');

% Com yaw damper: rudder = -Kr * s/(s+1) * r  (washout tau_w = 1).
% sys_lat ja vem com InputName={'aileron','rudder'} do MATRIZES_DH.m.
P_lat = sys_lat;

Cphi = make_pid(C_phi); Cphi.InputName = 'e_phi'; Cphi.OutputName = 'aileron';

% Yaw damper como bloco unico (ja com sinal e ganho)
Gw_blk = tf([1 0],[1 1]) * (-Kr);
Gw_blk.InputName = 'r'; Gw_blk.OutputName = 'rudder';

% Heading: bloco de ganho de verdade (sumblk NAO suporta ganho na formula)
Khead_blk = ss(K_heading);
Khead_blk.InputName = 'e_psi'; Khead_blk.OutputName = 'phi_ref';

Sum_psi = sumblk('e_psi   = psi_ref - psi');
Sum_phi = sumblk('e_phi   = phi_ref - phi');

CL_lat = connect(P_lat, Cphi, Gw_blk, Khead_blk, Sum_psi, Sum_phi, ...
                 {'psi_ref'}, ...
                 {'psi','phi','beta','p','r','aileron'});

%% -------- 5. simula linear --------
fprintf('[4/5] Simulando linear (lsim no time grid do NL)...\n');

% Excitacoes em DELTAS-do-trim:
h_ref_inc  = (h_ref  - he) * ones(size(t_nl));
VT_ref_inc = (VT_ref - Ve) * ones(size(t_nl));

psi_ref_inc = psi_ref_init * ones(size(t_nl));
psi_ref_inc(t_nl >= psi_ref_t) = psi_ref_final;

% Longitudinal
[y_long, ~] = lsim(CL_long, [h_ref_inc, VT_ref_inc], t_nl);
h_lin       = y_long(:,1) + h_trim;
theta_lin   = (y_long(:,2) + theta_trim) * R2D;
VT_lin      = y_long(:,3) + VT_trim;
de_lin      = (y_long(:,4) + Ue(2)) * R2D;
dT_lin      = y_long(:,5) + Ue(1);

% Lateral
[y_lat, ~]  = lsim(CL_lat, psi_ref_inc, t_nl);
psi_lin     = (y_lat(:,1) + psi_trim) * R2D;
phi_lin     = (y_lat(:,2) + phi_trim) * R2D;
da_lin      = (y_lat(:,6) + Ue(3)) * R2D;

%% -------- 6. plot + save --------
fprintf('[5/5] Salvando PNGs...\n');

% Forca todas as figuras subsequentes invisiveis (evita roubo de foco no macOS).
% Restauramos no fim do loop — sem onCleanup (que vaza entre scripts).
prev_vis = get(0, 'DefaultFigureVisible');
set(0, 'DefaultFigureVisible', 'off');

% Detecta excitacao pro prefixo (mesmo padrao do plot_PID)
ex_parts = {};
if abs(h_ref - he) > 1e-6
    ex_parts{end+1} = sprintf('h%+dm', round(h_ref - he));
end
if abs(VT_ref - Ve) > 1e-6
    ex_parts{end+1} = sprintf('VT%+dms', round(VT_ref - Ve));
end
if abs(psi_ref_final) > 1e-6
    ex_parts{end+1} = sprintf('psi%+ddeg', round(psi_ref_final * R2D));
end
if isempty(ex_parts)
    prefix_ex = 'trim';
elseif numel(ex_parts) == 1
    prefix_ex = ex_parts{1};
else
    prefix_ex = ['combinado_' strjoin(ex_parts, '_')];
end
prefix = ['compLN_' prefix_ex];

% Pasta
img_dir = fullfile(fileparts(mfilename('fullpath')), 'Imagens Apresentacao');
if ~exist(img_dir,'dir'); mkdir(img_dir); end

% Referencias constantes pra mostrar tracejadas
h_ref_const   = h_ref  * ones(size(t_nl));
VT_ref_const  = VT_ref * ones(size(t_nl));
psi_ref_const = psi_ref_inc * R2D;

% {nl, lin, ref, titulo, ylabel, filename}
comps = {
    h_nl,        h_lin,        h_ref_const,    'h',         'm',     'h';
    theta_nl,    theta_lin,    [],             '\theta',    'deg',   'theta';
    VT_nl,       VT_lin,       VT_ref_const,   'V_T',       'm/s',   'VT';
    elev_nl,     de_lin,       [],             'elev',      'deg',   'elev';
    throttle_nl, dT_lin,       [],             'throttle',  '-',     'throttle';
    psi_nl,      psi_lin,      psi_ref_const,  '\psi',      'deg',   'psi';
    phi_nl,      phi_lin,      [],             '\phi',      'deg',   'phi';
    ail_nl,      da_lin,       [],             'ail',       'deg',   'ail';
};

cNL  = [0.85 0.33 0.10];   % laranja (NL)
cLin = [0.20 0.45 0.75];   % azul (linear)
cRef = [0    0    0   ];   % preto (ref)
cGrid = [0.4 0.4 0.4];
LW   = 1.8;

for i = 1:size(comps,1)
    nl_  = comps{i,1};
    lin_ = comps{i,2};
    ref  = comps{i,3};
    ttl  = comps{i,4};
    ylbl = comps{i,5};
    fn   = comps{i,6};

    f = figure('Color','w','Position',[100 100 900 480],'Visible','off');
    try, f.Theme = 'light'; catch, end
    ax = axes(f); hold(ax,'on');
    set(ax, ...
        'Color','w','XColor','k','YColor','k', ...
        'GridColor',cGrid,'GridAlpha',0.3, ...
        'FontSize',12,'LineWidth',1.0,'Box','on');

    if ~isempty(ref) && any(~isnan(ref))
        plot(ax, t_nl, ref,  '--', 'Color', cRef, 'LineWidth', LW, 'DisplayName','ref');
    end
    plot(ax, t_nl, nl_,        'Color', cNL,  'LineWidth', LW, 'DisplayName','NL');
    plot(ax, t_nl, lin_, '-.', 'Color', cLin, 'LineWidth', LW, 'DisplayName','linear');

    grid(ax,'on');
    title(ax,  ttl, 'Interpreter','tex','FontSize',16, ...
          'Color','k','FontWeight','bold');
    ylabel(ax, ylbl,  'FontSize',13,'Color','k');
    xlabel(ax, 't [s]', 'FontSize',13,'Color','k');
    leg = legend(ax, 'Location','best','Box','off','FontSize',12);
    set(leg, 'TextColor','k');

    exportgraphics(f, fullfile(img_dir, [prefix '_' fn '.png']), ...
                   'BackgroundColor','white','Resolution',150);
    close(f);

    fprintf('     [%d/%d] %s.png\n', i, size(comps,1), fn);
end

set(0, 'DefaultFigureVisible', prev_vis);   % restaura visibilidade

fprintf('\nPNGs comparativos salvos em: %s\n', img_dir);
fprintf('  prefixo "%s"  (8 PNGs sobrepondo NL + linear)\n\n', prefix);

%% ---------- local function ----------
function C = makePID(s)
    if abs(s.Kd) < eps
        C = pid(s.Kp, s.Ki);                       % PI puro (sem filtro)
    else
        C = pid(s.Kp, s.Ki, s.Kd, 1/max(s.N,eps)); % PIDF com Tf=1/N
    end
end
