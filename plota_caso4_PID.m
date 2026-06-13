%% plota_caso4_PID.m
% Reproduz o CASO 4 do harness LQRY (VelHold + PsiHold + AltitudeHold)
% aplicando as MESMAS referencias no modelo PID (modelo_NL_DH_CL), e plota
% no mesmo layout 6x2 e MESMAS ESCALAS do plota_caso4.m do LQRY.
%
% Referencias do Caso 4 (extraidas de Plot_caso4.mat do LQRY):
%   VT : degrau 12 -> 15.2 m/s em t=5 s
%   psi: doublet 0 -> +5 -> -5 -> 0 deg em t=50/100/150 s
%   h  : doublet 600 -> 605 -> 595 -> 600 m em t=80/160/240 s
%   StopTime = 300 s
% =============================================================
run(fullfile(fileparts(mfilename('fullpath')),'DH_inicializacao.m'));
R2D = 180/pi;

%% --- Referencias do Caso 4 ---
VT_ref = Ve; VT_step_delta = 15.2 - Ve; VT_step_t = 5;          % VelHold 15.2
psi_ref_init=0; psi_ref_final=deg2rad(5); psi_ref_t=50; psi_ref_t2=100; psi_ref_t3=150;
h_ref = he; h_step_init=0; h_step_final=5; h_step_t=80; h_step_t2=160; h_step_t3=240;
phi_step_final=0; K_heading = Xe(1)/(9.80665*tau_psi); att_alt=0;

out = sim('modelo_NL_DH_CL','StopTime','300');

t  = out.tout;
Y  = out.Y.signals.values;     % [VT a b gam p q r phi th psi ax ay az xN h]
U  = out.U.signals.values;     % [thr elev ail rud]
stp = @(t,v0,v1,ts) v0 + (v1-v0).*(t>=ts);

% referencias reconstruidas (mesmo agendamento do modelo)
VT_r  = stp(t,Ve,15.2,5);
psi_r = ( deg2rad(5)*(t>=50) - 2*deg2rad(5)*(t>=100) + deg2rad(5)*(t>=150) )*R2D;
h_r   = he + 5*(t>=80) - 10*(t>=160) + 5*(t>=240);

%% --- Figura 6x2 (mesmo estilo/escala do LQRY) ---
titulo = 'Caso 4 (PID): Velocidade Hold com \psi Hold e Altitude Hold';
fig = figure('Name',titulo,'Color','w');
set(fig,'Units','normalized','Position',[0.03 0.05 0.94 0.88]);
try, theme(fig,'light'); catch, end
tl = tiledlayout(fig,6,2,'TileSpacing','compact','Padding','compact');
title(tl,titulo,'FontWeight','bold');

% ----- coluna esquerda (longitudinal) -----
painel(nexttile(tl,1),  t, Y(:,9)*R2D,  [],   '\theta','deg',   [0 300],[-10 40], false);
painel(nexttile(tl,3),  t, Y(:,6)*R2D,  [],   'q','deg/s',      [0 300],[-30 30], false);
painel(nexttile(tl,5),  t, U(:,2)*R2D,  [],   'Elevador','deg', [0 300],[-10 25], false);
painel(nexttile(tl,7),  t, Y(:,1),      VT_r, 'Velocidade','m/s',[0 300],[8 22],  false, 'Velocidade','Ref. velocidade');
painel(nexttile(tl,9),  t, U(:,1),      [],   'Throttle','u_T [-]',[0 300],[-0.2 1.2], false);
painel(nexttile(tl,11), t, Y(:,15),     h_r,  'Altitude','m',   [0 300],[580 620], true, 'Altitude','Ref. altitude');

% ----- coluna direita (latero-direcional) -----
painel(nexttile(tl,2),  t, Y(:,8)*R2D,  [],    '\phi','deg',    [0 300],[-30 30], false);
painel(nexttile(tl,4),  t, Y(:,5)*R2D,  [],    'p','deg/s',     [0 300],[-3 3],   false);
painel(nexttile(tl,6),  t, U(:,3)*R2D,  [],    'Aileron','deg', [0 300],[-1 1],   false);
painel(nexttile(tl,8),  t, Y(:,10)*R2D, psi_r, '\psi','deg',    [0 300],[-10 10], false, '\psi','Ref. \psi');
painel(nexttile(tl,10), t, Y(:,7)*R2D,  [],    'r','deg/s',     [0 300],[-5 5],   false);
painel(nexttile(tl,12), t, U(:,4)*R2D,  [],    'Leme','deg',    [0 300],[-5 5],   true);

out_png = fullfile(fileparts(mfilename('fullpath')),'Imagens','Caso4_12painel_PID.png');
exportgraphics(fig,out_png,'Resolution',150,'BackgroundColor','white');
fprintf('Figura salva em: %s\n', out_png);

%% ---------- funcao local de painel (igual ao LQRY) ----------
function painel(ax, t, y, yref, titulo, ylab, xl, yl, mostrar_xlabel, nome_y, nome_ref)
    plot(ax, t, y, 'LineWidth', 1.5); hold(ax,'on');
    if ~isempty(yref), plot(ax, t, yref, '--', 'LineWidth', 1.5); end
    hold(ax,'off'); grid(ax,'on'); box(ax,'on');
    xlim(ax, xl); ylim(ax, yl);
    title(ax, titulo); ylabel(ax, ylab);
    if mostrar_xlabel, xlabel(ax, 'Tempo [s]'); end
    if nargin >= 11 && ~isempty(yref)
        lgd = legend(ax, nome_y, nome_ref, 'Location', 'best');
    elseif nargin >= 10
        lgd = legend(ax, nome_y, 'Location', 'best');
    else
        lgd = legend(ax, titulo, 'Location', 'best');
    end
    lgd.FontSize = 8; ax.FontSize = 9;
end
