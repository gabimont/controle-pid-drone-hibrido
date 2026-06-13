%% valida_prefilter_missao.m
%  Valida o PRE-FILTRO de referencia (Opcao B) na missao NL (Caso 4):
%  backup (SEM pre-filtro) x atual (COM pre-filtro 1/(tau_ref*s+1)).
%  Confirma: teleportes removidos, sem transiente de partida, rastreamento mantido.

clear; clc; bdclose all; close all;

p = strsplit(path, pathsep);
alvo = p(contains(p, ['novo trabalho' filesep 'PID']));
if ~isempty(alvo), rmpath(alvo{:}); end
run('/Users/kauemartinsdesouza/NOVODH/PID/DH_inicializacao.m');   % ganhos originais + tau_ref=3
R2D = 180/pi;

% refs da missao (Caso 4)
VT_ref=Ve; VT_step_delta=15.2-Ve; VT_step_t=5;
psi_ref_init=0; psi_ref_final=deg2rad(5); psi_ref_t=50; psi_ref_t2=100; psi_ref_t3=150;
h_ref=he; h_step_init=0; h_step_final=5; h_step_t=80; h_step_t2=160; h_step_t3=240;
phi_step_final=0; att_alt=0;

ext = @(o) deal(o.tout, o.U.signals.values, o.Y.signals.values);

% BASELINE (modelo de backup, sem pre-filtro)
oB = sim('modelo_NL_DH_CL_bk_pre2dof_20260613_0307','StopTime','300');
[tB,UB,YB] = ext(oB);
% PRE-FILTRADO (modelo atual)
oR = sim('modelo_NL_DH_CL','StopTime','300');
[tR,UR,YR] = ext(oR);

trim=[Ue(2)*R2D 0 0 Ue(1)];  ev=[5 50 80 100 150 160 240];
rate=@(t,u) max(abs(diff(u)./diff(t)));
fprintf('\n=== Validacao do pre-filtro (missao Caso 4) ===\n');
fprintf('Arranque (t=0): h=%.1f/%.1f  VT=%.2f/%.2f  theta=%.2f/%.2f  (baseline/prefiltro)\n', ...
    YB(1,15),YR(1,15), YB(1,1),YR(1,1), YB(1,9)*R2D,YR(1,9)*R2D);
fprintf('%-10s | baseline pico/taxa | prefiltro pico/taxa\n','superf');
fprintf('Profundor  | %6.2f / %6.1f    | %6.2f / %6.1f  (deg, deg/s)\n', max(abs(UB(:,2)*R2D-trim(1))),rate(tB,UB(:,2)*R2D), max(abs(UR(:,2)*R2D-trim(1))),rate(tR,UR(:,2)*R2D));
fprintf('Aileron    | %6.3f / %6.1f    | %6.3f / %6.1f  (deg, deg/s)\n', max(abs(UB(:,3)*R2D)),rate(tB,UB(:,3)*R2D), max(abs(UR(:,3)*R2D)),rate(tR,UR(:,3)*R2D));
fprintf('Manete     | curso[%.2f %.2f] taxa %.3f | curso[%.2f %.2f] taxa %.3f\n', min(UB(:,1)),max(UB(:,1)),rate(tB,UB(:,1)), min(UR(:,1)),max(UR(:,1)),rate(tR,UR(:,1)));

%% ===================== Figura 2x3 =====================
cB=[0.55 0.55 0.55]; cR=[0 0.447 0.741]; cD=[0.5 0.5 0.5];
fig=figure('Color','w','Position',[30 30 1280 760]); try, theme(fig,'light'); catch, end
TL=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
title(TL,'Validacao do pre-filtro (\tau=3 s) — Missao Caso 4: baseline \times pre-filtrado','FontWeight','bold');

% linha 1: superficies
S = {UB(:,2)*R2D, UR(:,2)*R2D, trim(1), 'Profundor  \delta_e','graus'; ...
     UB(:,3)*R2D, UR(:,3)*R2D, 0,       'Aileron  \delta_a','graus'; ...
     UB(:,1),     UR(:,1),     trim(4), 'Manete  \delta_T','[-]'};
for i=1:3
    ax=nexttile(TL); hold(ax,'on');
    for k=1:numel(ev), xline(ax,ev(k),'-','Color',[0.92 0.92 0.92]); end
    yline(ax,S{i,3},':','Color',cD,'LineWidth',1.0);
    hB=plot(ax,tB,S{i,1},'-','Color',cB,'LineWidth',1.4);
    hR=plot(ax,tR,S{i,2},'-','Color',cR,'LineWidth',1.7);
    grid(ax,'on'); box(ax,'on'); xlim(ax,[0 300]); title(ax,S{i,4}); ylabel(ax,S{i,5}); xlabel(ax,'Tempo [s]'); ax.FontSize=10;
    if i==1, legend(ax,[hB hR],{'sem pre-filtro','com pre-filtro'},'Location','best','FontSize',9); end
end
% linha 2: rastreamento
refH = he + 5*(tR>=80) - 10*(tR>=160) + 5*(tR>=240);
refV = 12 + (15.2-12)*(tR>=5);
refP = 5*(tR>=50) - 10*(tR>=100) + 5*(tR>=150);
T = {YB(:,15), YR(:,15), refH, 'Altitude  h','m'; ...
     YB(:,1),  YR(:,1),  refV, 'Velocidade  V_T','m/s'; ...
     YB(:,10)*R2D, YR(:,10)*R2D, refP, 'Proa  \psi','deg'};
for i=1:3
    ax=nexttile(TL); hold(ax,'on');
    plot(ax,tR,T{i,3},'--','Color',cD,'LineWidth',1.0);
    plot(ax,tB,T{i,1},'-','Color',cB,'LineWidth',1.4);
    plot(ax,tR,T{i,2},'-','Color',cR,'LineWidth',1.7);
    grid(ax,'on'); box(ax,'on'); xlim(ax,[0 300]); title(ax,T{i,4}); ylabel(ax,T{i,5}); xlabel(ax,'Tempo [s]'); ax.FontSize=10;
end

out='/Users/kauemartinsdesouza/NOVODH/novo trabalho/Comparar LQR e PID/relatorio_missao/figs/valida_prefilter.png';
exportgraphics(fig,out,'Resolution',150,'BackgroundColor','white');
fprintf('\nFigura salva em: %s\n', out);
