%% valida_retune_missao.m
%  Valida na MISSAO NAO LINEAR (Caso 4) os retunes:
%     C_alt: wc 0.355 -> 0.18   (suaviza manete e PROFUNDOR nos degraus de h)
%     C_phi: x0.5               (suaviza AILERON nos degraus de proa)
%  Roda baseline (ganhos antigos) x retunado (ganhos novos) e sobrepoe os
%  4 atuadores em valor absoluto (linha pontilhada = trim).

clear; clc; bdclose all; close all;

p = strsplit(path, pathsep);
alvo = p(contains(p, ['novo trabalho' filesep 'PID']));
if ~isempty(alvo), rmpath(alvo{:}); end
run('/Users/kauemartinsdesouza/NOVODH/PID/DH_inicializacao.m');   % ja com retune
R2D = 180/pi;

% refs da missao (Caso 4)
VT_ref=Ve; VT_step_delta=15.2-Ve; VT_step_t=5;
psi_ref_init=0; psi_ref_final=deg2rad(5); psi_ref_t=50; psi_ref_t2=100; psi_ref_t3=150;
h_ref=he; h_step_init=0; h_step_final=5; h_step_t=80; h_step_t2=160; h_step_t3=240;
phi_step_final=0; att_alt=0;

%% --- RETUNADO (ganhos atuais do arquivo) ---
oR = sim('modelo_NL_DH_CL','StopTime','300'); tR=oR.tout; UR=oR.U.signals.values;

%% --- BASELINE (sobrescreve ganhos antigos) ---
C_alt.Kp=0.02875; C_alt.Ki=0.00259;
C_phi.Kp=-0.2831; C_phi.Ki=-0.2716;
oB = sim('modelo_NL_DH_CL','StopTime','300'); tB=oB.tout; UB=oB.U.signals.values;

%% --- superficies absolutas + trim ---
sig = @(U,c) U(:,c);
eR=UR(:,2)*R2D; aR=UR(:,3)*R2D; rR=UR(:,4)*R2D; TR=UR(:,1);
eB=UB(:,2)*R2D; aB=UB(:,3)*R2D; rB=UB(:,4)*R2D; TB=UB(:,1);
trim=[Ue(2)*R2D 0 0 Ue(1)];
ev=[5 50 80 100 150 160 240];

rate=@(t,u) max(abs(diff(u)./diff(t)));
fprintf('\n=== Validacao NL do retune (missao Caso 4) ===\n');
fprintf('%-10s | baseline pico/taxa | retunado pico/taxa\n','superf');
fprintf('Profundor  | %6.2f / %6.1f    | %6.2f / %6.1f  (deg, deg/s)\n', max(abs(eB-trim(1))),rate(tB,eB), max(abs(eR-trim(1))),rate(tR,eR));
fprintf('Aileron    | %6.3f / %6.1f    | %6.3f / %6.1f  (deg, deg/s)\n', max(abs(aB)),rate(tB,aB), max(abs(aR)),rate(tR,aR));
fprintf('Leme       | %6.3f / %6.2f    | %6.3f / %6.2f  (deg, deg/s)\n', max(abs(rB)),rate(tB,rB), max(abs(rR)),rate(tR,rR));
fprintf('Manete     | %6.3f / %6.3f    | %6.3f / %6.3f  ([-], 1/s)\n', max(abs(TB-trim(4))),rate(tB,TB), max(abs(TR-trim(4))),rate(tR,TR));

%% ===================== Figura 2x2 =====================
cB=[0.55 0.55 0.55]; cR=[0 0.447 0.741];
fig=figure('Color','w','Position',[40 40 1200 820]); try, theme(fig,'light'); catch, end
TL=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(TL,'Validacao NL do retune — Missao (Caso 4): baseline \times retunado','FontWeight','bold');

dados={tB,eB,tR,eR,trim(1),'Profundor  \delta_e','graus'; ...
       tB,aB,tR,aR,trim(2),'Aileron  \delta_a','graus'; ...
       tB,rB,tR,rR,trim(3),'Leme  \delta_r','graus'; ...
       tB,TB,tR,TR,trim(4),'Manete  \delta_T','[-]'};
for i=1:4
    ax=nexttile(TL); hold(ax,'on');
    for k=1:numel(ev), xline(ax,ev(k),'-','Color',[0.9 0.9 0.9]); end
    yline(ax,dados{i,5},':','Color',[0.5 0.5 0.5],'LineWidth',1.1);
    hB=plot(ax,dados{i,1},dados{i,2},'-','Color',cB,'LineWidth',1.4);
    hR=plot(ax,dados{i,3},dados{i,4},'-','Color',cR,'LineWidth',1.7);
    hold(ax,'off'); grid(ax,'on'); box(ax,'on'); xlim(ax,[0 300]);
    title(ax,dados{i,6}); ylabel(ax,dados{i,7}); xlabel(ax,'Tempo [s]'); ax.FontSize=10;
    if i==1, legend(ax,[hB hR],{'baseline','retunado'},'Location','best','FontSize',9); end
end

out='/Users/kauemartinsdesouza/NOVODH/novo trabalho/Comparar LQR e PID/relatorio_missao/figs/valida_retune_superficies.png';
exportgraphics(fig,out,'Resolution',150,'BackgroundColor','white');
fprintf('\nFigura salva em: %s\n', out);
