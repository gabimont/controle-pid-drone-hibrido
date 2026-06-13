%% demo_prefilter_linear.m
%  Escolhe a constante de tempo tau do PRE-FILTRO de referencia (Opcao B):
%  a referencia de altitude passa por 1/(tau*s+1) antes da malha (ganhos
%  ORIGINAIS, b=1). O degrau vira rampa suave -> sem "teleporte" no profundor,
%  sem mexer no controlador nem no clamp. Mostra profundor, manete e altitude
%  para alguns tau, num degrau de 5 m. So analise (linear).

clear; clc; close all;
run('/Users/kauemartinsdesouza/NOVODH/PID/DH_inicializacao.m');   % C_alt original
R2D = 180/pi;

P = sys_long;
Calt   = pid(C_alt.Kp,C_alt.Ki);     Calt.u='e_h';     Calt.y='theta_ref_int';
Ctheta = pid(C_theta.Kp,C_theta.Ki); Ctheta.u='e_theta'; Ctheta.y='u_theta';
Cvel   = pid(C_vel.Kp,C_vel.Ki);     Cvel.u='e_VT';    Cvel.y='throttle';
Kqb = ss(Kq); Kqb.u='q'; Kqb.y='kq_q';
Sh = sumblk('e_h     = h_ref         - h');
St = sumblk('e_theta = theta_ref_int - theta');
Sv = sumblk('e_VT    = VT_ref        - VT');
Sd = sumblk('elevator = u_theta - kq_q');
CL = connect(P,Calt,Ctheta,Cvel,Kqb,Sh,St,Sv,Sd, {'h_ref','VT_ref'}, {'h','theta','elevator','throttle'});
CLh = CL(:,'h_ref');     % resposta a h_ref (VT_ref=0)

t = (0:0.02:120)'; Hstep = 5;
taus = [0 1.5 3 5];
cores = [0 0 0; 0 0.447 0.741; 0.851 0.325 0.098; 0.466 0.674 0.188];
nomes = arrayfun(@(x) sprintf('\\tau=%.1f s',x), taus, 'uni',0); nomes{1}='sem filtro';

H=cell(numel(taus),1); DE=H; DT=H;
rate=@(u) max(abs(diff(u)./diff(t)));
fprintf('\n=== Pre-filtro de referencia (linear, degrau de %d m) ===\n', Hstep);
fprintf('%-12s | profundor pico | TAXA prof. | h ts2[s]\n','filtro');
for i=1:numel(taus)
    if taus(i)==0, F=tf(1,1); else, F=tf(1,[taus(i) 1]); end
    y = step(Hstep*series(F,CLh), t);   % h theta elevator throttle
    H{i}=y(:,1); DE{i}=y(:,3)*R2D; DT{i}=y(:,4);
    tol=0.02*Hstep; idx=find(abs(H{i}-Hstep)>tol,1,'last'); ts=~isempty(idx)*t(min(idx+1,numel(t)));
    fprintf('%-12s | %8.3f deg  | %7.1f /s | %5.0f\n', nomes{i}, max(abs(DE{i})), rate(DE{i}), ts);
end

%% Figura
fig=figure('Color','w','Position',[40 40 1200 420]); try, theme(fig,'light'); catch, end
TL=tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
title(TL,'Pre-filtro de referencia de altitude (linear) — degrau de 5 m','FontWeight','bold');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(H), plot(ax,t,H{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,Hstep,'--','Color',[.5 .5 .5]); grid on; box on; xlim([0 80]);
title(ax,'Altitude  h/h_{ref}'); ylabel(ax,'m'); xlabel(ax,'t [s]'); legend(ax,nomes,'Location','southeast','FontSize',9);

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(DE), plot(ax,t,DE{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 40]);
title(ax,'PROFUNDOR  \Delta\delta_e (o teleporte)'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(DT), plot(ax,t,DT{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 40]);
title(ax,'Manete  \Delta\delta_T'); ylabel(ax,'[-]'); xlabel(ax,'t [s]');

out='/Users/kauemartinsdesouza/NOVODH/PID/Imagens/demo_prefilter.png';
exportgraphics(fig,out,'Resolution',150,'BackgroundColor','white');
fprintf('\nFigura salva em: %s\n', out);
