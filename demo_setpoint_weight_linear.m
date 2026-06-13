%% demo_setpoint_weight_linear.m
%  Mostra no LINEAR que o "teleporte" do profundor no degrau de altitude e' o
%  CHUTE PROPORCIONAL (proportional kick) do C_alt, e que SETPOINT WEIGHTING
%  b=0 (PID 2-DOF: termo P age so na saida medida, nao na referencia) o REMOVE
%  — deixando o comando suave como o LQR, SEM mudar banda/margens (o caminho de
%  realimentacao C_fb e' o mesmo).
%
%  C_alt 2-DOF:  theta_ref = (Kp*b + Ki/s)*h_ref  -  (Kp + Ki/s)*h
%     b=1 -> PI no erro (ATUAL, tem teleporte)
%     b=0 -> P so na saida (sem teleporte)

clear; clc; close all;
run('/Users/kauemartinsdesouza/NOVODH/PID/DH_inicializacao.m');   % C_alt ja retunado (wc=0.18)
R2D = 180/pi;

P = sys_long;
Ctheta = pid(C_theta.Kp,C_theta.Ki); Ctheta.u='e_theta'; Ctheta.y='u_theta';
Cvel   = pid(C_vel.Kp,C_vel.Ki);     Cvel.u='e_VT';      Cvel.y='throttle';
Kqb = ss(Kq); Kqb.u='q'; Kqb.y='kq_q';
St = sumblk('e_theta = theta_ref_int - theta');
Sv = sumblk('e_VT = VT_ref - VT');
Sd = sumblk('elevator = u_theta - kq_q');

Kp = C_alt.Kp; Ki = C_alt.Ki;
Cfb = pid(Kp,Ki); Cfb.u='h'; Cfb.y='tr_fb';           % caminho de realimentacao (igual nos dois)
Salt = sumblk('theta_ref_int = tr_ff - tr_fb');

t = (0:0.02:120)'; Hstep = 5;
build = @(Cff) connect(P,Cff,Cfb,Salt,Ctheta,Cvel,Kqb,St,Sv,Sd, ...
                       {'h_ref','VT_ref'}, {'h','theta','elevator','throttle'});

% b=1 (atual): Cff = Kp + Ki/s  (= PI no erro)
Cff1 = pid(Kp,Ki); Cff1.u='h_ref'; Cff1.y='tr_ff';
y1 = step(Hstep*build(Cff1), t);

% b=0: Cff = Ki/s  (so integral na referencia)
Cff0 = tf(Ki,[1 0]); Cff0.u='h_ref'; Cff0.y='tr_ff';
y0 = step(Hstep*build(Cff0), t);

% colunas: h theta elevator(delta) throttle(delta)
de1 = y1(:,3)*R2D; de0 = y0(:,3)*R2D;
rate = @(u) max(abs(diff(u)./diff(t)));
fprintf('\n=== Setpoint weighting no C_alt (linear, degrau de %d m) ===\n', Hstep);
fprintf('b=1 (atual)  : profundor pico %.3f deg | TAXA pico %.1f deg/s | h ts2 ~%.0f s\n', max(abs(de1)), rate(de1), tt(t,y1(:,1),Hstep));
fprintf('b=0 (s.weight): profundor pico %.3f deg | TAXA pico %.1f deg/s | h ts2 ~%.0f s\n', max(abs(de0)), rate(de0), tt(t,y0(:,1),Hstep));

%% ===================== Figura =====================
c1=[0.55 0.55 0.55]; c0=[0 0.447 0.741];
fig=figure('Color','w','Position',[40 40 1200 820]); try, theme(fig,'light'); catch, end
TL=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(TL,'Setpoint weighting no C_{alt} (linear) — remove o "teleporte" do profundor','FontWeight','bold');

ax=nexttile(TL); hold(ax,'on');
plot(ax,t,y1(:,1),'-','Color',c1,'LineWidth',1.5); plot(ax,t,y0(:,1),'-','Color',c0,'LineWidth',1.7);
yline(ax,Hstep,'--','Color',[.5 .5 .5]); grid on; box on; xlim([0 120]);
title(ax,'Altitude  h/h_{ref}'); ylabel(ax,'m'); xlabel(ax,'t [s]');
legend(ax,{'b=1 (atual)','b=0 (setpoint weighting)'},'Location','southeast','FontSize',9);

ax=nexttile(TL); hold(ax,'on');
plot(ax,t,y1(:,2)*R2D,'-','Color',c1,'LineWidth',1.5); plot(ax,t,y0(:,2)*R2D,'-','Color',c0,'LineWidth',1.7);
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 120]);
title(ax,'Comando de arfagem  \theta'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on');
plot(ax,t,de1,'-','Color',c1,'LineWidth',1.5); plot(ax,t,de0,'-','Color',c0,'LineWidth',1.7);
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 120]);
title(ax,'PROFUNDOR  \Delta\delta_e  (o "teleporte")'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on');
plot(ax,t,y1(:,4),'-','Color',c1,'LineWidth',1.5); plot(ax,t,y0(:,4),'-','Color',c0,'LineWidth',1.7);
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 120]);
title(ax,'Manete  \Delta\delta_T'); ylabel(ax,'[-]'); xlabel(ax,'t [s]');

out='/Users/kauemartinsdesouza/NOVODH/PID/Imagens/demo_setpoint_weight.png';
exportgraphics(fig,out,'Resolution',150,'BackgroundColor','white');
fprintf('\nFigura salva em: %s\n', out);

function ts = tt(t,h,Hf)
    tol=0.02*Hf; idx=find(abs(h-Hf)>tol,1,'last'); ts = ~isempty(idx)*t(min(idx+1,numel(t)));
end
