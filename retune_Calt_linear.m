%% retune_Calt_linear.m
%  Retune ISOLADO da malha de ALTITUDE no modelo LINEAR.
%  Monta a planta interna P_h (theta_ref -> h, com arfagem [C_theta+Kq] e
%  velocidade [C_vel] fechadas), valida o C_alt atual (margens), e projeta
%  versoes MAIS LENTAS (menor wc) com pidtune. Mostra o efeito no degrau de
%  altitude e, principalmente, na ATIVIDADE DE MANETE por comando de altitude
%  (o throttle e' saida da malha linear longitudinal).
%
%  Nada e' alterado nos arquivos do projeto — so analise.

clear; clc; close all;

run('/Users/kauemartinsdesouza/NOVODH/PID/DH_inicializacao.m');   % sys_long, C_alt, C_theta, C_vel, Kq, Ue

%% --- planta interna P_h: theta_ref_int -> h (malhas internas fechadas) ---
P = sys_long;                              % in: throttle,elevator | out: u,alpha,q,theta,h,VT
Ctheta = makePI(C_theta); Ctheta.u='e_theta'; Ctheta.y='u_theta';
Cvel   = makePI(C_vel);   Cvel.u='e_VT';      Cvel.y='throttle';
Kqb = ss(Kq); Kqb.u='q'; Kqb.y='kq_q';
St = sumblk('e_theta = theta_ref_int - theta');
Sv = sumblk('e_VT = VT_ref - VT');
Sd = sumblk('elevator = u_theta - kq_q');
Sh = sumblk('e_h = h_ref - h');

Pinner = connect(P,Ctheta,Cvel,Kqb,St,Sv,Sd, {'theta_ref_int','VT_ref'}, {'h','theta','VT','throttle'});
Ph = Pinner('h','theta_ref_int');          % SISO: theta_ref -> h (planta que o C_alt enxerga)

%% --- baseline + candidatos mais lentos ---
Calt0 = pid(C_alt.Kp, C_alt.Ki);           % atual
wc_alvo = [0.25 0.18 0.12];                % rad/s (baseline ~0.35)
cands = {'baseline (atual)', Calt0};
for w = wc_alvo
    Cw = pidtune(Ph, 'pi', w);
    cands(end+1,:) = {sprintf('wc=%.2f', w), Cw}; %#ok<SAGROW>
end

%% --- build full CL p/ cada candidato + metricas ---
t = (0:0.1:150)';
Hstep = 5;                                  % degrau de altitude [m]
cores = [0 0 0; 0 0.447 0.741; 0.851 0.325 0.098; 0.466 0.674 0.188];

fprintf('\n=== Retune isolado da malha de altitude (linear) ===\n');
fprintf('%-16s | Kp        Ki        | wc[rad/s]  PM[deg]  GM[dB] | ts h[s]  pico|theta|  pico|dThr|\n','config');
H=cell(size(cands,1),1); TH=H; VT=H; THR=H;
for i=1:size(cands,1)
    Calt = cands{i,2}; Calt.u='e_h'; Calt.y='theta_ref_int';
    CL = connect(P,Calt,Ctheta,Cvel,Kqb,Sh,St,Sv,Sd, {'h_ref','VT_ref'}, {'h','theta','VT','throttle'});
    y = step(Hstep*CL(:,'h_ref'), t);       % cols: h theta VT throttle
    H{i}=y(:,1); TH{i}=y(:,2); VT{i}=y(:,3); THR{i}=y(:,4);
    L = cands{i,2}*Ph; [Gm,Pm,~,Wcp] = margin(L);
    % ts 2% da altitude
    tol=0.02*Hstep; idx=find(abs(H{i}-Hstep)>tol,1,'last'); ts = isempty(idx)*0 + ~isempty(idx)*t(min(idx+1,numel(t)));
    [kp,ki]=piddata(cands{i,2});
    fprintf('%-16s | %-9.5f %-9.6f | %7.3f  %6.1f  %6.1f | %5.1f   %7.3f°   %7.4f\n', ...
        cands{i,1}, kp, ki, Wcp, Pm, 20*log10(Gm), ts, max(abs(TH{i}))*180/pi, max(abs(THR{i})));
end

%% ===================== Figura =====================
fig=figure('Color','w','Position',[40 40 1200 820]); try, theme(fig,'light'); catch, end
TL=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(TL,sprintf('Retune isolado da malha de altitude (linear) — degrau de %d m',Hstep),'FontWeight','bold');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(H), plot(ax,t,H{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,Hstep,'--','Color',[.5 .5 .5]); grid on; box on; xlim([0 150]);
title(ax,'Altitude  h/h_{ref}'); ylabel(ax,'m'); xlabel(ax,'t [s]');
legend(ax,cands(:,1),'Location','southeast','FontSize',9);

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(TH), plot(ax,t,TH{i}*180/pi,'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,10,':','Color',[.5 .5 .5]); yline(ax,-10,':','Color',[.5 .5 .5]); grid on; box on; xlim([0 150]);
title(ax,'Comando de arfagem  \theta  (pontilhado = clamp \pm10°)'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(VT), plot(ax,t,VT{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 150]);
title(ax,'Perturbacao de velocidade  \DeltaV_T  (gera o tranco na manete)'); ylabel(ax,'m/s'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(THR), plot(ax,t,THR{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 150]);
title(ax,'Atividade de MANETE  \Delta\delta_T  por comando de altitude'); ylabel(ax,'[-]'); xlabel(ax,'t [s]');

out='/Users/kauemartinsdesouza/NOVODH/PID/Imagens/retune_Calt_linear.png';
exportgraphics(fig,out,'Resolution',150,'BackgroundColor','white');
fprintf('\nFigura salva em: %s\n', out);

%% ---------- helper ----------
function C = makePI(s)
    if abs(s.Kd) < eps, C = pid(s.Kp, s.Ki);
    else, C = pid(s.Kp, s.Ki, s.Kd, 1/max(s.N,eps)); end
end
