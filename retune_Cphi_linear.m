%% retune_Cphi_linear.m
%  Retune ISOLADO da malha LATERAL (rolamento) no modelo LINEAR, p/ suavizar
%  o AILERON nos degraus de proa. O aileron e' saida da malha de rolamento
%  (C_phi): no degrau de psi, o heading (K_heading) manda um phi_ref em
%  degrau e o C_phi (rapido, wc~2) reage com um bico de aileron.
%  Amaciar C_phi (menor wc) deixa o aileron mais suave; custo = rolamento/
%  proa mais lentos. Mostra psi, phi, AILERON e beta (coordenacao) p/ um
%  degrau de proa de 5 graus.  So analise — nada e' alterado nos arquivos.

clear; clc; close all;

run('/Users/kauemartinsdesouza/NOVODH/PID/DH_inicializacao.m');   % sys_lat, C_phi, Kr, K_heading
R2D = 180/pi;

%% --- blocos fixos da malha lateral ---
Gw = tf([1 0],[1 1])*(-Kr);  Gw.u='r'; Gw.y='rudder';      % yaw damper (washout)
Khead = ss(K_heading); Khead.u='e_psi'; Khead.y='phi_ref'; % heading (ganho)
Spsi = sumblk('e_psi = psi_ref - psi');
Sphi = sumblk('e_phi = phi_ref - phi');

% planta da malha de rolamento: aileron -> phi (yaw damper fechado)
P_phi = connect(sys_lat, Gw, {'aileron'}, {'phi'});

%% --- baseline + candidatos mais suaves (escala em C_phi) ---
Cphi0 = pid(C_phi.Kp, C_phi.Ki);
escalas = [1 0.5 0.3 0.2];
nomes = {'baseline (atual)','C_\phi \times0.5','C_\phi \times0.3','C_\phi \times0.2'};
cands = cell(numel(escalas),1);
for i=1:numel(escalas), cands{i} = escalas(i)*Cphi0; end

t = (0:0.02:40)';
psi_step = deg2rad(5);
cores = [0 0 0; 0 0.447 0.741; 0.851 0.325 0.098; 0.466 0.674 0.188];

fprintf('\n=== Retune isolado da malha de rolamento (linear) — degrau de proa 5 deg ===\n');
fprintf('%-16s | wc_phi[rad/s] PM[deg] | pico|aileron|  ts_psi[s]  pico|beta|\n','config');
PSI=cell(numel(cands),1); PHI=PSI; AIL=PSI; BET=PSI;
for i=1:numel(cands)
    Cphi = cands{i}; Cphi.u='e_phi'; Cphi.y='aileron';
    CL = connect(sys_lat, Cphi, Gw, Khead, Spsi, Sphi, {'psi_ref'}, {'psi','phi','aileron','beta'});
    y = step(psi_step*CL, t);   % cols: psi phi aileron beta
    PSI{i}=y(:,1)*R2D; PHI{i}=y(:,2)*R2D; AIL{i}=y(:,3)*R2D; BET{i}=y(:,4)*R2D;
    L = cands{i}*P_phi; [~,Pm,~,Wcp]=margin(L);
    psi_f = PSI{i}(end); tol=0.02*abs(psi_f);
    idx=find(abs(PSI{i}-psi_f)>tol,1,'last'); ts = ~isempty(idx)*t(min(idx+1,numel(t)));
    fprintf('%-16s | %8.3f    %6.1f | %9.4f deg  %6.1f    %8.4f deg\n', ...
        nomes{i}, Wcp, Pm, max(abs(AIL{i})), ts, max(abs(BET{i})));
end

%% ===================== Figura =====================
fig=figure('Color','w','Position',[40 40 1200 820]); try, theme(fig,'light'); catch, end
TL=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(TL,'Retune isolado da malha de rolamento (linear) — degrau de proa 5°','FontWeight','bold');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(PSI), plot(ax,t,PSI{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,5,'--','Color',[.5 .5 .5]); grid on; box on; xlim([0 40]);
title(ax,'Proa  \psi/\psi_{ref}'); ylabel(ax,'deg'); xlabel(ax,'t [s]');
legend(ax,nomes,'Location','southeast','FontSize',9);

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(PHI), plot(ax,t,PHI{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 40]);
title(ax,'Rolamento  \phi  (comando da curva coordenada)'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(AIL), plot(ax,t,AIL{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 40]);
title(ax,'AILERON  \delta_a  (o que queremos suavizar)'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

ax=nexttile(TL); hold(ax,'on'); for i=1:numel(BET), plot(ax,t,BET{i},'Color',cores(i,:),'LineWidth',1.6); end
yline(ax,0,'-','Color',[.7 .7 .7]); grid on; box on; xlim([0 40]);
title(ax,'Derrapagem  \beta  (qualidade de coordenacao)'); ylabel(ax,'deg'); xlabel(ax,'t [s]');

out='/Users/kauemartinsdesouza/NOVODH/PID/Imagens/retune_Cphi_linear.png';
exportgraphics(fig,out,'Resolution',150,'BackgroundColor','white');
fprintf('\nFigura salva em: %s\n', out);
