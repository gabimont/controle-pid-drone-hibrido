% gerar_fig_atuador.m
% Figura para o relatorio: esforco de atuador com servo realista vs atuador
% ideal, no profundor (manobra de altitude) e no aileron (PA de rolamento).
run(fullfile(fileparts(mfilename('fullpath')),'DH_inicializacao.m'));
R2D = 180/pi;
act_real = act;                                    % guarda specs do servo
act_ideal = act; act_ideal.rate = deg2rad(5000); act_ideal.bw = 100; act_ideal.tau = 1/act_ideal.bw;
rate_lim_deg = rad2deg(act_real.rate);

% ---------- profundor: h+10 m @ 10 s ----------
h_ref = he; h_step_final = 10; h_step_t = 10; K_heading_save = K_heading;
act = act_real;  oA = sim('modelo_NL_DH_CL');      % servo realista
act = act_ideal; oI = sim('modelo_NL_DH_CL');      % ~ideal (1 passo)
tA=oA.tout; deA=oA.U.signals.values(:,2)*R2D;
tI=oI.tout; deI=oI.U.signals.values(:,2)*R2D;

% ---------- aileron: PA de rolamento phi+20 deg @ 5 s ----------
h_step_final = 0; K_heading = 0; phi_step_final = deg2rad(20); phi_step_t = 5;
act = act_real;  oA2 = sim('modelo_NL_DH_CL');
act = act_ideal; oI2 = sim('modelo_NL_DH_CL');
tA2=oA2.tout; aiA=oA2.U.signals.values(:,3)*R2D;
tI2=oI2.tout; aiI=oI2.U.signals.values(:,3)*R2D;
act = act_real;

% ---------- figura ----------
cI=[0.6 0.6 0.6]; cS=[0 0.45 0.74]; cR=[0.85 0.33 0.10];
f=figure('Color','w','Position',[60 60 1080 720]); try f.Theme='light'; catch, end
tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
title(tl,'Esforco de atuador: servo realista (rate+banda) vs atuador ideal','FontWeight','bold');

nexttile; hold on; grid on;
plot(tI,deI,'--','Color',cI,'LineWidth',1.3);
plot(tA,deA,'Color',cS,'LineWidth',1.6);
xlabel('t [s]'); ylabel('\delta_e [deg]'); title('Profundor — manobra h+10 m'); xlim([9 22]);
legend({'atuador ideal','servo'},'Location','northeast');

nexttile; hold on; grid on;
plot(tI(1:end-1),diff(deI)./diff(tI),'--','Color',cI,'LineWidth',1.1);
plot(tA(1:end-1),diff(deA)./diff(tA),'Color',cR,'LineWidth',1.4);
yline(rate_lim_deg,':k','LineWidth',1.2); yline(-rate_lim_deg,':k','LineWidth',1.2);
xlabel('t [s]'); ylabel('d\delta_e/dt [deg/s]');
title(sprintf('Taxa do profundor (limite servo \\pm%g deg/s)',rate_lim_deg)); xlim([9 22]);

nexttile; hold on; grid on;
plot(tI2,aiI,'--','Color',cI,'LineWidth',1.3);
plot(tA2,aiA,'Color',cS,'LineWidth',1.6);
xlabel('t [s]'); ylabel('\delta_a [deg]'); title('Aileron — PA de rolamento \phi+20 deg'); xlim([4 14]);
legend({'atuador ideal','servo'},'Location','southeast');

nexttile; hold on; grid on;
plot(tI2(1:end-1),diff(aiI)./diff(tI2),'--','Color',cI,'LineWidth',1.1);
plot(tA2(1:end-1),diff(aiA)./diff(tA2),'Color',cR,'LineWidth',1.4);
yline(rate_lim_deg,':k','LineWidth',1.2); yline(-rate_lim_deg,':k','LineWidth',1.2);
xlabel('t [s]'); ylabel('d\delta_a/dt [deg/s]');
title(sprintf('Taxa do aileron (limite servo \\pm%g deg/s)',rate_lim_deg)); xlim([4 14]);

outp = fullfile(fileparts(mfilename('fullpath')),'Imagens','esforco_atuador_servo.png');
exportgraphics(f, outp, 'Resolution',150,'BackgroundColor','white');
fprintf('figura salva em %s\n', outp);
