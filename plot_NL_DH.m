% plot_NL_DH.m
% =============================================================
% Plota a ultima simulacao do modelo_NL_DH_CL.slx em DUAS figuras:
%
%   Fig 1 - LONGITUDINAL:        h+ref | VT+ref
%                                theta+theta_ref | alpha
%                                profundor | throttle
%
%   Fig 2 - LATERO-DIRECIONAL:   psi+ref | phi+ref
%                                beta | p e r
%                                aileron | leme
%
% Le os resultados do workspace em qualquer das duas formas:
%   a) variavel "out" (Simulink.SimulationOutput) - out = sim('modelo_NL_DH_CL')
%      ou botao Run com "Single Simulation Output" ligado;
%   b) variaveis soltas Y, U, tout (+ theta_ref) - Run com "Single
%      Simulation Output" desligado.
%
% As referencias sao reconstruidas dos parametros do harness
% (DH_inicializacao): h_ref/h_step_*, VT_ref, psi_ref_*, phi_step_*,
% K_heading. theta_ref vem do log do modelo (mostra o clamp atuando).
%
% Uso tipico:
%   DH_inicializacao
%   h_step_final = 5; h_step_t = 20;        % (exemplo de manobra)
%   out = sim('modelo_NL_DH_CL');
%   plot_NL_DH
%
% Para salvar PNGs em "Imagens Apresentacao/":  salvar_png = true
% =============================================================

%% ---------- coleta resultados ----------
if ~exist('out','var')
    if exist('Y','var') && exist('U','var') && exist('tout','var')
        out = struct();
        out.tout = tout;
        out.Y    = Y;
        out.U    = U;
        if exist('theta_ref','var'), out.theta_ref = theta_ref; end
    else
        error(['Nenhum resultado de simulacao no workspace.\n' ...
               'Rode o modelo (Run em modelo_NL_DH_CL) ou:\n' ...
               '    out = sim(''modelo_NL_DH_CL'');\n' ...
               'antes de chamar plot_NL_DH.']);
    end
end
if ~exist('K_heading','var')
    warning('K_heading nao definido - assumindo 0. Rode DH_inicializacao antes.');
    K_heading = 0;
end

R2D = 180/pi;
t   = out.tout;
Ys  = out.Y.signals.values;   % [VT alpha beta gamma p q r phi theta psi ax ay az xN h ...]
Us  = out.U.signals.values;   % [throttle elevator aileron rudder]

VT_s    = Ys(:,1);
alpha_d = Ys(:,2)  * R2D;
beta_d  = Ys(:,3)  * R2D;
p_d     = Ys(:,5)  * R2D;
r_d     = Ys(:,7)  * R2D;
phi_d   = Ys(:,8)  * R2D;
theta_d = Ys(:,9)  * R2D;
psi_d   = Ys(:,10) * R2D;
h_s     = Ys(:,15);
thr_s   = Us(:,1);
elev_d  = Us(:,2) * R2D;
ail_d   = Us(:,3) * R2D;
rud_d   = Us(:,4) * R2D;

% trim inferido do primeiro instante (robusto a mudanca de condicao)
thr_trim  = thr_s(1);   elev_trim = elev_d(1);
ail_trim  = ail_d(1);   rud_trim  = rud_d(1);

%% ---------- referencias ----------
step_sig = @(t, v0, v1, ts) v0 + (v1 - v0) .* (t >= ts);

% h_ref = constante h_ref + Step_h_ref (so faz sentido com att_alt = 0)
if exist('h_ref','var')
    h_ref_sig = h_ref * ones(size(t));
else
    h_ref_sig = h_s(1) * ones(size(t));
end
if exist('h_step_final','var')
    h_ref_sig = h_ref_sig + step_sig(t, h_step_init, h_step_final, h_step_t);
end
malha_h_aberta = exist('att_alt','var') && att_alt > 0.5;
if malha_h_aberta, h_ref_sig = nan(size(t)); end

% VT_ref constante
if exist('VT_ref','var')
    VT_ref_sig = VT_ref * ones(size(t));
else
    VT_ref_sig = nan(size(t));
end

% theta_ref logado no modelo (inclui clamp; cobre att_alt = 0 ou 1)
if isfield(out,'theta_ref') || isprop(out,'theta_ref')
    theta_ref_d = out.theta_ref.signals.values * R2D;
    t_thref     = out.theta_ref.time;
else
    warning('Log theta_ref nao encontrado - theta_ref nao sera plotado.');
    theta_ref_d = nan(size(t));  t_thref = t;
end

% psi_ref do step parametrico
if exist('psi_ref_final','var')
    psi_ref_sig = step_sig(t, psi_ref_init, psi_ref_final, psi_ref_t);
else
    psi_ref_sig = zeros(size(t));
end
psi_ref_d = psi_ref_sig * R2D;

% phi_ref = K_heading*(psi_ref - psi) + Step_phi_ref  (mesma soma do modelo)
phi_ref_sig = K_heading * (psi_ref_sig - Ys(:,10));
if exist('phi_step_final','var')
    phi_ref_sig = phi_ref_sig + step_sig(t, phi_step_init, phi_step_final, phi_step_t);
end
phi_ref_d = phi_ref_sig * R2D;

%% ---------- resumo da manobra (titulo / nome de arquivo) ----------
ex = {};
if exist('h_ref','var') && exist('he','var') && abs(h_ref-he) > 1e-6
    ex{end+1} = sprintf('h%+dm', round(h_ref-he));
end
if exist('h_step_final','var') && abs(h_step_final-h_step_init) > 1e-6
    ex{end+1} = sprintf('hstep%+dm@%gs', round(h_step_final-h_step_init), h_step_t);
end
if exist('VT_ref','var') && exist('Ve','var') && abs(VT_ref-Ve) > 1e-6
    ex{end+1} = sprintf('VT%+dms', round(VT_ref-Ve));
end
if exist('psi_ref_final','var') && abs(psi_ref_final-psi_ref_init) > 1e-6
    ex{end+1} = sprintf('psi%+ddeg@%gs', round((psi_ref_final-psi_ref_init)*R2D), psi_ref_t);
end
if exist('phi_step_final','var') && abs(phi_step_final-phi_step_init) > 1e-6
    ex{end+1} = sprintf('phi%+ddeg@%gs', round((phi_step_final-phi_step_init)*R2D), phi_step_t);
end
if malha_h_aberta && exist('theta_step_final','var') && abs(theta_step_final-theta_step_init) > 1e-6
    ex{end+1} = sprintf('thetastep%+ddeg@%gs', round((theta_step_final-theta_step_init)*R2D), theta_step_t);
end
if isempty(ex), manobra = 'trim'; else, manobra = strjoin(ex, ', '); end

%% ---------- estilo ----------
LW   = 1.4;
cSig = [0.00 0.45 0.74];    % azul (resposta NL)
cSig2= [0.85 0.33 0.10];    % laranja (2o sinal do painel)
cRef = [0.20 0.20 0.20];    % cinza escuro tracejado (referencia)
cTrm = [0.55 0.55 0.55];    % cinza claro pontilhado (trim)

%% ================= FIG 1 - LONGITUDINAL =================
f1 = figure('Name','NL - Longitudinal','Position',[80 80 980 760],'Color','w');
try f1.Theme = 'light'; catch, end
tl1 = tiledlayout(f1, 3, 2, 'TileSpacing','compact', 'Padding','compact');
title(tl1, sprintf('Modelo NL - Longitudinal   (%s)', manobra), 'FontWeight','bold');

nexttile; hold on; grid on;
plot(t, h_ref_sig, '--', 'Color', cRef, 'LineWidth', LW);
plot(t, h_s, 'Color', cSig, 'LineWidth', LW);
ylabel('h [m]');
if malha_h_aberta
    title('Altitude (malha aberta: att\_alt = 1)');
else
    title('Altitude'); legend({'h_{ref}','h'}, 'Location','best');
end
ylim(pad_lims([h_s; h_ref_sig(~isnan(h_ref_sig))]));

nexttile; hold on; grid on;
plot(t, VT_ref_sig, '--', 'Color', cRef, 'LineWidth', LW);
plot(t, VT_s, 'Color', cSig, 'LineWidth', LW);
ylabel('V_T [m/s]'); title('Velocidade'); legend({'V_{T,ref}','V_T'}, 'Location','best');
ylim(pad_lims([VT_s; VT_ref_sig(~isnan(VT_ref_sig))]));

nexttile; hold on; grid on;
plot(t_thref, theta_ref_d, '--', 'Color', cRef, 'LineWidth', LW);
plot(t, theta_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\theta [deg]'); title('Arfagem (\theta_{ref} = saida do C_{alt} c/ clamp)');
legend({'\theta_{ref}','\theta'}, 'Location','best');
ylim(pad_lims([theta_d; theta_ref_d(~isnan(theta_ref_d))]));

nexttile; hold on; grid on;
plot(t, alpha_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\alpha [deg]'); title('Angulo de ataque');
ylim(pad_lims(alpha_d));

nexttile; hold on; grid on;
yline(elev_trim, ':', 'Color', cTrm, 'LineWidth', 1.2);
plot(t, elev_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\delta_e [deg]'); xlabel('t [s]');
title('Profundor (absoluto; pontilhado = trim)');
ylim(pad_lims([elev_d; elev_trim]));

nexttile; hold on; grid on;
yline(thr_trim, ':', 'Color', cTrm, 'LineWidth', 1.2);
plot(t, thr_s, 'Color', cSig, 'LineWidth', LW);
ylabel('\delta_T [-]'); xlabel('t [s]');
title('Throttle (absoluto; pontilhado = trim)');
ylim(pad_lims([thr_s; thr_trim]));

%% ================= FIG 2 - LATERO-DIRECIONAL =================
f2 = figure('Name','NL - Latero-direcional','Position',[140 60 980 760],'Color','w');
try f2.Theme = 'light'; catch, end
tl2 = tiledlayout(f2, 3, 2, 'TileSpacing','compact', 'Padding','compact');
title(tl2, sprintf('Modelo NL - Latero-direcional   (%s)', manobra), 'FontWeight','bold');

nexttile; hold on; grid on;
plot(t, psi_ref_d, '--', 'Color', cRef, 'LineWidth', LW);
plot(t, psi_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\psi [deg]'); title('Heading'); legend({'\psi_{ref}','\psi'}, 'Location','best');
ylim(pad_lims([psi_d; psi_ref_d]));

nexttile; hold on; grid on;
plot(t, phi_ref_d, '--', 'Color', cRef, 'LineWidth', LW);
plot(t, phi_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\phi [deg]'); title('Rolamento (\phi_{ref} = K_{head}\cdote_\psi + step)');
legend({'\phi_{ref}','\phi'}, 'Location','best');
ylim(pad_lims([phi_d; phi_ref_d]));

nexttile; hold on; grid on;
plot(t, beta_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\beta [deg]'); title('Derrapagem');
ylim(pad_lims(beta_d));

nexttile; hold on; grid on;
plot(t, p_d, 'Color', cSig, 'LineWidth', LW);
plot(t, r_d, 'Color', cSig2, 'LineWidth', LW);
ylabel('[deg/s]'); title('Taxas p e r'); legend({'p','r'}, 'Location','best');
ylim(pad_lims([p_d; r_d]));

nexttile; hold on; grid on;
yline(ail_trim, ':', 'Color', cTrm, 'LineWidth', 1.2);
plot(t, ail_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\delta_a [deg]'); xlabel('t [s]');
title('Aileron (absoluto; pontilhado = trim)');
ylim(pad_lims([ail_d; ail_trim]));

nexttile; hold on; grid on;
yline(rud_trim, ':', 'Color', cTrm, 'LineWidth', 1.2);
plot(t, rud_d, 'Color', cSig, 'LineWidth', LW);
ylabel('\delta_r [deg]'); xlabel('t [s]');
title('Leme (absoluto; pontilhado = trim)');
ylim(pad_lims([rud_d; rud_trim]));

%% ---------- export opcional ----------
if exist('salvar_png','var') && salvar_png
    outDir = fullfile(fileparts(mfilename('fullpath')), 'Imagens Apresentacao');
    if ~isfolder(outDir), mkdir(outDir); end
    tag = regexprep(manobra, '[^a-zA-Z0-9+@.-]', '_');
    exportgraphics(f1, fullfile(outDir, sprintf('NL_long_%s.png',  tag)), ...
                   'Resolution', 150, 'BackgroundColor', 'white');
    exportgraphics(f2, fullfile(outDir, sprintf('NL_latero_%s.png', tag)), ...
                   'Resolution', 150, 'BackgroundColor', 'white');
    fprintf('PNGs salvos em %s (NL_long_%s.png / NL_latero_%s.png)\n', outDir, tag, tag);
end

%% ---------- helper ----------
function L = pad_lims(v)
    v = v(~isnan(v));
    lo = min(v); hi = max(v); d = hi - lo;
    if d < 1e-9, d = max(abs(hi), 1) * 0.1; end
    L = [lo - 0.1*d, hi + 0.1*d];
end
