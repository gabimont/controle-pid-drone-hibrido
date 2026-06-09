% plot_PID.m
% =============================================================
% Plota os resultados de uma simulacao do modelo NL closed-loop
% (modelo_NL_DH_CL.slx) em 10 paineis 5x2 (mesmo layout do scope
% do LQR do Mirko, mas com SEU controle PID).
%
% Paineis (linha por linha, esquerda -> direita):
%   q          | r
%   theta+ref  | ail
%   elev       | psi+ref
%   h = -xD    | p
%   phi+ref    | rud
%
% O script le tres coisas do workspace:
%   1) resultados da simulacao, em uma destas formas:
%        a) variavel "out" (Simulink.SimulationOutput) — produzida por
%           sim('modelo_NL_DH_CL') OU pelo botao Run com "Single
%           Simulation Output" ligado.
%        b) variaveis soltas Y, U, tout (+ theta_ref opcional) — botao
%           Run com "Single Simulation Output" desligado, os blocos
%           To Workspace salvam direto no base.
%   2) K_heading       - ganho da malha de heading (de DH_inicializacao)
%   3) psi_ref_init/final/t - parametros do step de psi (de DH_inicializacao)
%
% Uso tipico:
%   DH_inicializacao           % carrega refs/ganhos default
%   % (opcional: excitar alguma malha, ex h_ref = he + 50)
%   % roda o Simulink (botao Run em modelo_NL_DH_CL  ou  out = sim(...))
%   plot_PID
% =============================================================

%% ---------- checagens ----------
% Aceita resultados em "out" ou em Y/U/tout soltos no base workspace.
if ~exist('out','var')
    if exist('Y','var') && exist('U','var') && exist('tout','var')
        out = struct();
        out.tout = tout;
        out.Y    = Y;
        out.U    = U;
        if exist('theta_ref','var')
            out.theta_ref = theta_ref;
        end
    else
        error(['Nenhum resultado de simulacao encontrado no workspace.\n' ...
               'Rode o modelo Simulink (Run em modelo_NL_DH_CL) ou:\n' ...
               '    out = sim(''modelo_NL_DH_CL'');\n' ...
               'antes de chamar plot_PID.']);
    end
end
if ~exist('K_heading','var')
    warning(['K_heading nao definido — vou assumir 0 (sem malha de heading). ' ...
             'Rode DH_inicializacao antes para ter o valor correto.']);
    K_heading = 0;
end

%% ---------- sinais da planta ----------
t = out.tout;
Y = out.Y.signals.values;     % [VT alpha beta gamma p q r phi theta psi ax ay az xN h mi lambda qdot]
U = out.U.signals.values;     % [throttle elevator aileron rudder]

R2D = 180/pi;
p_deg     = Y(:,5)  * R2D;
q_deg     = Y(:,6)  * R2D;
r_deg     = Y(:,7)  * R2D;
phi_deg   = Y(:,8)  * R2D;
theta_deg = Y(:,9)  * R2D;
psi_deg   = Y(:,10) * R2D;
h         = Y(:,15);
elev_deg  = U(:,2)  * R2D;
ail_deg   = U(:,3)  * R2D;
rud_deg   = U(:,4)  * R2D;

%% ---------- referencias ----------
% psi_ref: construida do step parametrico (psi_ref_init/final/t)
psi_ref = zeros(size(t));
if exist('psi_ref_init','var') && exist('psi_ref_final','var') && exist('psi_ref_t','var')
    psi_ref(t <  psi_ref_t) = psi_ref_init;
    psi_ref(t >= psi_ref_t) = psi_ref_final;
end
psi_ref_deg = psi_ref * R2D;

% phi_ref: saida da malha de heading = K_heading*(psi_ref - psi)
phi_ref_deg = K_heading * (psi_ref - Y(:,10)) * R2D;

% theta_ref: vem direto do modelo via ToWorkspace
if isfield(out,'theta_ref') || isprop(out,'theta_ref')
    theta_ref_deg = out.theta_ref.signals.values * R2D;
else
    warning('out.theta_ref nao encontrado — theta_ref nao sera plotado.');
    theta_ref_deg = nan(size(t));
end

%% ---------- helpers ----------
LW   = 1.4;
cSig = [1.0 0.55 0.0];      % laranja (PID)
cRef = [1.0 1.0 1.0];       % branco tracejado

set_dark = @() set(gca,'Color','k','XColor','w','YColor','w', ...
                       'GridColor','w','GridAlpha',0.3);

%% ---------- figura ----------
figure('Color','k','Position',[100 100 1100 1400]);

% --- (1,1) q ---
subplot(5,2,1); plot(t,q_deg,'Color',cSig,'LineWidth',LW); grid on;
title('q','Color','w'); ylabel('deg/s','Color','w');
ylim(pad_ylim(q_deg)); set_dark();

% --- (1,2) r ---
subplot(5,2,2); plot(t,r_deg,'Color',cSig,'LineWidth',LW); grid on;
title('r','Color','w'); ylabel('deg/s','Color','w');
ylim(pad_ylim(r_deg)); set_dark();

% --- (2,1) theta + ref ---
subplot(5,2,3); hold on;
plot(t,theta_ref_deg,'--','Color',cRef,'LineWidth',LW);
plot(t,theta_deg,'Color',cSig,'LineWidth',LW);
grid on; title('theta','Color','w'); ylabel('deg','Color','w');
legend({'ref','theta'},'TextColor','w','Color','k','Box','off');
ylim(pad_ylim([theta_deg; theta_ref_deg(~isnan(theta_ref_deg))]));
set_dark();

% --- (2,2) ail ---
subplot(5,2,4); plot(t,ail_deg,'Color',cSig,'LineWidth',LW); grid on;
title('ail','Color','w'); ylabel('deg','Color','w');
ylim(pad_ylim(ail_deg)); set_dark();

% --- (3,1) elev ---
subplot(5,2,5); plot(t,elev_deg,'Color',cSig,'LineWidth',LW); grid on;
title('elev','Color','w'); ylabel('deg','Color','w');
ylim(pad_ylim(elev_deg)); set_dark();

% --- (3,2) psi + ref ---
subplot(5,2,6); hold on;
plot(t,psi_ref_deg,'--','Color',cRef,'LineWidth',LW);
plot(t,psi_deg,'Color',cSig,'LineWidth',LW);
grid on; title('psi','Color','w'); ylabel('deg','Color','w');
legend({'ref','psi'},'TextColor','w','Color','k','Box','off');
ylim(pad_ylim([psi_deg; psi_ref_deg])); set_dark();

% --- (4,1) h = -xD ---
subplot(5,2,7); plot(t,h,'Color',cSig,'LineWidth',LW); grid on;
title('h = -xD','Color','w'); ylabel('m','Color','w');
ylim(pad_ylim(h)); set_dark();

% --- (4,2) p ---
subplot(5,2,8); plot(t,p_deg,'Color',cSig,'LineWidth',LW); grid on;
title('p','Color','w'); ylabel('deg/s','Color','w');
ylim(pad_ylim(p_deg)); set_dark();

% --- (5,1) phi + ref ---
subplot(5,2,9); hold on;
plot(t,phi_ref_deg,'--','Color',cRef,'LineWidth',LW);
plot(t,phi_deg,'Color',cSig,'LineWidth',LW);
grid on; title('phi','Color','w'); ylabel('deg','Color','w');
xlabel('t [s]','Color','w');
legend({'ref','phi'},'TextColor','w','Color','k','Box','off');
ylim(pad_ylim([phi_deg; phi_ref_deg])); set_dark();

% --- (5,2) rud ---
subplot(5,2,10); plot(t,rud_deg,'Color',cSig,'LineWidth',LW); grid on;
title('rud','Color','w'); ylabel('deg','Color','w');
xlabel('t [s]','Color','w');
ylim(pad_ylim(rud_deg)); set_dark();

%% ---------- imagens individuais para apresentacao ----------
% Salva um PNG por sinal em "Imagens Apresentacao/" no diretorio do
% script. O nome do arquivo recebe um prefixo derivado da excitacao
% (h_ref, VT_ref, psi_ref_final, theta_step_final) — assim runs
% diferentes nao se sobrescrevem.
%
% Para pular este passo: defina  save_apresentacao = false  no
% workspace antes de rodar plot_PID.
if ~exist('save_apresentacao','var') || save_apresentacao
    % --- detecta excitacoes ativas e monta prefixo ---
    ex_parts = {};
    if exist('h_ref','var') && exist('he','var') && abs(h_ref - he) > 1e-6
        ex_parts{end+1} = sprintf('h%+dm', round(h_ref - he));
    end
    if exist('VT_ref','var') && exist('Ve','var') && abs(VT_ref - Ve) > 1e-6
        ex_parts{end+1} = sprintf('VT%+dms', round(VT_ref - Ve));
    end
    if exist('psi_ref_final','var') && abs(psi_ref_final) > 1e-6
        ex_parts{end+1} = sprintf('psi%+ddeg', round(psi_ref_final * R2D));
    end
    if exist('att_alt','var') && att_alt > 0.5 && ...
       exist('theta_step_final','var') && abs(theta_step_final) > 1e-6
        ex_parts{end+1} = sprintf('thetaStep%+ddeg', ...
                                  round(theta_step_final * R2D));
    end
    if isempty(ex_parts)
        prefix = 'trim';
    elseif numel(ex_parts) == 1
        prefix = ex_parts{1};
    else
        prefix = ['combinado_' strjoin(ex_parts, '_')];
    end

    % --- pasta de saida (ao lado do plot_PID.m) ---
    try
        script_dir = fileparts(mfilename('fullpath'));
    catch
        script_dir = pwd;
    end
    img_dir = fullfile(script_dir, 'Imagens Apresentacao');
    if ~exist(img_dir, 'dir'); mkdir(img_dir); end

    % --- sinais extras (nao plotados na figura principal) ---
    VT_ms    = Y(:,1);     % velocidade
    throttle = U(:,1);     % throttle (adim, 0..1)

    % --- referencias constantes (mostradas tracejadas mesmo sem excitacao) ---
    if exist('h_ref','var'),  h_ref_vec  = h_ref  * ones(size(t)); else, h_ref_vec  = []; end
    if exist('VT_ref','var'), VT_ref_vec = VT_ref * ones(size(t)); else, VT_ref_vec = []; end

    % --- catalogo: {sinal, ref opcional, titulo TeX, ylabel, filename} ---
    plots = {
        q_deg,        [],              'q',         'deg/s',  'q';
        r_deg,        [],              'r',         'deg/s',  'r';
        p_deg,        [],              'p',         'deg/s',  'p';
        theta_deg,    theta_ref_deg,   '\theta',    'deg',    'theta';
        phi_deg,      phi_ref_deg,     '\phi',      'deg',    'phi';
        psi_deg,      psi_ref_deg,     '\psi',      'deg',    'psi';
        elev_deg,     [],              'elev',      'deg',    'elev';
        ail_deg,      [],              'ail',       'deg',    'ail';
        rud_deg,      [],              'rud',       'deg',    'rud';
        h,            h_ref_vec,       'h',         'm',      'h';
        VT_ms,        VT_ref_vec,      'V_T',       'm/s',    'VT';
        throttle,     [],              'throttle',  '-',      'throttle';
    };

    % --- estilo claro para slides ---
    cSig_p  = [0.85 0.33 0.10];   % laranja escuro (sinal)
    cRef_p  = [0    0    0   ];   % preto (ref)
    cGrid_p = [0.4  0.4  0.4 ];   % cinza (grid)
    LW_p    = 1.8;

    % Forca figuras subsequentes invisiveis (evita roubo de foco no macOS).
    % Restauramos no fim do loop — sem onCleanup (que vaza entre scripts).
    prev_vis = get(0, 'DefaultFigureVisible');
    set(0, 'DefaultFigureVisible', 'off');

    for i = 1:size(plots,1)
        sig  = plots{i,1};
        ref  = plots{i,2};
        ttl  = plots{i,3};
        ylbl = plots{i,4};
        fn   = plots{i,5};

        f = figure('Color','w', 'Position',[100 100 900 480], 'Visible','off');
        try, f.Theme = 'light'; catch, end   % R2023a+ — forca tema claro

        ax = axes(f); hold(ax,'on');
        set(ax, ...
            'Color',     'w', ...
            'XColor',    'k', ...
            'YColor',    'k', ...
            'GridColor', cGrid_p, ...
            'GridAlpha', 0.3, ...
            'FontSize',  12, ...
            'LineWidth', 1.0, ...
            'Box',       'on');

        if ~isempty(ref) && any(~isnan(ref))
            plot(ax, t, ref, '--', 'Color', cRef_p, 'LineWidth', LW_p, ...
                 'DisplayName', 'ref');
        end
        plot(ax, t, sig, 'Color', cSig_p, 'LineWidth', LW_p, ...
             'DisplayName', ttl);

        grid(ax,'on');
        title(ax,  ttl, 'Interpreter','tex', 'FontSize',16, ...
              'Color','k', 'FontWeight','bold');
        ylabel(ax, ylbl,  'FontSize',13, 'Color','k');
        xlabel(ax, 't [s]', 'FontSize',13, 'Color','k');

        if ~isempty(ref) && any(~isnan(ref))
            leg = legend(ax, 'Location','best', 'Box','off', 'FontSize',12);
            set(leg, 'TextColor', 'k');
        end

        exportgraphics(f, fullfile(img_dir, [prefix '_' fn '.png']), ...
                       'BackgroundColor', 'white', 'Resolution', 150);
        close(f);

        fprintf('     [%2d/%d] %s.png\n', i, size(plots,1), fn);
    end

    set(0, 'DefaultFigureVisible', prev_vis);   % restaura visibilidade

    fprintf('Imagens salvas em: %s\n', img_dir);
    fprintf('  prefixo "%s"  (12 PNGs)\n', prefix);

    %% ---------- metricas (stepinfo + esforco de controle) ----------
    rep = {};
    rep{end+1} = sprintf('=== Metricas: %s ===', prefix);
    rep{end+1} = '';

    % Helper p/ janela final (ultimos 10% da sim) — usado pra regime
    tail_idx  = t > (t(end) - 0.1*(t(end) - t(1)));
    fmt_info  = @(s, info, yfinal, yinit, ss_val, unit) {
        sprintf('%s  (ref = %.3f %s, inicial = %.3f %s)', s, yfinal, unit, yinit, unit), ...
        sprintf('  Rise time     : %.3f s', info.RiseTime), ...
        sprintf('  Settling 2%%   : %.3f s', info.SettlingTime), ...
        sprintf('  Overshoot     : %.2f %%',  info.Overshoot), ...
        sprintf('  Peak          : %.3f %s  (t = %.2f s)', info.Peak, unit, info.PeakTime), ...
        sprintf('  Regime        : %.3f %s  (erro = %.3f %s)', ss_val, unit, abs(yfinal-ss_val), unit), ...
        '' };

    % --- altitude ---
    if exist('h_ref','var') && exist('he','var') && abs(h_ref - he) > 1e-6
        try
            info = stepinfo(h, t, h_ref, h(1));
            ss   = mean(h(tail_idx));
            rep  = [rep, fmt_info('Altitude (h)', info, h_ref, h(1), ss, 'm')];
        catch ME
            rep{end+1} = sprintf('Altitude: erro em stepinfo (%s)', ME.message);
            rep{end+1} = '';
        end
    end

    % --- velocidade ---
    if exist('VT_ref','var') && exist('Ve','var') && abs(VT_ref - Ve) > 1e-6
        try
            info = stepinfo(VT_ms, t, VT_ref, VT_ms(1));
            ss   = mean(VT_ms(tail_idx));
            rep  = [rep, fmt_info('Velocidade (V_T)', info, VT_ref, VT_ms(1), ss, 'm/s')];
        catch ME
            rep{end+1} = sprintf('Velocidade: erro em stepinfo (%s)', ME.message);
            rep{end+1} = '';
        end
    end

    % --- heading (step em t = psi_ref_t, precisa truncar tempo) ---
    if exist('psi_ref_final','var') && exist('psi_ref_t','var') && abs(psi_ref_final) > 1e-6
        try
            idx          = t >= psi_ref_t;
            t_local      = t(idx) - psi_ref_t;
            y_local      = psi_deg(idx);
            psi_tgt_deg  = psi_ref_final * R2D;
            info         = stepinfo(y_local, t_local, psi_tgt_deg, 0);
            ss           = mean(y_local(t_local > max(t_local)*0.9));
            rep          = [rep, fmt_info('Heading (psi)', info, psi_tgt_deg, 0, ss, 'deg')];
        catch ME
            rep{end+1} = sprintf('Heading: erro em stepinfo (%s)', ME.message);
            rep{end+1} = '';
        end
    end

    % --- pitch (rastreamento de theta_ref pela malha interna) ---
    if ~all(isnan(theta_ref_deg))
        [~, idx_pk] = max(abs(theta_ref_deg));
        rep{end+1} = 'Pitch (theta)  rastreando theta_ref gerado pela cascata';
        rep{end+1} = sprintf('  Pico theta_ref : %+.3f deg  (t = %.2f s)', ...
                              theta_ref_deg(idx_pk), t(idx_pk));
        rep{end+1} = sprintf('  theta no pico  : %+.3f deg', theta_deg(idx_pk));
        rep{end+1} = sprintf('  Regime theta_ref: %+.3f deg', mean(theta_ref_deg(tail_idx)));
        rep{end+1} = sprintf('  Regime theta   : %+.3f deg', mean(theta_deg(tail_idx)));
        rms_err    = sqrt(mean((theta_deg - theta_ref_deg).^2, 'omitnan'));
        rep{end+1} = sprintf('  RMS(theta-ref) : %.3f deg', rms_err);
        rep{end+1} = '';
    end

    % --- atuadores ---
    rep{end+1} = 'Atuadores:';
    rep{end+1} = sprintf('  Profundor (elev) pico abs: %.3f deg   regime: %+.3f deg', ...
                          max(abs(elev_deg)), mean(elev_deg(tail_idx)));
    rep{end+1} = sprintf('  Aileron   (ail)  pico abs: %.3f deg   regime: %+.3f deg', ...
                          max(abs(ail_deg)),  mean(ail_deg(tail_idx)));
    rep{end+1} = sprintf('  Rudder    (rud)  pico abs: %.3f deg   regime: %+.3f deg', ...
                          max(abs(rud_deg)),  mean(rud_deg(tail_idx)));
    rep{end+1} = sprintf('  Throttle         pico:     %.3f       regime:  %.3f', ...
                          max(throttle),     mean(throttle(tail_idx)));

    % imprime no console e salva
    txt = strjoin(rep, newline);
    fprintf('\n%s\n\n', txt);

    txt_file = fullfile(img_dir, [prefix '_metricas.txt']);
    fid = fopen(txt_file, 'w');
    if fid > 0
        fprintf(fid, '%s\n', txt);
        fclose(fid);
        fprintf('Metricas salvas em: %s\n', txt_file);
    end
end

%% ---------- local function ----------
function ylims = pad_ylim(y)
    y = y(~isnan(y));
    a = min(y); b = max(y);
    span = max(b-a, 1);
    ylims = [a-0.1*span, b+0.1*span];
end
