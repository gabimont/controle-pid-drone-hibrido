% validar_atuadores.m
% Varre os 4 atuadores (throttle, profundor, aileron, leme) em varias
% manobras e tabula taxa de pico, deflexao e se atinge limite de taxa/posicao.
run(fullfile(fileparts(mfilename('fullpath')),'DH_inicializacao.m'));
R2D = 180/pi;

% limites
rate_lim = [eng.rate, act.rate, act.rate, act.rate];        % [thr de da dr]  (thr em 1/s; resto rad/s)
pos_lim  = [1, 0.4363, 0.4363, 0.4363];                     % curso (thr [0,1]; resto +-25 deg)
nomes    = {'Throttle','Profundor','Aileron','Leme'};
unid     = {'1/s','deg/s','deg/s','deg/s'};

% manobras (cada uma reseta as excitacoes e define a sua)
M = {};
M{end+1} = struct('nome','Missao perfil',        'St',280, 'VTd',3.2,'psi',5,'psiDbl',[100 150],'h',5,'hDbl',[160 240],'phi',0,'Kh',K_heading,'hT',80,'psiT',50,'phiT',5,'VTt',5);
M{end+1} = struct('nome','Altitude +25 m',       'St',80,  'VTd',0,  'psi',0,'psiDbl',[1e9 1e9],'h',25,'hDbl',[1e9 1e9],'phi',0,'Kh',K_heading,'hT',10,'psiT',5,'phiT',5,'VTt',5);
M{end+1} = struct('nome','PA roll phi+20',       'St',40,  'VTd',0,  'psi',0,'psiDbl',[1e9 1e9],'h',0, 'hDbl',[1e9 1e9],'phi',20,'Kh',0,        'hT',10,'psiT',5,'phiT',5,'VTt',5);
M{end+1} = struct('nome','Heading psi+40',       'St',60,  'VTd',0,  'psi',40,'psiDbl',[1e9 1e9],'h',0,'hDbl',[1e9 1e9],'phi',0,'Kh',K_heading,'hT',10,'psiT',5,'phiT',5,'VTt',5);
M{end+1} = struct('nome','Agressiva h+30 psi+40','St',80,  'VTd',0,  'psi',40,'psiDbl',[1e9 1e9],'h',30,'hDbl',[1e9 1e9],'phi',0,'Kh',K_heading,'hT',5, 'psiT',5,'phiT',5,'VTt',5);

fprintf('\n================ VARREDURA DE ATUADORES ================\n');
for k=1:numel(M)
  c = M{k};
  % reset + set excitacoes
  VT_ref=Ve; VT_step_delta=c.VTd; VT_step_t=c.VTt;
  h_ref=he;  h_step_final=c.h;  h_step_t=c.hT;  h_step_t2=c.hDbl(1);  h_step_t3=c.hDbl(2);
  psi_ref_init=0; psi_ref_final=deg2rad(c.psi); psi_ref_t=c.psiT; psi_ref_t2=c.psiDbl(1); psi_ref_t3=c.psiDbl(2);
  phi_step_init=0; phi_step_final=deg2rad(c.phi); phi_step_t=c.phiT; phi_step_t2=1e9; phi_step_t3=1e9;
  theta_step_init=0; theta_step_final=0; theta_step_t2=1e9; theta_step_t3=1e9;
  K_heading=c.Kh; att_alt=0;

  o = sim('modelo_NL_DH_CL','StopTime',num2str(c.St));
  t=o.tout; U=o.U.signals.values; dt=diff(t);
  fprintf('\n--- %s (t=%ds) ---\n', c.nome, c.St);
  fprintf('%-10s | taxa pico (%%lim) | deflexao [min,max] (%%lim)\n','superf');
  for s=1:4
    u = U(:,s); if s>1, u=u*R2D; end          % profundor/aileron/leme em deg; throttle [-]
    rate = max(abs(diff(u)./dt));
    rl = rate_lim(s); if s>1, rl=rl*R2D; end   % limite em deg/s p/ superficies
    pl = pos_lim(s);  if s>1, pl=pl*R2D; end   % limite de posicao em deg
    if s==1, dmin=min(u); dmax=max(u); prng=100*max(abs([dmin dmax-1]))/1; else
             dmin=min(u); dmax=max(u); prng=100*max(abs([dmin dmax]))/pl; end
    fprintf('%-10s | %7.1f (%4.0f%%)  | [%6.2f, %6.2f] (%3.0f%%)\n', ...
            nomes{s}, rate, 100*rate/rl, dmin, dmax, prng);
  end
end
fprintf('\n(limites: throttle 1.0/s e [0,1]; superficies 150 deg/s e +-25 deg)\n');
