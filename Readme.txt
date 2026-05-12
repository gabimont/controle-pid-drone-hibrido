# README - Drone Hibrido (DH)

Modelo nao-linear + linearizacao do drone hibrido.
Requer MATLAB R2025b ou superior + Optimization Toolbox.

## Fluxo de uso

### 1) Build (uma vez ou quando mudar coef/trim)
       DH_build_model

   Faz trim (fsolve) e linearizacao para AS DUAS calibracoes
   aerodinamicas e grava:
     linear/sato/MATRIZES_DH.m
     linear/ana/MATRIZES_DH.m

   Recompile sempre que mudar:
     - coeficientes em coef_DH.m
     - Ve, he, gammae
     - parametros fisicos (massa, S, b, c, inercia)

### 2) Inicializar workspace (rapido, toda sessao)
       DH_inicializacao

   No topo do script, selecione qual modelo usar:
       coef_choice = 'sato';   % ou 'ana'

   O script carrega linear/<coef_choice>/MATRIZES_DH.m,
   configura ganhos do autopilot e referencias.
   Pronto para abrir/simular qualquer .slx.

### 3) Simular
       open modelo_linear_DH_CL          % closed-loop
       open Comparation_L_and_NL_Long    % validacao L vs NL
       open Comparation_L_and_NL_Lat     % validacao L vs NL

## Estrutura da pasta

DH_build_model.m       Trim + linearizacao para Sato e Ana
DH_inicializacao.m     Carrega variante escolhida + ganhos/refs
Readme.txt             Este arquivo

comum/                 Nucleo dinamico (compartilhado)
                       - dyn_rigidbody_DH.m, obs_rigidbody_DH.m
                       - modelo_DH.m, coef_DH.m
                       - aerodynamics.m, propulsion.m
                       - ISA.m, trimagem_DH.m

linear/                Utilitarios de linearizacao + modelos Simulink
                       - lin_DH.m, modelo_linear_DH.m
                       - modelo_linear_DH_CL.slx
                       - Comparation_L_and_NL_*.slx
   sato/MATRIZES_DH.m  Matrizes geradas com coef_Sato=1
   ana/MATRIZES_DH.m   Matrizes geradas com coef_Ana=1

nao_linear/            Planta NL para Simulink
                       - sfunction_DH.m, nonLinear.slx

## Configuracoes uteis em DH_inicializacao.m

- coef_choice ('sato' | 'ana'): qual modelo aerodinamico usar.

- Referencias (h_ref, VT_ref):  default = trim + offset.
  ex: h_ref  = he + 50  ->  step de 50m acima do trim
      VT_ref = Ve +  6  ->  step de  6 m/s acima do trim

- Ganhos PIDs (C_alt, C_theta, C_vel, C_phi):
  ja tunados via pidtune para o DH (longitudinal).
  Lateral ainda com chute do Piper (a tunar).

- SAS (Kq, Kp, Kr, K_heading): editaveis via workspace.
