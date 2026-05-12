# Controle PID — Drone Híbrido (DH) modo asa-fixa

Modelo linear e não-linear do drone híbrido (DH) com piloto automático em cascata
PID para o modo de voo asa-fixa. Planta dinâmica baseada nos arquivos do Sato (ITA / EEC-D).

## Pré-requisitos

- MATLAB R2025b ou superior
- Optimization Toolbox (para `fsolve` no trim)
- Simulink

## Fluxo de uso

### 1. Build do modelo linear (uma vez, ou quando mudar coeficientes / trim)

```matlab
DH_build_model
```

Faz trim com `fsolve` e linearização para **as duas calibrações aerodinâmicas** (Sato e Ana) e grava as matrizes em:

- `linear/sato/MATRIZES_DH.m`
- `linear/ana/MATRIZES_DH.m`

Re-rodar sempre que mudar:
- coeficientes em `comum/coef_DH.m`
- velocidade/altitude/gama de trim (`Ve`, `he`, `gammae`)
- parâmetros físicos (massa, S, b, c, inércia)

### 2. Inicialização do workspace (toda sessão, rápido)

```matlab
DH_inicializacao
```

No topo do script, escolher a variante:

```matlab
coef_choice = 'sato';   % ou 'ana'
```

Carrega `linear/<coef_choice>/MATRIZES_DH.m`, configura ganhos do piloto automático e
referências. Pronto pra abrir/simular qualquer `.slx`.

### 3. Simular

```matlab
open linear/modelo_linear_DH_CL.slx        % closed-loop linear
open nao_linear/modelo_NL_DH_CL.slx        % closed-loop nao-linear (S-Function Sato)
open linear/Comparation_L_and_NL_Long.slx  % validacao linear vs nao-linear
open linear/Comparation_L_and_NL_Lat.slx   % validacao linear vs nao-linear
```

## Estrutura

```
.
├── DH_build_model.m       Trim + linearizacao para Sato e Ana
├── DH_inicializacao.m     Carrega variante escolhida + ganhos PID + referencias
├── README.md              Este arquivo
│
├── comum/                 Nucleo dinamico Sato (compartilhado)
│   ├── sfunction_DH.m         S-Function da planta NL (level-1)
│   ├── dyn_rigidbody_DH.m     Derivadas (flag=1)
│   ├── obs_rigidbody_DH.m     Saidas (flag=3)
│   ├── modelo_DH.m            Forcas e momentos
│   ├── coef_DH.m              Coeficientes aerodinamicos
│   ├── aerodynamics.m
│   ├── propulsion.m
│   ├── ISA.m                  Atmosfera padrao
│   └── trimagem_DH.m          Trim via fsolve
│
├── linear/                Linearizacao + modelos Simulink
│   ├── lin_DH.m
│   ├── modelo_linear_DH.m
│   ├── modelo_linear_DH_CL.slx          Closed-loop linear
│   ├── Comparation_L_and_NL_Long.slx    Validacao L vs NL
│   ├── Comparation_L_and_NL_Lat.slx     Validacao L vs NL
│   ├── sato/MATRIZES_DH.m               Matrizes (gerado por DH_build_model)
│   └── ana/MATRIZES_DH.m
│
└── nao_linear/            Modelo NL closed-loop
    ├── sfunction_DH.m
    ├── nonLinear.slx                    Planta NL standalone
    └── modelo_NL_DH_CL.slx              Closed-loop NL com PID
```

## Arquitetura do controle (cascata PID)

### Longitudinal

```
                    Altitude Hold (PID lento)
h_ref ───►(+)───► PID ───► theta_ref ───┐
[h] ──────┘                              │
                                         ▼
                Pitch Attitude Hold (PID rapido)
theta_ref ───►(+)───► PID ───► delta_e
[theta] ─────┘

                Velocity Hold (PIDF)
VT_ref ───►(+)───► PID ───► delta_T
[VT] ─────┘
```

### Latero-direcional

```
            Heading Hold (P)
psi_ref ───►(+)───► K_psi ───► phi_ref ───┐
[psi] ─────┘                                │
                                            ▼
            Bank Angle Hold (PID, ganhos negativos)
phi_ref ───►(+)───► PID ───► delta_a
[phi] ─────┘
```

Rudder: washout filter `s/(s+1)` em `r` (yaw damper).

## Ganhos (variante SATO, Ve=12 m/s, he=600 m)

Tunados via `pidtune`:

| Malha | Funcao | Tipo | P | I | D | N |
|---|---|---|---|---|---|---|
| Altitude Hold | h_ref → theta_ref | PI | 0.0220 | 0.0085 | — | 20 |
| Pitch Attitude | theta_ref → delta_e | PIDF | 0.2731 | 0.3821 | 0.0406 | 229 |
| Velocity Hold | VT_ref → delta_T | PIDF | 0.1770 | 0.0840 | -0.002 | 114 |
| Bank Angle | phi_ref → delta_a | PIDF | -0.8300 | -0.1712 | 0.3230 | 2 |
| Heading | psi_ref → phi_ref | P | 0.1712 | — | — | — |

Ganhos do roll com sinal negativo porque `Cl_da < 0` (logo `B_lat(p, aileron) < 0`).

## Margens de robustez (open-loop)

| Loop | GM | PM | BW (cross) |
|---|---|---|---|
| Pitch interno | ∞ | 69° | 2.0 rad/s |
| Altitude (externo) | 38 dB | 65° | 0.30 rad/s |
| Velocidade | 51 dB | 87° | 1.0 rad/s |
| Roll interno | ∞ | 80° | 2.0 rad/s |
| Heading (externo) | 7.1 dB | 75° | 0.40 rad/s |

Todas as malhas com PM > 45° e GM > 6 dB. Separação de banda inner/outer ≈ 5–7×.

## Modos dinâmicos da planta (variante SATO, Ve=12 m/s)

### Longitudinal (5 estados: u, alpha, q, theta, h)

| Modo | Autovalores | wn (rad/s) | Amortecimento |
|---|---|---|---|
| Curto-período | -11.41 ± 5.59j | 12.7 | 0.90 |
| Fugóide | -0.088 ± 0.689j | 0.69 | 0.13 |
| Altitude | -0.0001 | — | — |

### Latero-direcional (5 estados: beta, p, r, phi, psi)

| Modo | Autovalores | wn (rad/s) | Amortecimento |
|---|---|---|---|
| Roll | -4.70 | 4.70 | 1.0 |
| Dutch roll | -3.08 ± 2.10j | 3.72 | 0.83 |
| Spiral | -0.18 | — | — |
| Heading | 0 | — | — |

## Topologia de comandos no modelo NL

A `S-Function` espera o vetor de entrada na ordem:

```
U = [throttle, elevator, aileron, rudder, wind_x, wind_y, wind_z]
     U(1)     U(2)      U(3)     U(4)    U(5..7)
```

No `modelo_NL_DH_CL.slx`, o piloto automático produz **delta** (perturbação em torno do trim).
A soma com o trim absoluto acontece **antes** dos saturadores:

```
PID_out (delta) ───►(+)───► Sat ───► canal absoluto da S-Function
                     ▲
                  TrimInput(k) = Ue(k)
```

Saturadores (limites físicos):
- Throttle: `[0, 1]`
- Elevator/Aileron/Rudder: `[-25°, +25°] = [-0.4363, +0.4363] rad`

## Configurações úteis em `DH_inicializacao.m`

- `coef_choice` (`'sato'` ou `'ana'`) — qual modelo aerodinâmico usar
- Referências (default = trim, sem step):
  - `h_ref = he` — para excitar: `h_ref = he + 50`
  - `VT_ref = Ve` — para excitar: `VT_ref = Ve + 6`
  - `psi_ref_final = 0` — para excitar: `psi_ref_final = deg2rad(20)`
- Ganhos PID (`C_alt`, `C_theta`, `C_vel`, `C_phi`) — já tunados via `pidtune`
- SAS dampers (`Kq`, `Kp`, `Kr`) — editáveis no workspace
- `K_heading` — ganho da malha de heading

## Validação

Os modelos linear e não-linear apresentam respostas próximas para perturbações pequenas
em torno do trim (RMS de 0.86° em theta e 0.74 m em altitude na simulação longitudinal de 10 s).

Para perturbações grandes, divergem como esperado (não-linearidades aerodinâmicas e
acoplamentos cinemáticos).

## Referências

- STEVENS, B. L.; LEWIS, F. L. *Aircraft Control and Simulation*, 3rd ed. Wiley, 2016.
- SANTOS, M. *Modelagem e Controle de Aeronaves*, ITA, 2018.
- SATO, F. C. Y. C. *Modelo matemático completo não-linear DH*, ITA / EEC-D.
