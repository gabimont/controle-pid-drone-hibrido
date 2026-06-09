% coef_DH.m
% =============================================================
% MODELO AERODINAMICO OFICIAL (e UNICO) do Drone Hibrido (DH).
% Este arquivo e' a unica fonte dos coeficientes: alimenta a
% planta nao-linear (modelo_DH.m / obs_rigidbody_DH.m) e a
% linearizacao (DH_build_model.m -> linear/MATRIZES_DH.m).
% Nao manter copias/variantes em outros arquivos.
%
% Modelo identificado por Ana Carolina de Lima Angelo (24/04/2026):
%   - Longitudinal: Modelo 4 (Janela 379000-380000 ms)
%   - Latero-direcional: Modelo 2 (com vetor longitudinal do Modelo 4)
% Convencao de sinal do profundor: ORIGINAL (nao invertido).
% =============================================================

%% Longitudinal
Cl0     =  1.93615e-01;
Cd0     =  1.00383e-01;
Clalpha =  2.87836e+00;
Cmalpha = -8.81949e-01;
Clq     =  3.11023e+01;
Cm0     =  8.30091e-02;
Cmq     = -9.92909e+00;
Cmde    =  1.04662e+00;

%% Latero-direcional
Cyb  = -3.98966e-01;
Clb  = -2.77332e-01;
Cnb  =  1.53474e-01;
Cyp  = -6.93329e-01;
Clp  = -1.32871e+00;
Cnp  = -1.08353e-01;
Cyr  =  3.66157e-02;
Clr  =  1.00433e+00;
Cnr  = -4.61062e-01;
Clda = -7.10799e-01;
Cnda =  3.95561e-02;
Cyda =  0;
Cydr =  4.65883e-02;
Cndr =  1.82983e-01;
Cldr = -6.90524e-02;
