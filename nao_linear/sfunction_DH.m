% sfunction_DH.m
% S-function (Level 1) que encapsula o modelo nao-linear completo do
% Drone Hibrido. Estados, derivadas e saidas vem de dyn_rigidbody_DH
% e obs_rigidbody_DH (utilitarios/).
% =============================================================

function [sys,x0,str,ts,simStateCompliance] = sfunction_DH(t,x,u,flag,Xe)

switch flag

  %% Inicializacao
  case 0
    [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe);

  %% Derivadas
  case 1
    sys = dyn_rigidbody_DH(t,x,u);

  %% Update / Terminate
  case {2,9}
    sys = [];

  %% Output
  case 3
    sys = obs_rigidbody_DH(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end


function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe)

sizes = simsizes;
sizes.NumContStates  = 14;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 18;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = Xe;
ts  = [0 0];
simStateCompliance = 'DefaultSimState';
