% MODELO MATEMÁTICO COMPLETO NÃO-LINEAR DH
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/03/2026
% SATO F C Y C, ITA 2026

function [A,B,C,D] = lin_DH(Xe,Ue,coef_Sato,coef_Ana)

n = size(Xe,1);
m = size(Ue,1);

%%
%Matriz A

for j = 1:n
    dxj    = zeros(n,1);
    dxj(j) = Xe(j)*0.025 + eps;
    Xpup   = dyn_rigidbody_DH(0,Xe+dxj,[Ue; 0; 0; 0],coef_Sato,coef_Ana);
    Xpdw   = dyn_rigidbody_DH(0,Xe-dxj,[Ue; 0; 0; 0],coef_Sato,coef_Ana);
    A(:,j) = (Xpup-Xpdw)/(2*dxj(j));

end

%Matriz B

for j = 1:m
    duj    = zeros(m,1);
    duj(j) = Ue(j)*0.025 + eps;
    Xpup   = dyn_rigidbody_DH(0,Xe,[Ue+duj; 0; 0; 0],coef_Sato,coef_Ana);
    Xpdw   = dyn_rigidbody_DH(0,Xe,[Ue-duj; 0; 0; 0],coef_Sato,coef_Ana);
    B(:,j) = (Xpup-Xpdw)/(2*duj(j));

end

%Matriz C

for j = 1:n
    dxj    = zeros(n,1);
    dxj(j) = Xe(j)*0.025 + eps;
    Yup   = obs_rigidbody_DH(0,Xe+dxj,[Ue; 0; 0; 0],coef_Sato,coef_Ana);
    Ydw   = obs_rigidbody_DH(0,Xe-dxj,[Ue; 0; 0; 0],coef_Sato,coef_Ana);
    C(:,j) = (Yup-Ydw)/(2*dxj(j));

end

%Matriz D

for j = 1:m
    duj    = zeros(m,1);
    duj(j) = Ue(j)*0.025 + eps;
    Yup   = obs_rigidbody_DH(0,Xe,[Ue+duj; 0; 0; 0],coef_Sato,coef_Ana);
    Ydw   = obs_rigidbody_DH(0,Xe,[Ue-duj; 0; 0; 0],coef_Sato,coef_Ana);
    D(:,j) = (Yup-Ydw)/(2*duj(j));

end

end