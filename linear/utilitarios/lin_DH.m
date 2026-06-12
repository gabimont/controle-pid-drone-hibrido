% lin_DH.m
% Linearizacao numerica do modelo do Drone Hibrido em torno do trim
% (Xe, Ue). Retorna A, B, C, D do modelo completo.
%
% Passo de perturbacao: relativo com piso absoluto. Um passo
% proporcional ao valor de trim degenera para ~eps nos estados/
% entradas nulos no equilibrio (q, beta, aileron, ...), e a
% diferenca central vira ruido de arredondamento (coluna de q do
% A_long saia com fugoide espuriamente instavel).
% =============================================================

function [A,B,C,D] = lin_DH(Xe,Ue)

n = size(Xe,1);
m = size(Ue,1);

%% Matriz A
for j = 1:n
    dxj    = zeros(n,1);
    dxj(j) = 1e-5*max(abs(Xe(j)), 1);
    Xpup   = dyn_rigidbody_DH(0,Xe+dxj,[Ue; 0; 0; 0]);
    Xpdw   = dyn_rigidbody_DH(0,Xe-dxj,[Ue; 0; 0; 0]);
    A(:,j) = (Xpup-Xpdw)/(2*dxj(j));
end

%% Matriz B
for j = 1:m
    duj    = zeros(m,1);
    duj(j) = 1e-5*max(abs(Ue(j)), 1);
    Xpup   = dyn_rigidbody_DH(0,Xe,[Ue+duj; 0; 0; 0]);
    Xpdw   = dyn_rigidbody_DH(0,Xe,[Ue-duj; 0; 0; 0]);
    B(:,j) = (Xpup-Xpdw)/(2*duj(j));
end

%% Matriz C
for j = 1:n
    dxj    = zeros(n,1);
    dxj(j) = 1e-5*max(abs(Xe(j)), 1);
    Yup    = obs_rigidbody_DH(0,Xe+dxj,[Ue; 0; 0; 0]);
    Ydw    = obs_rigidbody_DH(0,Xe-dxj,[Ue; 0; 0; 0]);
    C(:,j) = (Yup-Ydw)/(2*dxj(j));
end

%% Matriz D
for j = 1:m
    duj    = zeros(m,1);
    duj(j) = 1e-5*max(abs(Ue(j)), 1);
    Yup    = obs_rigidbody_DH(0,Xe,[Ue+duj; 0; 0; 0]);
    Ydw    = obs_rigidbody_DH(0,Xe,[Ue-duj; 0; 0; 0]);
    D(:,j) = (Yup-Ydw)/(2*duj(j));
end

end
