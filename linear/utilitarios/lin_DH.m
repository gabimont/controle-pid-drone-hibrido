% lin_DH.m
% Linearizacao numerica do modelo do Drone Hibrido em torno do trim
% (Xe, Ue). Retorna A, B, C, D do modelo completo.
% =============================================================

function [A,B,C,D] = lin_DH(Xe,Ue)

n = size(Xe,1);
m = size(Ue,1);

%% Matriz A
for j = 1:n
    dxj    = zeros(n,1);
    dxj(j) = Xe(j)*0.025 + eps;
    Xpup   = dyn_rigidbody_DH(0,Xe+dxj,[Ue; 0; 0; 0]);
    Xpdw   = dyn_rigidbody_DH(0,Xe-dxj,[Ue; 0; 0; 0]);
    A(:,j) = (Xpup-Xpdw)/(2*dxj(j));
end

%% Matriz B
for j = 1:m
    duj    = zeros(m,1);
    duj(j) = Ue(j)*0.025 + eps;
    Xpup   = dyn_rigidbody_DH(0,Xe,[Ue+duj; 0; 0; 0]);
    Xpdw   = dyn_rigidbody_DH(0,Xe,[Ue-duj; 0; 0; 0]);
    B(:,j) = (Xpup-Xpdw)/(2*duj(j));
end

%% Matriz C
for j = 1:n
    dxj    = zeros(n,1);
    dxj(j) = Xe(j)*0.025 + eps;
    Yup    = obs_rigidbody_DH(0,Xe+dxj,[Ue; 0; 0; 0]);
    Ydw    = obs_rigidbody_DH(0,Xe-dxj,[Ue; 0; 0; 0]);
    C(:,j) = (Yup-Ydw)/(2*dxj(j));
end

%% Matriz D
for j = 1:m
    duj    = zeros(m,1);
    duj(j) = Ue(j)*0.025 + eps;
    Yup    = obs_rigidbody_DH(0,Xe,[Ue+duj; 0; 0; 0]);
    Ydw    = obs_rigidbody_DH(0,Xe,[Ue-duj; 0; 0; 0]);
    D(:,j) = (Yup-Ydw)/(2*duj(j));
end

end
