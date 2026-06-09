% trimagem_DH.m
% Trimagem em voo nivelado para o Drone Hibrido.
% Retorna estado de equilibrio Xe e comandos de equilibrio Ue.
% =============================================================

function [Xe,Ue,xe] = trimagem_DH(Ve,he,gammae)

x0 = [0*pi/180 0 0];      % x0 = [alphae dte dee]
f  = @(x) strflt2(x,Ve,he,gammae);
xe = fsolve(f,x0);

function f = strflt2(y,Ve,he,gammae)

ae  = y(1);
dte = y(2);
dee = y(3);

Xe = [Ve*cos(ae)   % u [m/s]
      0            % v [m/s]
      Ve*sin(ae)   % w [m/s]
      0            % p [rad/s]
      0            % q
      0            % r
      0            % phi [rad]
      ae + gammae  % theta
      0            % psi
      0            % xN
      0            % xE
      -he          % -xD
      0            % mi
      0];          % lambda

Ue = [dte   % manete throttle
      dee   % profundor
      0     % aileron
      0     % rudder
      0     % wind_x
      0     % wind_y
      0];   % wind_z

  Xpontoe = dyn_rigidbody_DH(0,Xe,Ue);

  f = [Xpontoe(1)
       Xpontoe(3)
       Xpontoe(5)];

end

end
