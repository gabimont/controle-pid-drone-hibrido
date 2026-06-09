%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                  %
%                 Project eVTOL                    %
% Author: Huascar Mirko Montecinos Cortez          %
% Data init: 23/April/2026                         %
% Data end:  --/----/202-                          %
% Technological Institute of Aeronautics - ITA     %
% Electronic Devices and Systems (EEC-D)           %
%                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Technological Institute of Aeronautics
% Electronic Devices and Systems (EEC-D)
% Copyright 2026 Regents of the Technological Institute of Aeronautics.
% All rights reserved.

sys_DH = ss(A,B,C,D, ...
    'StateName', {'1 u'; '2 v'; '3 w'; '4 p'; '5 q'; '6 r'; '7 phi'; '8 theta'; '9 psi'; '10 xN'; '11 xE'; '12 xD'; '13 mi'; '14 lambda'}, ...
    'InputName', {'1 throttle'; '2 elevator'; '3 aileron'; '4 rudder'; '5 wind_x'; '6 wind_y'; '7 wind_z'}, ...
    'OutputName', { '1 VT'; '2 alpha'; '3 beta'; '4 gamma'; '5 p'; '6 q'; '7 r'; '8 phi'; '9 theta'; '10 psi'; '11 ax'; '12 ay'; ...
    '13 az'; '14 xN'; '15 h'; '16 mi'; '17 lambda'; '18 qdot'})

%% ---------- MODO LONGITUDINAL ----------
% x_long = [u alpha q theta h]'
% u_long = delta_e

idx_long = [1 3 5 8 12];   % [u w q theta xD]
in_long  = [1 2];          % elevator

A_long_native = A(idx_long, idx_long);
B_long_native = B(idx_long, in_long);

u0 = Xe(1);
w0 = Xe(3);

T_long = [ ...
    1,                    0,                  0, 0,  0;
    -w0/(u0^2+w0^2),   u0/(u0^2+w0^2),        0, 0,  0;
    0,                    0,                  1, 0,  0;
    0,                    0,                  0, 1,  0;
    0,                    0,                  0, 0, -1];

A_long = T_long * A_long_native / T_long;
B_long = T_long * B_long_native;

% Saidas: 5 estados + VT linearizado em torno do trim:
%   dVT = cos(alpha_e)*du + sin(alpha_e)*dw
%       = (VT_e/u_e)*du + (VT_e*w_e/u_e)*dalpha
% VT (nao u) e' a realimentacao correta da malha de velocidade.
VT_e   = sqrt(u0^2 + w0^2);
C_long = [eye(5); VT_e/u0, VT_e*w0/u0, 0, 0, 0];
D_long = zeros(6,2);

sys_long = ss(A_long, B_long, C_long, D_long, ...
    'StateName', {'u'; 'alpha'; 'q'; 'theta'; 'h'}, ...
    'InputName', {'throttle'; 'elevator'}, ...
    'OutputName', {'u'; 'alpha'; 'q'; 'theta'; 'h'; 'VT'})

%% ---------- MODO LATERO-DIRECCIONAL ----------
idx_lat = [2 4 6 7 9];   % [v p r phi psi]
in_lat  = [3 4];         % [aileron rudder]

A_lat_native = A(idx_lat, idx_lat);
B_lat_native = B(idx_lat, in_lat);

VT0 = sqrt(Xe(1)^2 + Xe(3)^2);

T_lat = [ ...
    1/VT0, 0, 0, 0, 0;
    0,     1, 0, 0, 0;
    0,     0, 1, 0, 0;
    0,     0, 0, 1, 0;
    0,     0, 0, 0, 1];

A_lat = T_lat * A_lat_native / T_lat;
B_lat = T_lat * B_lat_native;

C_lat = eye(5);
D_lat = zeros(5,2);

sys_lat = ss(A_lat, B_lat, C_lat, D_lat, ...
    'StateName', {'beta'; 'p'; 'r'; 'phi'; 'psi'}, ...
    'InputName', {'aileron'; 'rudder'}, ...
    'OutputName', {'beta'; 'p'; 'r'; 'phi'; 'psi'})

%% Comparacao do modelo longitudinal e latero-direcional
if abrir_modelo_simulink == 1
    % Longitudinal
    open('Comparation_L_and_NL_Long.slx')
    sim('Comparation_L_and_NL_Long.slx')
    % latero-direcional
    open('Comparation_L_and_NL_Lat.slx')
    sim('Comparation_L_and_NL_Lat.slx')
end

if plot_figure == 1
    out_long = sim('Comparation_L_and_NL_Long.slx');
    out_lat = sim('Comparation_L_and_NL_Lat.slx');

    figure;
    sgtitle('PLanta Longitudinal')
    subplot(3,2,1)
    plot(out_long.Vs.time, out_long.Vs.signals.values(:,1),'--')
    hold on
    plot(out_long.Vs.time, out_long.Vs.signals.values(:,2))
    hold off
    grid on
    axis([0 10 0 20])
    title('Vs')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('m/s')
    
    subplot(3,2,2)
    plot(out_long.alpha.time, out_long.alpha.signals.values(:,1),'--')
    hold on
    plot(out_long.alpha.time, out_long.alpha.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -2 2])
    title('Alpha')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg')

    subplot(3,2,3)
    plot(out_long.q.time, out_long.q.signals.values(:,1),'--')
    hold on
    plot(out_long.q.time, out_long.q.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -5 5])
    title('q')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg/s')

    subplot(3,2,4)
    plot(out_long.theta.time, out_long.theta.signals.values(:,1),'--')
    hold on
    plot(out_long.theta.time, out_long.theta.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -5 5])
    title('Theta')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg')

    subplot(3,2,[5,6]);
    plot(out_long.xD.time, out_long.xD.signals.values(:,1),'--')
    hold on
    plot(out_long.xD.time, out_long.xD.signals.values(:,2))
    hold off
    grid on
    axis([0 10 590 610])
    title('-xD')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('m')    

    figure;
    sgtitle('PLanta latero-Direcional')
    subplot(3,2,1)
    plot(out_lat.beta.time, out_lat.beta.signals.values(:,1),'--')
    hold on
    plot(out_lat.beta.time, out_lat.beta.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -10 10])
    title('Beta')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg')    

    subplot(3,2,2)
    plot(out_lat.p.time, out_lat.p.signals.values(:,1),'--')
    hold on
    plot(out_lat.p.time, out_lat.p.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -10 10])
    title('p')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg/s')

    subplot(3,2,3)
    plot(out_lat.phi.time, out_lat.phi.signals.values(:,1),'--')
    hold on
    plot(out_lat.phi.time, out_lat.phi.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -5 5])
    title('phi')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg')    

    subplot(3,2,4)
    plot(out_lat.psi.time, out_lat.psi.signals.values(:,1),'--')
    hold on
    plot(out_lat.psi.time, out_lat.psi.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -20 20])
    title('psi')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg')    

    subplot(3,2,[5,6]);
    plot(out_lat.r.time, out_lat.r.signals.values(:,1),'--')
    hold on
    plot(out_lat.r.time, out_lat.r.signals.values(:,2))
    hold off
    grid on
    axis([0 10 -5 5])
    title('r')
    legend('Nao Linear','Linear')
    xlabel('Tempo [s]')
    ylabel('deg/s')    

end
