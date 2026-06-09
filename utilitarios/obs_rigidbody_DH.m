% Autor: SATO, F. C. Y. CURSINO
% ITA - PG/EEC-D
% MODELO MATEM�TICO COMPLETO DH
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/03/2026 

function Y = obs_rigidbody_DH(t,X,U)

%DECLARA��O DE VARIAVEIS: ESTADOS, CONTROLES E PERTURBA��ES

u     = X(1);
v     = X(2);
w     = X(3);
p     = X(4);
q     = X(5); 
r     = X(6);
phi   = X(7);
theta = X(8);
psi   = X(9);
xN    = X(10);
xE    = X(11);
xD    = X(12);
mi    = X(13);
lambda= X(14);

%Lbn Matrix

Lbn = [cos(theta)*cos(psi)                            cos(theta)*sin(psi)                            -sin(theta)
       sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta)
       cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];

Lnb = Lbn';

%PERTURBA��ES DO VENTO

wD = U(7);
U(5:7) = Lbn*U(5:7);
wx = U(5);
wy = U(6);
wz = U(7);

%PAR�METROS DA AERONAVE
%%
VT0 = 12; %VELOCIDADE DE REFER�NCIA [m/s]UTILIZADA NAS SIMULA��ES EM AVL PARA OBTEN��O DAS DERIVADAS DE ESTABILIDADE (VETOR THETA)
S   = 0.27; %�REA DA ASA [m^2]
m   = 2.2;%MASSA TOTAL DA AERONAVE (CONSIDERAR A CARGA �TIL) [kg]
c   = 0.226 ; %CORDA M�DIA AERODIN�MICA [m]
b   = 1.2; %ENVERGADURA DA ASA [m] 
g   = 9.80665;%ACELERA��O DA GRAVIDADE [m/s^2]
pi = 3.14159265;%N�MERO Pi
e =0.8;
AR = (b^2)/S;%ALONGAMENTO DA ASA

 
%%Matriz de inercia Piper_V11

    Ti  = [0.14410 0  -0.00167
         0 0.11550  0
        -0.00167 0 0.25716];
    
  Iy = 0.11550;

% COMANDOS PARA SUPERF�CIES DE CONTROLE DA AERONAVE

dt    = U(1);%manete trotle
de    = U(2);%elevator
da    = U(3);%aileron
dr    = U(4);%leme

% DADOS ATMOSF�RICOS, VELOCIDADE REAL E �NGULOS DE INCID�NCIA

VT    = sqrt((u-wx)^2 + (v-wy)^2 + (w-wz)^2 ); %Aerodynamic velocity
alpha = atan((w-wz)/(u-wx));
beta  = asin((v-wy)/VT);

T    = 288.15*(1-6.5e-3*(-xD)/288.15);
rho  = 1013.25e2*(1-6.5e-3*(-xD)/288.15)^(5.2561)/(287.3*T);


%%DEFINI��O DO VETOR DE DERIVADAS DE ESTABILIDADE OBTIDO NO AVL(THETA - CONFORME
%%PROGRAMA DE IDENTIFICA�AO OEM)

coef_DH;%UTILIZAR O ARQUIVO COEF QUE REPRESENTA O MODELO AERODIN�MICO COMPLETO DA AERONAVE DH

%%

qbar = 0.5*rho*VT^2;% PRESS�O DIN�MICA

Cm   = Cm0 + Cmalpha*alpha + Cmde*de + Cmq*q*c/(2*VT0);% COEFICIENTE DE MOMENTO DE ARFAGEM
qp = qbar*S*Cm/Iy;%ACELERA��O ANGULAR DE ARFAGEM [rad/s]


%Gamma - RELA��O ENTRE O �NGULO DE ATAQUE E O �NGULO DE ARFAGEM

xNpxEpxDp = Lnb*[u v w]';
xDp       = xNpxEpxDp(3);
gamma     = asin((-xDp-wD)/VT);

% C�LCULO DE FOR�AS E MOMENTOS

[Ti,Fext,Mext,m,axayaz] = modelo_DH(qbar,VT,alpha,beta,X,U,rho);


% SA�DAS OBSERVADAS

Y =  [VT
      alpha
      beta
      gamma
      p
      q
      r
      phi
      theta
      psi
      axayaz
      xN
      -xD
      mi
      lambda
      qp];
  

end

