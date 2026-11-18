%COSTANTI ENCELADO-SATURNO
mu=7.21e9;                         %parametro gravitazionale di Encelado [m^3/s^2]
muS=3.793e16;                      %parametro gravitazionale di Saturno [m^3/s^2]
a=2.380e8;                         %semiasse dell'orbita di Encelado [m]
n=sqrt(muS/a^3);                   %moto medio [rad/s]
R=252.1e3;                         %raggio di Encelado [m]
g0=9.80665;                        %[m/s^2]
kC=1; kw=1; kS=1;                  %interruttori: Coriolis, centrifuga, marea

%LANDER
m_in=1000;                         %massa iniziale [kg]
Isp=300;                           %motore principale [s]
Tmax=700;                          %spinta massima del motore principale [N]
b0=[0;0;1];                        %asse di spinta in assi corpo
lx=0.98; ly=1.07; lz=1.15;         %lati del parallelepipedo [m]
Ixx=m_in/12*(ly^2+lz^2);
Iyy=m_in/12*(lx^2+lz^2);
Izz=m_in/12*(lx^2+ly^2);
J=diag([Ixx Iyy Izz]);
Jmax=max([Ixx Iyy Izz]);

%CONDIZIONI INIZIALI
uf=[1;0;0];                        %verticale nel punto di partenza
h0=2000;                           %quota iniziale [m]
p_in=(R+h0)*uf;
dp_in=[-5;40;0];
q_in=[1 0 0 0]';
w_in=[0 0 0]';

%GUIDA D'SOUZA (tempo finale libero)
Dy=8000;                           %distanza del sito lungo y [m]
phi=Dy/R;                          %angolo al centro di Encelado
u_f=[cos(phi);sin(phi);0];         %verticale nel sito di atterraggio
pf=(R-0.5)*u_f;                    %target mezzo metro sotto la superficie
vf=[0;0;0];                        %velocita' finale nulla
tgo_min=10;                        %limite inferiore di tgo [s]
Gam=0.005;                         %peso del tempo

%PROPULSORI DI ASSETTO (RCS)
Frcs=22;                           %spinta del singolo propulsore [N]
Isp_rcs=290;                       %[s]
L=0.6;                             %meta' lato della base [m]
hth=0.5;                           %quota dei grappoli sopra il centro [m]
r_off=[0;0;0];                     %posizione del CdM rispetto al centro geometrico [m]
taumax_rcs=2*Frcs*L;                  %coppia massima per asse [N m] (26.4)

%PWPF
Km=5;                              %guadagno del filtro
Tm=0.4;                            %costante di tempo del filtro [s]
Uon=0.2;                           %soglia di accensione
Uoff=0.05;                         %soglia di spegnimento
u_dz=Uon/Km;                       %zona morta (frazione di tau_rcs)
Dtmin=-Tm*log(1-(Uon-Uoff)/Km);    %impulso minimo [s]

%GUADAGNI PD (dimensionati sulla coppia dei propulsori)
zeta=0.7;
dqmax=sin(pi/4);
wn=sqrt(tau_rcs/(2*Jmax*dqmax));   %~0.301 rad/s
kp=2*J*wn^2;                       %all'inizio tutta la coppia viene da kp (errore massimo, w=0)
kd=2*zeta*wn*J;
w_des=[0 0 0]';                    %usato solo se il feedforward e' scollegato

%out=sim("encelado.slx");

%REACTION WHEELS (non piu' usate, sostituite dall'RCS)
% theta=pi;
% t_man=60;
% ddtheta=4*theta/t_man^2;
% taumax_rw=Jmax*ddtheta;            %coppia massima della ruota
% d=0.05;
% T=700;
% t_desc=400;
% h_dist=d*T*t_desc;
% hmax_rw=50;
% h_rw_in=[0;0;0;0];
% %Configurazione piramidale
% beta=54.74*pi/180;
% cb=cos(beta); sb=sin(beta);
% Z=[cb 0 -cb 0;
%    0 cb 0 -cb;
%    sb sb sb sb];
% Zpinv=Z'/(Z*Z');
% %Guadagni PD dimensionati sulle ruote
% wn=sqrt(taumax_rw/(2*Jmax*dqmax));
% kp=2*J*wn^2;
% kd=2*zeta*wn*J;

%KALMAN FILTER
dt_imu=0.02;
sigma_a=5e-4; %sigma_a=0;
sigma_ba=1e-6; %sigma_ba=0;
ba0=1e-3*randn(3,1); %ba0=1e-3*[1;0;0];%per test A ba0=zeros(3,1);

sigma_p0=100;            %[m]
sigma_v0=0.1;            %[m/s]
sigma_b0=1e-3;           %[m/s^2], coerente con ba0=1e-3*randn(3,1)
P0=diag([sigma_p0^2*ones(1,3),sigma_v0^2*ones(1,3),sigma_b0^2*ones(1,3)]);
x0_hat=[p_in+sigma_p0*randn(3,1);dp_in+sigma_v0*randn(3,1);zeros(3,1)];

%Altimetro
dt_alt=0.1;
sigma_h0=0.2;
k_h=0.005;
N_alt=round(dt_alt/dt_imu);

%Camera
dt_cam=1;
sigma_c0=0.5;          %sigma minima [m]
k_c=0.01;              %crescita con la quota
N_cam=round(dt_cam/dt_imu);   %=50

%MEKF ASSETTO
%Giroscopio
sigma_v=1e-5;                  %[rad/sqrt(s)]  rumore (ARW)
sigma_u=1e-8;                  %[rad/s^1.5]    random walk del bias
bg0=deg2rad(10/3600)*randn(3,1); %bias iniziale ~10 deg/h

%Star tracker
dt_st=0.2;                     %5 Hz
sigma_st=5e-5;                 %[rad] ~10 arcsec
N_st=round(dt_st/dt_imu);      %=10

%Attitude Kalman Filter
%Stima iniziale filtro d'assetto
sigma_th0=deg2rad(0.5);               %errore iniziale d'assetto [rad]
sigma_bg0=deg2rad(10/3600);           %incertezza sul bias [rad/s]
dth0=sigma_th0*randn(3,1);            %errore iniziale estratto
dq0=[1;dth0/2]; dq0=dq0/norm(dq0);
qn=q_in(:)/norm(q_in);
q_hat0=[dq0(1)*qn(1)-dq0(2:4)'*qn(2:4);
        dq0(1)*qn(2:4)+qn(1)*dq0(2:4)-cross(dq0(2:4),qn(2:4))]; %q_hat0=dq0 x q_in
xa0=[q_hat0;zeros(3,1)];              %IC del Unit Delay xa
Pa0=blkdiag(sigma_th0^2*eye(3),sigma_bg0^2*eye(3)); %IC del Unit Delay Pa


%========== DATI ==========
P=squeeze(out.p_log); if size(P,1)==3, P=P'; end    %posizione vera (N x 3)
V=squeeze(out.dp_log); if size(V,1)==3, V=V'; end   %velocita' vera (N x 3)
N=min(size(P,1),size(V,1)); P=P(1:N,:); V=V(1:N,:); %stessa lunghezza
vel=vecnorm(V,2,2);                                 %modulo della velocita' [m/s]

%========== COORDINATE LOCALI NEL SITO ==========
r_sito=R*u_f;                                       %punto di atterraggio
E=[[-sin(phi);cos(phi);0] [0;0;1] u_f];             %Est, Nord, Su nel sito
L=(P-r_sito')*E;                                    %traiettoria in coordinate locali [m]
est=L(:,1)/1000; nord=L(:,2); quota=L(:,3);         %Est in km, Nord e quota in m

%========== LIMITI DEGLI ASSI (per le ombre sulle pareti) ==========
xl=[min(est)-0.5 max(est)+0.5];                     %Est [km]
yl=[min(nord)-5 max(nord)+5];                       %Nord [m]
zl=[0 max(quota)*1.05];                             %Quota [m]
grigio=[0.6 0.6 0.6];

%========== GRAFICO 3D ==========
figure
surface([est est],[nord nord],[quota quota],[vel vel], ...
    'EdgeColor','interp','FaceColor','none','LineWidth',2.5); hold on  %traiettoria colorata con |v|
plot3(est,nord,zl(1)*ones(N,1),':','Color',grigio,'LineWidth',1)      %ombra a terra (Est-Nord)
plot3(est,yl(2)*ones(N,1),quota,':','Color',grigio,'LineWidth',1)      %ombra sulla parete di fondo (Est-Quota)
plot3(xl(1)*ones(N,1),nord,quota,':','Color',grigio,'LineWidth',1)     %ombra sulla parete laterale (Nord-Quota)
plot3(est(1),nord(1),quota(1),'g^','MarkerSize',10,'MarkerFaceColor','g')  %partenza
plot3(0,0,0,'rx','MarkerSize',14,'LineWidth',2.5)                          %sito
colormap(turbo); cb=colorbar; cb.Label.String='|v| [m/s]';
xlim(xl); ylim(yl); zlim(zl)
grid on; box on; view(-35,25)
xlabel('Est [km]'); ylabel('Nord [m]'); zlabel('Quota [m]')
title('Traiettoria di discesa (stato vero)')

