
% time advancement step for AUV model

function [xdot,del,PID] = dxdt(st,x,t,par,del,PID)

% rotation matrix, from local frame to lab frame
theta=x(3);
R=[cos(theta),-sin(theta);sin(theta),cos(theta)];
Ri=R';

% geometric parameters for force coefficients
dd=2*par.R;                                                                 % characteristic length
len=par.L+2*par.R;                                                          % hull length
Ap=pi*par.R^2+2*par.R*par.L;                                                % hull planform area
A=pi*par.R^2;                                                               % hull reference area (max cross-sectional area)

% relative current velocity and Reynolds number
U=[par.U*cos(par.alp);par.U*sin(par.alp)];
v=[x(4);x(5)];
u=U-v;
un=norm(u,2);
Re=un*dd*par.rhof/par.viscdin;
uc=Ri*u;
ucn=un;
m=[0,1];
phi=atan2(uc(1)*m(2)-uc(2)*m(1),uc(1)*m(1)+uc(2)*m(2));                     % incidence angle
phi=sign(phi)*pi-phi;
phin=sign(cos(phi)).*phi + sign(phi)*floor(abs(phi)/(pi/2))*pi;             % incidence angle in [-pi/2,pi/2]
Ren=Re*abs(sin(phin));                                                      % Reynolds number component normal to body axis

% hydrodynamic force and torque (body frame)
cdc=interp1(par.Rei,par.cdi,Ren,'linear','extrap');                         % crossflow drag coeff. of circular infinite cylinder
cA=par.cdf*(cos(phin))^2;
cN=sin(2*phin)*cos(phin/2)+par.eta*cdc*Ap/A*(sin(phin))^2;
cM=sign(cos(phi))*(par.V-A*len/2)/(A*dd)*sin(2*phin)*cos(phin/2);
xce=dd*cM/cN;
Fdl=0.5*par.rhof*ucn^2*A*cN;
Fdm=-0.5*par.rhof*ucn^2*A*cA;
Td=xce*Fdl;    

% update propeller orientation: PID controller with saturation
if (st==1)

    delo=del;

    tar=Ri*[par.tlocx(par.itar)-x(1);par.tlocy(par.itar)-x(2)]; % bring target points to the body frame

    err=atan2(tar(1)*m(2)-tar(2)*m(1),tar(1)*m(1)+tar(2)*m(2));

    % augment error for collision avoidance
    ro=sqrt( (x(1)-par.obs(1))^2 + (x(2)-par.obs(2))^2 );
    erra=par.obsa*exp( (1-1/(1-min(1,ro^2/(10*par.obsb))))/par.obsc );
    err=err+erra;
    
    del=PID.Kpid(1)*err + PID.Kpid(2)*(PID.int+err*par.dt) + PID.Kpid(3)*(err-PID.erro)/par.dt;
    del=sign(del)*min([abs(del),par.delmax]);    
    
    % under-relax the controller output to avoid rudder jumps    
    up=0.05;
    del=up*del+(1-up)*delo;
    
    PID.erro=err; 
    PID.int=PID.int+err*par.dt; 

end

% propulsion coefficient
Dp=par.R;                                                                   % assume the propeller have half the diameter of torpedo
vax=abs(norm(uc,2)*cos(phi));                                               % axial speed of the propeller
% vax=abs(norm(uc,2)*cos(pi-phi+del));                                      % axial speed of the propeller
J=vax/(par.n*Dp);                                                           % advance ratio
J=min([J,1.5]);                                                             % avoid negative Kt values when the axial velocity is large
Kt = interp1(par.Ji,par.Kti,J);

% propulsion forces
Ft=par.rhof*Kt*par.n^2*Dp^4;
Ftm=Ft*cos(del);
Ftl=Ft*sin(del);
Tt=-Ft*(par.L/2+par.R)*sin(del);

% rotate and invert added mass matrix
MadR=Ri*diag(par.Mad(1:2))*R;
MadR3=eye(3); MadR3(1:2,1:2)=MadR; MadR3(3,3)=par.Mad(3);
M=diag(par.Md)+MadR3;
Minv=inv(M);

% solve for the velocity state-space vector
Mi=[eye(3),zeros(3,3);zeros(3,3),Minv];
Fxy=R*[Fdl+Ftl;Fdm+Ftm];
rhs=[x(4),x(5),x(6),Fxy(1),Fxy(2),Td+Tt]';
xdot=Mi*rhs;

end

