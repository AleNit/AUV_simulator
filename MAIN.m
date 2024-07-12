
% dynamic and control of an AUV

clc
clear 
close all


%% input parameters
svf=false(1);                                               % save frames to file
tf=400;                                                     % final simulation time
par.dt=0.2;                                                 % time step size
pltstep=20;                                                 % step interval for output plot
par.LL=150;                                                 % side length of the domain
par.tlocx=[0.25,0.72,0.20,0.48,0.85].*par.LL;               % location of target points
par.tlocy=[0.71,0.76,0.51,0.19,0.35].*par.LL;
par.gam=6;                                                  % swimmer aspect ratio
par.L=1.0;                                                  % swimmer length
par.rhos=1100;                                              % swimmer density
par.alp=pi/3;                                               % angle of the (unitary) farfield velocity
par.U=0.2;                                                  % farfield velocity magnitude [m/s]
par.n=1200;                                                 % propeller rotational rate [rev/min]
par.apr=5*par.L;                                            % target approach radius
par.delmax=25*pi/180;                                       % maximum turning radius
par.obs=[0.5,0.6].*par.LL;                                  % obstacle location


%% pre-processing operations
[par,PID]=parinit(par);

% initialize variables
ns = ceil(tf/par.dt); 
t=0;
xo=[40,40,0,1.0e-5,1.0e-5,0];
xn=zeros(ns,6);
xn(1,:)=xo;


%% start time loop
figure(1)
% FF=figure('visible','off');
set(gcf,'position',[100,20,860,670])
par.itar=1;
del=0;
PID.int=0; PID.err=0; PID.erro=0;
for i=1:ns
    
    % plot current configuration
    if (mod(i-1,pltstep)==0)
        fprintf(1,'t = %0.3f, \t del = %0.3f, \t tar # %02d \n',t,del*180/pi,par.itar);
        plotconf(t,par,PID,xn,i,del,svf,i)        
    end
    
    % Runge-Kutta substeps
    [k1,del,PID] = dxdt(1,xo,t,par,del,PID);
    
    tmp = xo + k1'.*par.dt/2;    
    [k2,~,~] = dxdt(2,tmp,t+par.dt/2,par,del,PID);
    
    tmp = xo + k2'.*par.dt/2;
    [k3,~,~] = dxdt(3,tmp,t+par.dt/2,par,del,PID);
    
    tmp = xo + k3'.*par.dt;
    [k4,~,~] = dxdt(4,tmp,t+par.dt,par,del,PID);    
        
    xn(i+1,:) = xo + par.dt.*(k1'./6 + k2'./3 + k3'./3 + k4'./6);
    
    xo = xn(i+1,:);
    
    t=t+par.dt;

    % store quantities of interest
    time(i+1)=t;
    errot(i+1)=PID.erro;
    delt(i+1)=del;
    
    % update target if necessary
    dist=sqrt((par.tlocx(par.itar)-xn(i,1))^2+(par.tlocy(par.itar)-xn(i,2))^2);
    if (dist<par.apr && par.itar<length(par.tlocx))
        par.itar=par.itar+1;
    end
    
    % stop simulation if the agent exit the domain
    if (any(xn(i,1:2)>par.LL) | any(xn(i,1:2)<0))
        error('...agent out of the domain')
    end
    
end


%% plot time-traces of error and rudder angle
figure(2)
tiledlayout(2,1)        
nexttile
plot(time,delt.*180/pi,'-')
xlabel('$t$ [s]','Interpreter','latex','FontSize',14)
ylabel('$ \delta$ [deg]','Interpreter','latex','FontSize',14)
nexttile
plot(time,errot.*180/pi,'-')
xlabel('$t$ [s]','Interpreter','latex','FontSize',14)
ylabel('$e$ [deg]','Interpreter','latex','FontSize',14)
