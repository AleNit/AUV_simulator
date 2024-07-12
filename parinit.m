
% compute preprocessing data

function [par,PID]=parinit(par)

% propeller rotation rate in rps
par.n=par.n/60;

% fluid properties (water)
par.rhof=1025;                                                              % sea water density
par.viscdin=1.2e-3;                                                         % sea water dynamic viscosity
par.cdf=0.2;                                                                % hull form drag at zero angle of attack based on cross-sectional area

% obstacle avoidance parameters
par.obsa=2*par.delmax;
par.obsb=30.0;
par.obsc=20.0;

% geometric quantities
par.R=par.L/(2*(par.gam-1));
par.V=4/3*pi*par.R^3+pi*par.R^2*(par.L-2*par.R);
m=par.rhos*par.V;
Im=par.rhos*pi*par.R^5*((par.gam-1)/6*(3+4*(par.gam-1)^2)+4/3*(83/320+(par.gam-5/8)^2));
par.Md=[m,m,Im];   
par.Mad=[par.rhof*par.V,m/10,par.L^2*par.rhof*par.V/12];   

% coefficients of the PID controller
PID.Kpid=[0.8,0,2];                                                         % proportional, integral, derivative

% drag coefficient of circular cylinder as a function of the Reynolds 
% number (LH Jorgensen - 1973)
filen='./cd.txt';
fileID = fopen(filen,'r');
formatSpec = '%f %f';
size = [2 Inf];
scval = fscanf(fileID,formatSpec,size);
fclose(fileID);

par.Rei=scval(1,:);
par.cdi=scval(2,:);
clear scval


% drag proportionality factor as a function of the length-to-diameter ratio
% of circular cylinder (LH Jorgensen - 1973)
filen='./eta.txt';
fileID = fopen(filen,'r');
formatSpec = '%f %f';
size = [2 Inf];
scval = fscanf(fileID,formatSpec,size);
fclose(fileID);

AR=scval(1,:);
eta=scval(2,:);
clear scval

par.eta=interp1(AR,eta,par.gam);


% propulsion coefficient as a function of the advance ratio J, for a 3 blade propeller
% with pitch-to-diameter ratio = 1.4 (Newmann 1977, Fig. 2.10)
filen='./Kp.txt';
fileID = fopen(filen,'r');
formatSpec = '%f %f';
size = [2 Inf];
scval = fscanf(fileID,formatSpec,size);
fclose(fileID);

par.Ji=scval(1,:);
par.Kti=scval(2,:);
clear scval


end
