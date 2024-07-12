
function plotconf(t,par,PID,xn,i,del,svframes,inst)

scol=[0.86,0,0];  % swimmer color
ccol='#0072BD';  % current color

% absolute velocity and travelled distance
v=[xn(i,4);xn(i,5)];
travel=sum(sqrt( (xn(2:i,1)-xn(1:i-1,1)).^2 + (xn(2:i,2)-xn(1:i-1,2)).^2 ));

hold off
delete(findall(gcf,'type','annotation'))

% plot target points
plot(par.tlocx,par.tlocy,'o','Color',scol)        
hold on
text(par.tlocx-3.3,par.tlocy+3,string(1:length(par.tlocx)),'color',scol)
xlabel('$x$ [m]','interpreter','latex','fontsize',14)
ylabel('$y$ [m]','interpreter','latex','fontsize',14)

% plot obstacle and avoidance circle
plot(par.obs(1),par.obs(2),'xk')  
plot(par.obs(1),par.obs(2),'ok')  
% tho=linspace(0,2*pi,100);
% xco=par.obs(1)+par.obsb*cos(tho);
% yco=par.obs(2)+par.obsb*sin(tho);
% plot(xco,yco,'--b')


% create triangle, then rotate and translate it
l=par.L*4;
at=75*pi/180;
A=[-l*cot(at),-l/2];
B=[l*cot(at),-l/2];
C=[0,l/2];
R=[cos(xn(i,3)),-sin(xn(i,3));sin(xn(i,3)),cos(xn(i,3))];
A=R*A'+xn(i,1:2)';
B=R*B'+xn(i,1:2)';
C=R*C'+xn(i,1:2)';
patch([A(1),B(1),C(1)],[A(2),B(2),C(2)],scol)

% plot swimmer trajectory
plot(xn(1:i,1),xn(1:i,2),'--','Color',scol)

axis equal
axis([0,par.LL,0,par.LL])

% show current orientation
lf=par.LL/14;
arrows(0.1*par.LL,0.9*par.LL,lf,90-par.alp*180/pi,'FaceColor',ccol,'LineWidth',0.05)
th=0:0.01:2*pi;
xcb=0.1*par.LL+lf/2*cos(par.alp) + lf/2.*cos(th);
ycb=0.9*par.LL+lf/2*sin(par.alp) + lf/2.*sin(th);
plot(xcb,ycb,'-','color',ccol)
annotation(gcf,'textbox',[0.21,0.825,0.0527,0.052],'Color',ccol,...
   'String',{'$U_{\infty}$'},'LineStyle','none','Interpreter','latex','FontSize',14);

set(gca,'position',[0.03,0.1,0.77,0.815])

% include textbox with data
annotation(gcf,'textbox',[0.74,0.265,0.2,0.65],'String',{ ...
        'case setting:', ...
        strcat('$L=',num2str(par.L),'\, m$'), ...
        strcat('$\gamma=',num2str(par.gam),'$'), ...        
        strcat('$U_{\infty}=',num2str(par.U),'\, m/s$'), ...
        strcat('$\alpha=',num2str(par.alp*180/pi),'^{\circ}$'), ...
        strcat('$n=',sprintf('%0.1f',par.n*60),'\, rpm$'), ...        
        strcat('$\delta_{max}=',num2str(par.delmax*180/pi),'^{\circ}$'), ...
        ' ', ...
        'operating par.s:', ...
        strcat('$t=',sprintf('%0.2f',t),'\, s$'), ...
        strcat('$|\mathbf{v}|=',sprintf('%0.2f',norm(v,2)),'\, m/s$'), ...        
        strcat('$\delta=',sprintf('%+05.1f',del*180/pi),'^{\circ}$'), ...
        strcat('$d_t=',sprintf('%0.2f',travel),'\, m$')}, ...
    'Interpreter','latex','FontSize',13,'FitBoxToText','off','margin',10);

drawnow

if (svframes)
    picname=strcat('./frames/frame_',sprintf('%05d',inst),'.png');    
    exportgraphics(gcf,picname,'Resolution',180)            
end


end
