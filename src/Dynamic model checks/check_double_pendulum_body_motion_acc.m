clear all
addpath("./circular_arrow")

%% Check simple pendulum
% General parameters
N = 1e4;
z = zeros(1,N);

% Body parameters (ponctual mass)
m1 = 1; %kg
l1 = 1; %m
xg1 = 0;
xx1 = 0;
yy1 = m1*l1^2;
zz1 = m1*l1^2;
m2 = m1*2; %kg
l2 = l1*2; %m
xg2 = 0;
xx2 = 0;
yy2 = m2*l2^2;
zz2 = m2*l2^2;

% Mouvement
tf = 10;
t = linspace(0,tf,N);
omega1 = 2*pi*1;
theta01 = 90*pi/180;
theta1 = theta01*cos(omega1*t);
dtheta1_force = -theta01*omega1*sin(omega1*t);
ddtheta1_force = -theta01*omega1^2*cos(omega1*t);
omega2 = omega1/2;
theta02 = 90*pi/180;
theta2 = theta02*cos(omega2*t);
dtheta2_force = -theta02*omega2*sin(omega2*t);
ddtheta2_force = -theta02*omega2^2*cos(omega2*t);
% theta2 = z;
% dtheta2_force = z;
% ddtheta2_force = z;

% Points
xA2 = -l2*cos(theta2);
zA2 = l2*sin(theta2);
xA1 = xA2 -l1*cos(theta1);
zA1 = zA2 +l1*sin(theta1);

theta = theta1;
l = l1;
xg = xg1;
% Prepare for filtering
span = 20;

dxB = zeros(1,N);
dyB = zeros(1,N);
dzB = zeros(1,N);
dxG = zeros(1,N);
dyG = zeros(1,N);
dzG = zeros(1,N);
ddxG = zeros(1,N);
ddyG = zeros(1,N);
ddzG = zeros(1,N);
%     ddxB = zeros(1,N);
ddyB = zeros(1,N);
ddzB = zeros(1,N);
omega_x = zeros(1,N);
omega_y = zeros(1,N);
omega_z = zeros(1,N);
domega_x = zeros(1,N);
domega_y = zeros(1,N);
domega_z = zeros(1,N);
theta_tot = zeros(1,N);
phi_tot = zeros(1,N);
dtss = diff(t);

psi = z;

phi = z;

xA = xA1;
yA = z;
zA = zA1;
xA = smooth(xA,span)';
yA = smooth(yA,span)';
zA = smooth(zA,span)';

dxA = diff(xA)./dtss;
dxA = smooth(dxA,span)';
dyA = diff(yA)./dtss;
dyA = smooth(dyA,span)';
dzA = diff(zA)./dtss;
dzA = smooth(dzA,span)';

ddxA = diff(dxA)./dtss(1:end-1);
ddxA = smooth(ddxA,span)';
ddyA = diff(dyA)./dtss(1:end-1);
ddyA = smooth(ddyA,span)';
ddzA = diff(dzA)./dtss(1:end-1);
ddzA = smooth(ddzA,span)';

psi = smooth(psi,span)';
theta = smooth(theta,span)';
phi = smooth(phi,span)';

dpsi = diff(psi)./dtss;
dpsi = smooth(dpsi,span)';
dtheta = diff(theta)./dtss;
dtheta = smooth(dtheta,span)';
dphi = diff(phi)./dtss;
dphi = smooth(dphi,span)';

ddpsi = diff(dpsi)./dtss(1:end-1);
ddpsi = smooth(ddpsi,span)';
ddtheta = diff(dtheta)./dtss(1:end-1);
ddtheta = smooth(ddtheta,span)';
ddphi = diff(dphi)./dtss(1:end-1);
ddphi = smooth(ddphi,span)';

for k = 1:N-1
    th = theta(k);
    R = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
    %% Velocities
    omega_x(k) = dtheta(k)*sin(phi(k))-dpsi(k)*sin(theta(k))*cos(phi(k));
    omega_y(k) = dtheta(k)*cos(phi(k))+dpsi(k)*sin(theta(k))*sin(phi(k));
    omega_z(k) = dphi(k) + dpsi(k)*cos(theta(k));

    dOPA = [dxA(k) dyA(k) dzA(k)]';
    diPB = R'*dOPA + l*[0;omega_z(k);-omega_y(k)];
    diPG = R'*dOPA + xg*[0;omega_z(k);-omega_y(k)];

    dxB(k) = diPB(1);
    dyB(k) = diPB(2);
    dzB(k) = diPB(3);

    dxG(k) = diPG(1);
    dyG(k) = diPG(2);
    dzG(k) = diPG(3); 

end

%% Accelerations
for k =1:N-2
    th = theta(k);
    R = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
    
    domega_x(k) = ddtheta(k)*sin(phi(k))+dtheta(k)*dphi(k)*cos(phi(k))-ddpsi(k)*sin(theta(k))*cos(phi(k))-dpsi(k)*dtheta(k)*cos(theta(k))*cos(phi(k))+dpsi(k)*dphi(k)*sin(theta(k))*sin(phi(k));
    domega_y(k) = ddtheta(k)*cos(phi(k))-dtheta(k)*dphi(k)*sin(phi(k))+ddpsi(k)*sin(theta(k))*sin(phi(k))+dpsi(k)*dtheta(k)*cos(theta(k))*sin(phi(k))+dpsi(k)*dphi(k)*sin(theta(k))*cos(phi(k));
    domega_z(k) = ddphi(k)+ddpsi(k)*cos(theta(k))-dpsi(k)*dtheta(k)*sin(theta(k));

    ddOPA = [ddxA(k) ddyA(k) ddzA(k)]';
    ddiPB = R'*ddOPA + l*[-omega_z(k)^2-omega_y(k)^2;domega_z(k)+omega_x(k)*omega_y(k);-domega_y(k)+omega_x(k)*omega_z(k)];
    ddiPG = R'*ddOPA + xg*[-omega_z(k)^2-omega_y(k)^2;domega_z(k)+omega_x(k)*omega_y(k);-domega_y(k)+omega_x(k)*omega_z(k)];

    ddxB(k) = ddiPB(1);
    ddyB(k) = ddiPB(2);
    ddzB(k) = ddiPB(3);

    ddxG(k) = ddiPG(1);
    ddyG(k) = ddiPG(2);
    ddzG(k) = ddiPG(3); 
end

% Forces from the general model
m = m1;
xx = xx1;
yy = yy1;
zz = zz1;
[XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z);
g = 9.81;
XB1 = m*ddxG-XA-m*g*sin(theta).*cos(phi);
YB1 = m*ddyG-YA+m*g*sin(theta).*sin(phi);
ZB1 = m*ddzG-ZA+m*g*cos(theta);
LB1 = -LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG);
MB1 = -MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG);
NB1 = -NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG);

l = l2;
xg = xg2;
theta = theta2;
% Prepare for filtering
span = 1;

dxB = zeros(1,N);
dyB = zeros(1,N);
dzB = zeros(1,N);
dxG = zeros(1,N);
dyG = zeros(1,N);
dzG = zeros(1,N);
ddxG = zeros(1,N);
ddyG = zeros(1,N);
ddzG = zeros(1,N);
%     ddxB = zeros(1,N);
ddyB = zeros(1,N);
ddzB = zeros(1,N);
omega_x = zeros(1,N);
omega_y = zeros(1,N);
omega_z = zeros(1,N);
domega_x = zeros(1,N);
domega_y = zeros(1,N);
domega_z = zeros(1,N);
theta_tot = zeros(1,N);
phi_tot = zeros(1,N);
dtss = diff(t);

psi = z;

phi = z;

xA = xA2;
yA = z;
zA = zA2;
xA = smooth(xA,span)';
yA = smooth(yA,span)';
zA = smooth(zA,span)';

dxA = diff(xA)./dtss;
dxA = smooth(dxA,span)';
dyA = diff(yA)./dtss;
dyA = smooth(dyA,span)';
dzA = diff(zA)./dtss;
dzA = smooth(dzA,span)';

ddxA = diff(dxA)./dtss(1:end-1);
ddxA = smooth(ddxA,span)';
ddyA = diff(dyA)./dtss(1:end-1);
ddyA = smooth(ddyA,span)';
ddzA = diff(dzA)./dtss(1:end-1);
ddzA = smooth(ddzA,span)';

psi = smooth(psi,span)';
theta = smooth(theta,span)';
phi = smooth(phi,span)';

dpsi = diff(psi)./dtss;
dpsi = smooth(dpsi,span)';
dtheta = diff(theta)./dtss;
dtheta = smooth(dtheta,span)';
dphi = diff(phi)./dtss;
dphi = smooth(dphi,span)';

ddpsi = diff(dpsi)./dtss(1:end-1);
ddpsi = smooth(ddpsi,span)';
ddtheta = diff(dtheta)./dtss(1:end-1);
ddtheta = smooth(ddtheta,span)';
ddphi = diff(dphi)./dtss(1:end-1);
ddphi = smooth(ddphi,span)';

for k = 1:N-1
    th = theta(k);
    R = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
    %% Velocities
    omega_x(k) = dtheta(k)*sin(phi(k))-dpsi(k)*sin(theta(k))*cos(phi(k));
    omega_y(k) = dtheta(k)*cos(phi(k))+dpsi(k)*sin(theta(k))*sin(phi(k));
    omega_z(k) = dphi(k) + dpsi(k)*cos(theta(k));

    dOPA = [dxA(k) dyA(k) dzA(k)]';
    diPB = R'*dOPA + l*[0;omega_z(k);-omega_y(k)];
    diPG = R'*dOPA + xg*[0;omega_z(k);-omega_y(k)];

    dxB(k) = diPB(1);
    dyB(k) = diPB(2);
    dzB(k) = diPB(3);

    dxG(k) = diPG(1);
    dyG(k) = diPG(2);
    dzG(k) = diPG(3); 

end

%% Accelerations
for k =1:N-2
    th = theta(k);
    R = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
    
    domega_x(k) = ddtheta(k)*sin(phi(k))+dtheta(k)*dphi(k)*cos(phi(k))-ddpsi(k)*sin(theta(k))*cos(phi(k))-dpsi(k)*dtheta(k)*cos(theta(k))*cos(phi(k))+dpsi(k)*dphi(k)*sin(theta(k))*sin(phi(k));
    domega_y(k) = ddtheta(k)*cos(phi(k))-dtheta(k)*dphi(k)*sin(phi(k))+ddpsi(k)*sin(theta(k))*sin(phi(k))+dpsi(k)*dtheta(k)*cos(theta(k))*sin(phi(k))+dpsi(k)*dphi(k)*sin(theta(k))*cos(phi(k));
    domega_z(k) = ddphi(k)+ddpsi(k)*cos(theta(k))-dpsi(k)*dtheta(k)*sin(theta(k));

    ddOPA = [ddxA(k) ddyA(k) ddzA(k)]';
    ddiPB = R'*ddOPA + l*[-omega_z(k)^2-omega_y(k)^2;domega_z(k)+omega_x(k)*omega_y(k);-domega_y(k)+omega_x(k)*omega_z(k)];
    ddiPG = R'*ddOPA + xg*[-omega_z(k)^2-omega_y(k)^2;domega_z(k)+omega_x(k)*omega_y(k);-domega_y(k)+omega_x(k)*omega_z(k)];

    ddxB(k) = ddiPB(1);
    ddyB(k) = ddiPB(2);
    ddzB(k) = ddiPB(3);

    ddxG(k) = ddiPG(1);
    ddyG(k) = ddiPG(2);
    ddzG(k) = ddiPG(3); 
end


% Forces from the general model
m = m2;
xx = xx2;
yy = yy2;
zz = zz2;

F_new = zeros(3,N);
M_new = zeros(3,N);
F_test_list = zeros(3,N);
for k=1:N
    F = [XB1(k) YB1(k) ZB1(k)]';
    M = [LB1(k) MB1(k) NB1(k)]';
    th = theta1(k);
    R0 = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)]; 
    th = theta2(k);
    R1 = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];   
    F_new(:,k) = R1'*R0*F;
    M_new(:,k) = R1'*R0*M;
end
XA = -F_new(1,:);
YA = -F_new(2,:);
ZA = -F_new(3,:);
LA = -M_new(1,:);
MA = -M_new(2,:);
NA = -M_new(3,:);  

% [XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z);

XB2 = m*ddxG-XA-m*g*sin(theta).*cos(phi);
YB2 = m*ddyG-YA+m*g*sin(theta).*sin(phi);
ZB2 = m*ddzG-ZA+m*g*cos(theta);
LB2 = -LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG);
MB2 = -MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG);
NB2 = -NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG);


% Forces from the pendulum model
theta1_force = theta1;
theta2_force = theta2;
XB1_pend = m1*(l1*dtheta1_force.^2+l2*ddtheta2_force.*sin(theta2_force-theta1_force)+l2*dtheta2_force.^2.*cos(theta2_force-theta1_force)-g*sin(theta1_force));
YB1_pend = z;
ZB1_pend = m1*(l1*ddtheta1_force+l2*ddtheta2_force.*cos(theta2_force-theta1_force)-l2*dtheta2_force.^2.*sin(theta2_force-theta1_force)+g*cos(theta1_force));
LB1_pend = z;
delta1 = m1*(l1^2*ddtheta1_force+l1*l2*ddtheta2_force.*cos(theta2_force-theta1_force)-l1*l2*dtheta2_force.^2.*sin(theta2_force-theta1_force));
MB1_pend = delta1+l1*m1*g*cos(theta1_force);
NB1_pend = z;

XB2_pend = m2*l2*dtheta2_force.^2-m2*g*sin(theta2_force)+XB1_pend.*cos(theta2_force-theta1_force)-ZB1_pend.*sin(theta2_force-theta1_force);
YB2_pend = z;
ZB2_pend = m2*l2*ddtheta2_force+m2*g*cos(theta2_force)+XB1_pend.*sin(theta2_force-theta1_force)+ZB1_pend.*cos(theta2_force-theta1_force);
LB2_pend = z;
MB2_pend = m2*l2^2*ddtheta2_force+MB1_pend+l2*(XB1_pend.*sin(theta2_force-theta1_force)+ZB1_pend.*cos(theta2_force-theta1_force))+l2*m2*g*cos(theta2_force);
NB2_pend = z;

% Plots
%%
figHandle = figure(1);
a = 1.2;
xmax = a*(l1+l2);
xmin = -a*(l1+l2);
zmax = a*(l1+l2);
zmin = -a*(l1+l2);
k = 1;
dt = t(2)-t(1);
slow_down = 0.5;

% Moment arrow
radius = 0.1; % Height from top to bottom
centre = [0 0];
arrow_angle = 0; % Desired orientation angle in degrees
moment_angle_scale = 180/max(max((abs(MB1))),max((abs(MB2)))); % Anglebetween start and end of arrow
head_size = 3; % Arrow head size

% % Points
% xA2 = -l2*cos(theta2);
% zA2 = l2*sin(theta2);
% xA1 = xA2 -l1*cos(theta1);
% zA1 = zA2 +l1*sin(theta1);

while k < N
    clf
    tic,
    
    scale = 0.01;
    plot([0,zA2(k),zA1(k)],[0,xA2(k),xA1(k)],'-ok')
    hold on
    
    centre = [0 0];
    fxx2_pend = XB2_pend(k)*cos(theta2(k));
    fxz2_pend = -XB2_pend(k)*sin(theta2(k));
    fzx2_pend = ZB2_pend(k)*sin(theta2(k));
    fzz2_pend = ZB2_pend(k)*cos(theta2(k));
    quiver([centre(1) centre(1)],[centre(2) centre(2)],scale*[fxz2_pend fzz2_pend],scale*[fxx2_pend fzx2_pend],'r')
    
    th = theta2(k);
    R = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
    F2_pend = R*[XB2_pend(k) 0 ZB2_pend(k)]';
    quiver(centre(1),centre(2) ,scale*F2_pend(3),scale*F2_pend(1),'r','LineWidth',2)
    
    fxx2 = XB2(k)*cos(theta2(k));
    fxz2 = -XB2(k)*sin(theta2(k));
    fzx2 = ZB2(k)*sin(theta2(k));
    fzz2 = ZB2(k)*cos(theta2(k));
    quiver([centre(1) centre(1)],[centre(2) centre(2)],scale*[fxz2 fzz2],scale*[fxx2 fzx2],'b')
    
    F2 = R*[XB2(k) 0 ZB2(k)]';
    quiver(centre(1),centre(2) ,scale*F2(3),scale*F2(1),'b','LineWidth',2)
    
    colour = 'r'; % Colour of arrow
    angle = MB2_pend(k)*moment_angle_scale; % Anglebetween start and end of arrow
    if(sign(MB2_pend(k)) == 1)
        direction = 0; % for CW enter 1, for CCW enter 0
    else
        direction = 1; % for CW enter 1, for CCW enter 0
    end
    circular_arrow(figHandle, radius, centre, arrow_angle, angle, direction, colour, head_size);
    
    colour = 'b'; % Colour of arrow
    angle = MB2(k)*moment_angle_scale; % Anglebetween start and end of arrow
    if(sign(MB2(k)) == 1)
        direction = 0; % for CW enter 1, for CCW enter 0
    else
        direction = 1; % for CW enter 1, for CCW enter 0
    end
    circular_arrow(figHandle, radius, centre, arrow_angle, angle, direction, colour, head_size);
    
    
    centre = [zA2(k) xA2(k)];
    fxx1_pend = XB1_pend(k)*cos(theta1(k));
    fxz1_pend = -XB1_pend(k)*sin(theta1(k));
    fzx1_pend = ZB1_pend(k)*sin(theta1(k));
    fzz1_pend = ZB1_pend(k)*cos(theta1(k));    
    quiver([centre(1) centre(1)],[centre(2) centre(2)],scale*[fxz1_pend fzz1_pend],scale*[fxx1_pend fzx1_pend],'r')
    
    th = theta1(k);
    R = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
    F1_pend = R*[XB1_pend(k) 0 ZB1_pend(k)]';
    quiver(centre(1),centre(2) ,scale*F1_pend(3),scale*F1_pend(1),'r','LineWidth',2)
    
    fxx1 = XB1(k)*cos(theta1(k));
    fxz1 = -XB1(k)*sin(theta1(k));
    fzx1 = ZB1(k)*sin(theta1(k));
    fzz1 = ZB1(k)*cos(theta1(k));
    quiver([centre(1) centre(1)],[centre(2) centre(2)],scale*[fxz1 fzz1],scale*[fxx1 fzx1],'b')
    
    F1 = R*[XB1(k) 0 ZB1(k)]';
    quiver(centre(1),centre(2) ,scale*F1(3),scale*F1(1),'b','LineWidth',2)
    
    colour = 'r'; % Colour of arrow
    angle = MB1_pend(k)*moment_angle_scale; % Anglebetween start and end of arrow
    if(sign(MB1_pend(k)) == 1)
        direction = 0; % for CW enter 1, for CCW enter 0
    else
        direction = 1; % for CW enter 1, for CCW enter 0
    end
    circular_arrow(figHandle, radius, centre, arrow_angle, angle, direction, colour, head_size);
    
    colour = 'b'; % Colour of arrow
    angle = MB1(k)*moment_angle_scale; % Anglebetween start and end of arrow
    if(sign(MB1(k)) == 1)
        direction = 0; % for CW enter 1, for CCW enter 0
    else
        direction = 1; % for CW enter 1, for CCW enter 0
    end
    circular_arrow(figHandle, radius, centre, arrow_angle, angle, direction, colour, head_size);
    
    
%     F = [XB1(k) YB1(k) ZB1(k)]';
%     th = theta1(k);
%     R0 = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)]; 
%     th = theta2(k);
%     R1 = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];   
%     F_test = R1'*R0*F;
%     fxx_test = F_test(1)*cos(theta2(k));
%     fxz_test = -F_test(1)*sin(theta2(k));
%     fzx_test = F_test(3)*sin(theta2(k));
%     fzz_test = F_test(3)*cos(theta2(k));
%     quiver([centre(1) centre(1)],[centre(2) centre(2)],scale*[fxz_test fzz_test],scale*[fxx_test fzx_test],'o','LineWidth',2)
%     F_test_list(:,k) = F_test;
    
    axis equal
    xlim([zmin,zmax])
    ylim([xmin,xmax])    
    
    grid on
    title("t = " + num2str(round(t(k),2))+ " s");
    xlabel("z (m)")
    ylabel("x (m)")
    drawnow
    
    t_end = toc;
    k = k + round(slow_down*(t_end/dt));
end

%%
figure,
subplot(3,2,1)
plot(t,XB1,t,XB1_pend)
legend("gene","pendul")
title("XB1")

subplot(3,2,3)
plot(t,YB1,t,YB1_pend)
legend("gene","pendul")
title("YB1")

subplot(3,2,5)
plot(t,ZB1,t,ZB1_pend)
legend("gene","pendul")
title("ZB1")

subplot(3,2,2)
plot(t,LB1,t,LB1_pend)
legend("gene","pendul")
title("LB1")

subplot(3,2,4)
plot(t,MB1,t,MB1_pend)
legend("gene","pendul")
title("MB1")

subplot(3,2,6)
plot(t,NB1,t,NB1_pend)
legend("gene","pendul")
title("NB1")


figure,
subplot(3,2,1)
plot(t,XB2,t,XB2_pend)
legend("gene","pendul")
title("XB2")

subplot(3,2,3)
plot(t,YB2,t,YB2_pend)
legend("gene","pendul")
title("YB2")

subplot(3,2,5)
plot(t,ZB2,t,ZB2_pend)
legend("gene","pendul")
title("ZB2")

subplot(3,2,2)
plot(t,LB2,t,LB2_pend)
legend("gene","pendul")
title("LB2")

subplot(3,2,4)
plot(t,MB2,t,MB2_pend)
legend("gene","pendul")
title("MB2")

subplot(3,2,6)
plot(t,NB2,t,NB2_pend)
legend("gene","pendul")
title("NB2")

