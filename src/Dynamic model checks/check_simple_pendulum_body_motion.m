addpath("./circular_arrow")

%% Check simple pendulum
% General parameters
N = 1e4;
z = zeros(1,N);

% Body parameters (ponctual mass)
m = 1; %kg
l = 1; %m
xg = 0;
xx = 0;
yy = m*l^2;
zz = m*l^2;

% Mouvement
% theta(t) = omega*t
omega = 2*pi*1;
tf = 10;
t = linspace(0,tf,N);


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
theta = omega*t;
phi = z;

xA = -l*cos(theta);
yA = z;
zA = l*sin(theta);
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
[XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z);
g = 9.81;
XB = m*ddxG-XA-m*g*sin(theta).*cos(phi);
YB = m*ddyG-YA+m*g*sin(theta).*sin(phi);
ZB = m*ddzG-ZA+m*g*cos(theta);
LB = -LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG);
MB = -MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG);
NB = -NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG);

% Forces from the pendulum model
XB_pend = m*l*omega^2-m*g*sin(omega*t);
YB_pend = z;
ZB_pend = m*g*cos(omega*t);
LB_pend = z;
MB_pend = l*m*g*cos(omega*t);
NB_pend = z;

figure,
subplot(3,2,1)
plot(t,XB,t,XB_pend)
legend("gene","pendul")
title("XB")

subplot(3,2,3)
plot(t,YB,t,YB_pend)
legend("gene","pendul")
title("YB")

subplot(3,2,5)
plot(t,ZB,t,ZB_pend)
legend("gene","pendul")
title("ZB")

subplot(3,2,2)
plot(t,LB,t,LB_pend)
legend("gene","pendul")
title("LB")

subplot(3,2,4)
plot(t,MB,t,MB_pend)
legend("gene","pendul")
title("MB")

subplot(3,2,6)
plot(t,NB,t,NB_pend)
legend("gene","pendul")
title("NB")