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
dxB = 0;
dyB = 0;
dzB = z + l*omega;
dxG = dxB;
dyG = dyB;

dzG = dzB;
ddxG = l*omega^2+z;
ddyG = z;
ddzG = z;
ddxB = ddxG;
ddyB = ddyG;
ddzB = ddzG;
theta = omega*t;
phi = z; 
omega_x = z;
omega_y = z + omega;
omega_z = z;
domega_x = 0;
domega_y = 0;
domega_z = 0;

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