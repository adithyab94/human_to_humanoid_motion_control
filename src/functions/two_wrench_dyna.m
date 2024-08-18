function [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N)
    % this function returns the wrench on the second point of a limb in
    % this limb coordinates
    XB = zeros(1,N);
    YB = zeros(1,N);
    ZB = zeros(1,N);
    LB = zeros(1,N);
    MB = zeros(1,N);
    NB = zeros(1,N);
    xx = xxi(body);
    yy = yyi(body);
    zz = zzi(body);
    m = mi(body);
    l = l_list(body);
    xg = xg_list(body);
    %%
    g = 9.81;
    XB = m*ddxG-XA-m*g*sin(theta).*cos(phi);
    YB = m*ddyG-YA+m*g*sin(theta).*sin(phi);
    ZB = m*ddzG-ZA+m*g*cos(theta);
    LB = -LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG);
    MB = -MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG);
    NB = -NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG);
    
end