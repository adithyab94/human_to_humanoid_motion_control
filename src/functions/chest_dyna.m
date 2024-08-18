function [XB,YB,ZB,LB,MB,NB] = chest_dyna(body,XA,YA,ZA,LA,MA,NA,XAl,YAl,ZAl,LAl,MAl,NAl,XAr,YAr,ZAr,LAr,MAr,NAr,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N,joint_poses,body_xi,body_yi,body_zi)
    % this function returns the wrench on the B point of the chest
    % this the chest coordinates
    if (body ~= 16)
        disp("smthing strange in the 'chest dyna' code")
    end
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
    %% Moving the moment (BABAR)
    % Here I compute as the position of the points of the chest (Al and Ar)
    % can move along the time
    z = zeros(1,N);
    LBl = z;
    MBl = z;
    NBl = z;
    LBr = z;
    MBr = z;
    NBr = z;
    for timestep=1:N
        %% Checked, I'm pretty sure the xl,yl,zl,xr,yr,zr are correct
        % Rotation matrix 0Rchest
        R = get_R(body,timestep,body_xi,body_yi,body_zi);
        
        % Point B
        poseB = joint_poses(2,timestep,:); % point B expressed in frame zero
        xBi = poseB(:,:,1);
        yBi = poseB(:,:,2);
        zBi = poseB(:,:,3);
        B_0 = [xBi yBi zBi]';
        B_chest = R'*B_0;
        xB = B_chest(1);
        yB = B_chest(2);
        zB = B_chest(3);
        
        % Point Al
        posel = joint_poses(5,timestep,:); % point Al expressed in frame zero
        xli = posel(:,:,1);
        yli = posel(:,:,2);
        zli = posel(:,:,3);
        Al_0 = [xli yli zli]';
        Al_chest = R'*Al_0;
        xAl = Al_chest(1);
        yAl = Al_chest(2);
        zAl = Al_chest(3);
        
        % Point Ar
        poser = joint_poses(6,timestep,:); % point Ar expressed in frame zero
        xri = poser(:,:,1);
        yri = poser(:,:,2);
        zri = poser(:,:,3);
        Ar_0 = [xri yri zri]';
        Ar_chest = R'*Ar_0;
        xAr = Ar_chest(1);
        yAr = Ar_chest(2);
        zAr = Ar_chest(3);
        
        % Coordinates in the local frame
        xl = xB - xAl;
        yl = yB - yAl;
        zl = zB - zAl;
        xr = xB - xAr;
        yr = yB - yAr;
        zr = zB - zAr;
        %%
        
        LBl(timestep) = LAl(timestep) + zl*YAl(timestep) - yl*ZAl(timestep);
        MBl(timestep) = MAl(timestep) + xl*ZAl(timestep) - zl*XAl(timestep);
        NBl(timestep) = NAl(timestep) + yl*XAl(timestep) - xl*YAl(timestep);
        LBr(timestep) = LAr(timestep) + zr*YAr(timestep) - yr*ZAr(timestep);
        MBr(timestep) = MAr(timestep) + xr*ZAr(timestep) - zr*XAr(timestep);
        NBr(timestep) = NAr(timestep) + yr*XAr(timestep) - xr*YAr(timestep);
    end
    %% length N vectors of the wrench : chest -> hip
    g = 9.81;
    XB = m*ddxG-XA-m*g*sin(theta).*cos(phi)-XAl-XAr;
    YB = m*ddyG-YA+m*g*sin(theta).*sin(phi)-YAl-YAr;
    ZB = m*ddzG-ZA+m*g*cos(theta)-ZAl-ZAr;
    LB = -LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG)-LBl-LBr;
    MB = -MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG)-MBl-MBr;
    NB = -NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG)-NBl-NBr;

end