function [XAl,YAl,ZAl,XAr,YAr,ZAr,LAl,MAl,NAl,LAr,MAr,NAr] = hip_dyna_ds(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N,joint_poses,body_xi,body_yi,body_zi)
    % this function returns the wrench on the B point of the chest
    % this the chest coordinates
    if (body ~= 9)
        disp("smthing strange in the 'hip dyna' code")
    end
    XAl = zeros(1,N);
    YAl = zeros(1,N);
    ZAl = zeros(1,N);
    LAl = zeros(1,N);
    MAl = zeros(1,N);
    NAl = zeros(1,N);
    XAr = zeros(1,N);
    YAr = zeros(1,N);
    ZAr = zeros(1,N);
    LAr = zeros(1,N);
    MAr = zeros(1,N);
    NAr = zeros(1,N);
    xx = xxi(body);
    yy = yyi(body);
    zz = zzi(body);
    m = mi(body);
    l = l_list(body);
    xg = xg_list(body);
    
    %% length N vectors of the wrench : chest -> hip
    g = 9.81;
    XAi = 1/2*(m*ddxG-XA-m*g*sin(theta).*cos(phi));
    YAi = 1/2*(m*ddyG-YA+m*g*sin(theta).*sin(phi));
    ZAi = 1/2*(m*ddzG-ZA+m*g*cos(theta));
    LAi = 1/2*(-LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG));
    MAi = 1/2*(-MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG));
    NAi = 1/2*(-NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG));
    
    XAl = XAi;
    XAr = XAi;
    YAl = YAi;
    YAr = YAi;
    ZAl = ZAi;
    ZAr = ZAi;
    
    %% Moving the moment (BABAR)
    % Here I compute as the position of the points of the chest (Al and Ar)
    % can move along the time
    for timestep=1:N
        %% Checked, I'm pretty sure the xl,yl,zl,xr,yr,zr are correct
        % Rotation matrix 0Rchest
        R = get_R(body,timestep,body_xi,body_yi,body_zi);
        
        % Point B
        poseB = joint_poses(1,timestep,:); % point B expressed in frame zero
        xBi = poseB(:,:,1);
        yBi = poseB(:,:,2);
        zBi = poseB(:,:,3);
        B_0 = [xBi yBi zBi]';
        B_chest = R'*B_0;
        xB = B_chest(1);
        yB = B_chest(2);
        zB = B_chest(3);
        
        % Point Al
        posel = joint_poses(13,timestep,:); % point Al expressed in frame zero
        xli = posel(:,:,1);
        yli = posel(:,:,2);
        zli = posel(:,:,3);
        Al_0 = [xli yli zli]';
        Al_chest = R'*Al_0;
        xAl = Al_chest(1);
        yAl = Al_chest(2);
        zAl = Al_chest(3);
        
        % Point Ar
        poser = joint_poses(14,timestep,:); % point Ar expressed in frame zero
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
        
        LAl(timestep) = LAi(timestep) - zl*YAi(timestep) + yl*ZAi(timestep);
        MAl(timestep) = MAi(timestep) - xl*ZAi(timestep) + zl*XAi(timestep);
        NAl(timestep) = NAi(timestep) - yl*XAi(timestep) + xl*YAi(timestep);
        LAr(timestep) = LAi(timestep) - zr*YAi(timestep) + yr*ZAi(timestep);
        MAr(timestep) = MAi(timestep) - xr*ZAi(timestep) + zr*XAi(timestep);
        NAr(timestep) = NAi(timestep) - yr*XAi(timestep) + xr*YAi(timestep);
    end
    

end