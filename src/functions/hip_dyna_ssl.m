function [XAl,YAl,ZAl,LAl,MAl,NAl] = hip_dyna_ssl(body,XAr,YAr,ZAr,LAr,MAr,NAr,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N,joint_poses,body_xi,body_yi,body_zi)
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

    xx = xxi(body);
    yy = yyi(body);
    zz = zzi(body);
    m = mi(body);
    l = l_list(body);
    xg = xg_list(body);
    
    %% length N vectors of the wrench : chest -> hip
    g = 9.81;
    
    LB = (-LA+xx*domega_x-(yy*omega_y-m*(xg-l)*dzB).*omega_z + (zz*omega_z+m*(xg-l)*dyB).*omega_y+m*(dyB.*dzG-dzB.*dyG));
    MB = (-MA-l*ZA-m*g*(xg-l)*cos(theta)+yy*domega_y-m*(xg-l)*ddzB+xx*omega_x.*omega_z-(zz*omega_z+m*(xg-l)*dyB).*omega_x+m*(-dxB.*dzB+dzB.*dxG));
    NB = (-NA+l*YA+m*g*(xg-l)*sin(theta).*sin(phi)+zz*domega_z+m*(xg-l)*ddyB-xx*omega_x.*omega_y+(yy*omega_y-m*(xg-l)*ddzB).*omega_x+m*(dxB.*dyG-dyB.*dxG));
    
    XAl = m*ddxG-XA-m*g*sin(theta).*cos(phi)-XAr;
    YAl = m*ddyG-YA+m*g*sin(theta).*sin(phi)-YAr;
    ZAl = m*ddzG-ZA+m*g*cos(theta)-ZAr;
    
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
        
        % BABAR from r to l
        
        %%
        
        LAl(timestep) = LB(timestep) - LAr(timestep) + (zl-zr)*YAr(timestep) - (yl-yr)*ZAr(timestep);
        MAl(timestep) = MB(timestep) - MAr(timestep) + (xl-xr)*ZAr(timestep) - (zl-zr)*XAr(timestep);
        NAl(timestep) = NB(timestep) - NAr(timestep) + (yl-yr)*XAr(timestep) - (xl-xr)*YAr(timestep);
    end
end