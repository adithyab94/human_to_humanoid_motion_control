function [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta_tot, phi_tot, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi)
    % This function provides the needed motion computation to compute the
    % efforts from the dynamic model
    
    % Prepare for filtering
    span = 20;
    
    N = numel(tss);
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
    dtss = diff(tss);
    
    l=l_list(body);
    xg=xg_list(body);
    psi = zeros(1,N);
    theta = zeros(1,N);
    phi = zeros(1,N);
    i_A = body_joints(body,1);
    i_B = body_joints(body,2);
%         xB = joint_poses(i_B,:,1);
%         yB = joint_poses(i_B,:,2);
%         zB = joint_poses(i_B,:,3);
    xA = joint_poses(i_A,:,1);
    yA = joint_poses(i_A,:,2);
    zA = joint_poses(i_A,:,3);
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
    
    for k = 1:N
        R = get_R(body,k,body_xi,body_yi,body_zi);
        eul_angle = rotm2eul(R,'ZYZ');
        psi(k) = eul_angle(1);
        theta(k) = eul_angle(2);
        phi(k) = eul_angle(3);

        theta_tot(k) = theta(k);
        phi_tot(k) = phi(k);
    end
    
    psi = unwrap(psi);
    theta = unwrap(theta);
    phi = unwrap(phi);
    
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
        R = get_R(body,k,body_xi,body_yi,body_zi);
        
        %% Velocities
        omega_x(k) =  dtheta(k)*sin(phi(k))-dpsi(k)*sin(theta(k))*cos(phi(k));
        omega_y(k) = dtheta(k)*cos(phi(k))+dpsi(k)*sin(theta(k))*sin(phi(k));
        omega_z(k) = dphi(k) + dpsi(k)*cos(theta(k));
        
        dOPA = [dxA(k) dyA(k) dzA(k)]';
        diPB = R'*dOPA + l*[0;omega_z(k);-omega_y(k)]; %%%% GROS CACAAA => R = cste
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
        R = get_R(body,k,body_xi,body_yi,body_zi);
        
        domega_x(k) = ddtheta(k)*sin(phi(k))+dtheta(k)*dphi(k)*cos(phi(k))-ddpsi(k)*sin(theta(k))*cos(phi(k))-dpsi(k)*dtheta(k)*cos(theta(k))*cos(phi(k))+dpsi(k)*dphi(k)*sin(theta(k))*sin(phi(k));
        domega_y(k) = ddtheta(k)*cos(phi(k))-dtheta(k)*dphi(k)*sin(phi(k))+ddpsi(k)*sin(theta(k))*sin(phi(k))+dpsi(k)*dtheta(k)*cos(theta(k))*sin(phi(k))+dpsi(k)*dphi(k)*sin(theta(k))*cos(phi(k));
        domega_z(k) = ddphi(k)+ddpsi(k)*cos(theta(k))-dpsi(k)*dtheta(k)*sin(theta(k));
        
        ddOPA = [ddxA(k) ddyA(k) ddzA(k)]';
        ddiPB = R'*ddOPA + l*[-omega_z(k)^2-omega_y(k)^2;domega_z(k)+omega_x(k)*omega_y(k);-domega_y(k)+omega_x(k)*omega_z(k)];
        ddiPG = R'*ddOPA + xg*[-omega_z(k)^2-omega_y(k)^2;domega_z(k)+omega_x(k)*omega_y(k);-domega_y(k)+omega_x(k)*omega_z(k)]; %%% GROS CACAAA R = cste

        ddxB(k) = ddiPB(1);
        ddyB(k) = ddiPB(2);
        ddzB(k) = ddiPB(3);

        ddxG(k) = ddiPG(1);
        ddyG(k) = ddiPG(2);
        ddzG(k) = ddiPG(3); 
    end
    

%         dxB(:) = diff(xB)./dtss;
%         dyB(:) = diff(yB)./dtss;
%         dzB(:) = diff(zB)./dtss;

%     ddyB(1:end-1) = diff(dyB)./dtss;
%     ddzB(1:end-1) = diff(dzB)./dtss;
    
    % Filtering
%     ddyB = filter(b,a,ddyB);
%     ddzB = filter(b,a,ddzB);

%         dxG = diff(OGi(:,1))./dtss;
%         dyG = diff(OGi(:,2))./dtss;
%         dzG = diff(OGi(:,3))./dtss;
    
%     ddxG(1:end-1) = diff(dxG)./dtss;
%     ddyG(1:end-1) = diff(dyG)./dtss;
%     ddzG(1:end-1) = diff(dzG)./dtss;
%     
%     % Filtering
%     ddxG = filter(b,a,ddxG);
%     ddyG = filter(b,a,ddyG);
%     ddzG = filter(b,a,ddzG);
    
%     % "c" stands for "cut" -> without the last(s) data 
%     psic = psi(1:end-2);
%     dpsic = dpsi(1:end-1);
%     thetac = theta(1:end-2);
%     dthetac = dtheta(1:end-1);
%     phic = phi(1:end-2);
%     dphic = dphi(1:end-1);
% 
%     domega_x(1:end-2) =  ddtheta.*cos(phic)-dthetac.*dphic.*sin(phic)+ddpsi.*sin(thetac).*cos(phic)+dpsic.*(dthetac.*cos(thetac).*cos(phic)-dpsic.*sin(thetac).*sin(phic));
%     domega_y(1:end-2) = -ddtheta.*sin(phic)-dthetac.*dphic.*cos(phic)-ddpsi.*sin(thetac).*sin(phic)-dpsic.*(dthetac.*cos(thetac).*sin(phic)+dpsic.*sin(thetac).*cos(phic));
%     domega_z(1:end-2) = ddphi+ddpsi.*cos(thetac)-dpsic.*dthetac.*sin(thetac);
    
end