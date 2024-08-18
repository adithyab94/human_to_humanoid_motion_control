function [Fx1,Fy1,Fz1,Mx1,My1,Mz1,Fx2,Fy2,Fz2,Mx2,My2,Mz2] = reaction_force_ssr(body_xi,body_yi,body_zi,joint_poses,joint_rot_matrices,body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss)
    body_ids = ["right foot", "left foot", "right upper leg", "left upper leg", ...
            "right lower leg", "left lower arm", "left lower leg", "left upper arm", ...
            "hip", "right hand", "right upper arm", "right lower arm", "left shoulder", "right shoulder",...
            "left hand", "chest", "head"];
    N = numel(tss);
    
    %% Left hand to chest
    % Left hand to left lower arm
    body = find(body_ids == "left hand");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    z = zeros(1,N);
    [XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z); % No forces at the end of the left hand
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "left hand");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "left lower arm");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    
    % Left lower arm to left upper arm
    body = find(body_ids == "left lower arm");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:);    
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "left lower arm");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "left upper arm");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    
    % Left upper arm to chest
    body = find(body_ids == "left upper arm");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:);    
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "left upper arm");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "chest");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "Al" of the chest
    XAl = -F_new(1,:);
    YAl = -F_new(2,:);
    ZAl = -F_new(3,:);
    LAl = -M_new(1,:);
    MAl = -M_new(2,:);
    NAl = -M_new(3,:);
    
    %% Right hand to chest
    % Right hand to right lower arm
    body = find(body_ids == "right hand");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    [XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z); % No forces at the end of the right hand
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "right hand");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "right lower arm");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    
    % Right lower arm to right upper arm
    body = find(body_ids == "right lower arm");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:);    
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "right lower arm");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "right upper arm");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    
    % Right upper arm to chest
    body = find(body_ids == "right upper arm");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:);    
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "right upper arm");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "chest");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "Ar" of the chest
    XAr = -F_new(1,:);
    YAr = -F_new(2,:);
    ZAr = -F_new(3,:);
    LAr = -M_new(1,:);
    MAr = -M_new(2,:);
    NAr = -M_new(3,:); 
    
    %% Head to chest
    % Head to chest
    body = find(body_ids == "head");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    [XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z); % No forces at the end of the head
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "head");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "chest");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "A" of the chest
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:); 
    
    %% Chest to hip
    body = find(body_ids == "chest");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    [XB,YB,ZB,LB,MB,NB] = chest_dyna(body,XA,YA,ZA,LA,MA,NA,XAl,YAl,ZAl,LAl,MAl,NAl,XAr,YAr,ZAr,LAr,MAr,NAr,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N,joint_poses,body_xi,body_yi,body_zi);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "chest");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "hip");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "A" of the hip
    XA_hip = -F_new(1,:);
    YA_hip = -F_new(2,:);
    ZA_hip = -F_new(3,:);
    LA_hip = -M_new(1,:);
    MA_hip = -M_new(2,:);
    NA_hip = -M_new(3,:);
    
    %% Left foot (flying) to hip
    % Left foot to left lower leg
    body = find(body_ids == "left foot");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    z = zeros(1,N);
    [XA,YA,ZA,LA,MA,NA] = deal(z,z,z,z,z,z); % No forces at the end of the left hand
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "left foot");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "left lower leg");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    
    % Left lower leg to left upper leg
    body = find(body_ids == "left lower leg");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:);    
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "left lower leg");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "left upper leg");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    
    % Left upper leg to hip
    body = find(body_ids == "left upper leg");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    XA = -F_new(1,:);
    YA = -F_new(2,:);
    ZA = -F_new(3,:);
    LA = -M_new(1,:);
    MA = -M_new(2,:);
    NA = -M_new(3,:);    
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "left upper leg");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "hip");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "Al" of the hip
    XAl = -F_new(1,:);
    YAl = -F_new(2,:);
    ZAl = -F_new(3,:);
    LAl = -M_new(1,:);
    MAl = -M_new(2,:);
    NAl = -M_new(3,:);
    
    %% Hip to right upper leg
    body = find(body_ids == "hip");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
    [XAr,YAr,ZAr,LAr,MAr,NAr] = hip_dyna_ssr(body,XAl,YAl,ZAl,LAl,MAl,NAl,XA,YA,ZA,LA,MA,NA,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N,joint_poses,body_xi,body_yi,body_zi);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XAr(timestep) YAr(timestep) ZAr(timestep)]';
        M = [LAr(timestep) MAr(timestep) NAr(timestep)]';
        body = find(body_ids == "hip");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "right upper leg");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "A" of the right upper leg
    XAr = -F_new(1,:);
    YAr = -F_new(2,:);
    ZAr = -F_new(3,:);
    LAr = -M_new(1,:);
    MAr = -M_new(2,:);
    NAr = -M_new(3,:);
    
    
    %% Right upper leg to right lower leg
    body = find(body_ids == "right upper leg");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);   
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XAr,YAr,ZAr,LAr,MAr,NAr,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "right upper leg");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "right lower leg");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "A" of the right lower leg
    XAr = -F_new(1,:);
    YAr = -F_new(2,:);
    ZAr = -F_new(3,:);
    LAr = -M_new(1,:);
    MAr = -M_new(2,:);
    NAr = -M_new(3,:);
    
    %% Right lower leg to right foot
    body = find(body_ids == "right lower leg");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);   
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XAr,YAr,ZAr,LAr,MAr,NAr,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the next body
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "right lower leg");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        body = find(body_ids == "right foot");
        R1 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R1'*R0*F;
        M_new(:,timestep) = R1'*R0*M;
    end
    % Forces to provide to the point "A" of the right foot
    XAr = -F_new(1,:);
    YAr = -F_new(2,:);
    ZAr = -F_new(3,:);
    LAr = -M_new(1,:);
    MAr = -M_new(2,:);
    NAr = -M_new(3,:);
    
    %% Right foot to ground
    body = find(body_ids == "right foot");
    [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);   
    [XB,YB,ZB,LB,MB,NB] = two_wrench_dyna(body,XAr,YAr,ZAr,LAr,MAr,NAr,dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z,xxi,yyi,zzi,mi,l_list,xg_list,N);
    % Rotate the forces to give them to the ground
    F_new = zeros(3,N);
    M_new = zeros(3,N);
    for timestep=1:N
        F = [XB(timestep) YB(timestep) ZB(timestep)]';
        M = [LB(timestep) MB(timestep) NB(timestep)]';
        body = find(body_ids == "right foot");
        R0 = get_R(body,timestep,body_xi,body_yi,body_zi);
        F_new(:,timestep) = R0*F;
        M_new(:,timestep) = R0*M;
    end
    % Forces to provide to the right force plate
    Fx1 = F_new(1,:);
    Fy1 = F_new(2,:);
    Fz1 = F_new(3,:);
    Mx1 = M_new(1,:);
    My1 = M_new(2,:);
    Mz1 = M_new(3,:);
    
    Fx2 = z;
    Fy2 = z;
    Fz2 = z;
    Mx2 = z;
    My2 = z;
    Mz2 = z;
    
end