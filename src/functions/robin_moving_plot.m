%% Test Plot
figure,
dt = tss(2)-tss(1);
after_synchro = find(tss>t0);
% timestep = after_synchro(1); % uncomment this if you want the plot to start from t0
timestep = 1;               % uncomment this if you want the plot to start from t = 0
while timestep < N
    clf
    tic,
    for i=1:20
        pose = joint_poses(i,timestep,:);
        X = pose(:,:,1);
        Y = pose(:,:,2);
        Z = pose(:,:,3);
        plot3(X,Y,Z,'.')
        
        hold on
        
        rot_mat = joint_rot_matrices(i,timestep,:);
        r11 = rot_mat(:,:,1);
        r12 = rot_mat(:,:,2);
        r13 = rot_mat(:,:,3);
        r21 = rot_mat(:,:,4);
        r22 = rot_mat(:,:,5);
        r23 = rot_mat(:,:,6);
        r31 = rot_mat(:,:,7);
        r32 = rot_mat(:,:,8);
        r33 = rot_mat(:,:,9);
        R = [r11 r12 r13;r21 r22 r23;r31 r32 r33];
        
        xi = R*[1 0 0]';
        yi = R*[0 1 0]';
        zi = R*[0 0 1]';
%         scale = 100;$
%         plot3([X,X+scale*xi(1)],[Y,Y+scale*xi(2)],[Z,Z+scale*xi(3)],'r-')
%         plot3([X,X+scale*yi(1)],[Y,Y+scale*yi(2)],[Z,Z+scale*yi(3)],'g-')
%         plot3([X,X+scale*zi(1)],[Y,Y+scale*zi(2)],[Z,Z+scale*zi(3)],'b-')
    end
    for i=1:n_joint_links
        body1 = joint_links(i,1);
        body2 = joint_links(i,2);
        pose1 = joint_poses(body1,timestep,:);
        X1 = pose1(:,:,1);
        Y1 = pose1(:,:,2);
        Z1 = pose1(:,:,3);
        pose2 = joint_poses(body2,timestep,:);
        X2 = pose2(:,:,1);
        Y2 = pose2(:,:,2);
        Z2 = pose2(:,:,3);
        plot3([X1 X2],[Y1 Y2],[Z1 Z2],'-k')
    end
    for body = 1:17
        
        i = body_joints(body,1);
        pose = joint_poses(i,timestep,:);
        X = pose(:,:,1);
        Y = pose(:,:,2);
        Z = pose(:,:,3);
        xi = body_xi(body,timestep,:);
        yi = body_yi(body,timestep,:);
        zi = body_zi(body,timestep,:);
        scale = 50e-3;
        plot3([X,X+scale*xi(1)],[Y,Y+scale*xi(2)],[Z,Z+scale*xi(3)],'r-')
        plot3([X,X+scale*yi(1)],[Y,Y+scale*yi(2)],[Z,Z+scale*yi(3)],'g-')
        plot3([X,X+scale*zi(1)],[Y,Y+scale*zi(2)],[Z,Z+scale*zi(3)],'b-')
        
        xgi = xg_list(body);
        plot3(X+xgi*xi(1),Y+xgi*xi(2),Z+xgi*xi(3),'.r')
        
    end
    plot3(xg(timestep),yg(timestep),zg(timestep),'k+')
    plot3(Px(timestep),Py(timestep),0,'k+')
    
%     fp_timestep = find(t_force_plate>tss(timestep));
%     fp_timestep = fp_timestep(1);
%     plot3(Px_mes(fp_timestep),Py_mes(fp_timestep),0,'r+')
    
    axis equal
    xlim([-0.6,1])
    ylim([-1,1])
    zlim([-0.1,2])
%     view(0, 0)
    
    grid on
    title("t = " + num2str(round(tss(timestep),2))+ " s");
    xlabel("x (m)")
    ylabel("y (m)")
    zlabel("z (m)")
    drawnow
    
    t_end = toc;
    timestep = timestep + round(t_end/dt);
end
