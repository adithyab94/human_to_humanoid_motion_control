function [xg,yg,zg] = CoM(mi,xg_list,body_joints,joint_poses,body_xi,N)
    Pg = zeros(3,N); %CoM position
    mtot = sum(mi(mi>0));
    for timestep = 1:N
        for body=1:17
            if body ~= 13 && body ~= 14 %not the shoulders
                i = body_joints(body,1);
                pose = joint_poses(i,timestep,:);
                X = pose(:,:,1);
                Y = pose(:,:,2);
                Z = pose(:,:,3); 
                xi = body_xi(body,timestep,:);

                xgi = xg_list(body);
                m = mi(body);
                Pg(:,timestep) = Pg(:,timestep) + m*[X+xgi*xi(1),Y+xgi*xi(2),Z+xgi*xi(3)]';
        %         plot3(X+xg*xi(1),Y+xg*xi(2),Z+xg*xi(3),'.r')
            end
        end
        Pg(:,timestep) = Pg(:,timestep)/mtot;
    end
    
    xg = Pg(1,:);
    yg = Pg(2,:);
    zg = Pg(3,:);
end