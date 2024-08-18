function T = kine_ener(joint_poses,joint_rot_matrices,body_joints,l_list,xg_list,tss,xxi,yyi,zzi,mi,body_xi,body_yi,body_zi)
    N = numel(tss);
    T = zeros(1,N);
    for body = 1:17
        if body ~= 13 && body ~= 14 %not the shoulders
            xx = xxi(body);
            yy = yyi(body);
            zz = zzi(body);
            m = mi(body);
            l = l_list(body);
            xg = xg_list(body);
            [dxB, dyB, dzB, dxG, dyG, dzG, ddxG, ddyG, ddzG, ddyB, ddzB, theta, phi, omega_x, omega_y, omega_z, domega_x, domega_y, domega_z] = body_motion(joint_poses,joint_rot_matrices,body,body_joints,l_list,xg_list,tss,body_xi,body_yi,body_zi);
            ti = xx*omega_x.^2+(yy*omega_y-m*(xg-l)*dzB).*omega_y+(zz*omega_z+m*(xg-l)*dyB).*omega_z+m*(dxG.*dxB+dyG.*dyB+dzB.*dzG);
            ti = ti/2;
            T = T + ti;
        end
    end
end