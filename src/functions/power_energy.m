%% Energy (kinetic + potential) + power of Robin body along the time
T = kine_ener(joint_poses,joint_rot_matrices,body_joints,l_list,xg_list,tss,xxi,yyi,zzi,mi,body_xi,body_yi,body_zi);
mtot = sum(mi(mi>0));
g = 9.81;
U = mtot*g*(zg-zg(1));
E = T+U;
power = diff(E)./diff(tss);
power = smooth(power,10);