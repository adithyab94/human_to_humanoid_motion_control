%% CoM and CoP
[xg,yg,zg] = CoM(mi,xg_list,body_joints,joint_poses,body_xi,N);
[Px, Py] = LIP_model(xg,yg,zg,tss,t0);