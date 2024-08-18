%% Feet forces model

z = zeros(1,N);
[Fx1_model,Fy1_model,Fz1_model,Mx1_model,My1_model,Mz1_model,Fx2_model,Fy2_model,Fz2_model,Mx2_model,My2_model,Mz2_model] = deal(z,z,z,z,z,z,z,z,z,z,z,z);
for k=1:numel(name_phases)
    % In the forceplate datasets
    x_start = start_end_phases(k,1);
    x_end = start_end_phases(k,2);
    % Transposing in the motion capture dataset
    tstart_fp = t_force_plate(x_start);
    if tstart_fp>0  % avoid points when the motion capture is not launched
        start = find(tss>tstart_fp);
        start = start(1);
        tend_fp = t_force_plate(x_end);
        if tend_fp < tss(end)
            end_n =  find(tss<tend_fp);
            end_n = end_n(end);
            if name_phases(k) == "ssl"
               [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = reaction_force_ssl(body_xi(:,start:end_n,:),body_yi(:,start:end_n,:),body_zi(:,start:end_n,:),joint_poses(:,start:end_n,:),joint_rot_matrices(:,start:end_n,:),body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss(start:end_n));
            elseif name_phases(k) == "ds"
               [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = reaction_force_ds(body_xi(:,start:end_n,:),body_yi(:,start:end_n,:),body_zi(:,start:end_n,:),joint_poses(:,start:end_n,:),joint_rot_matrices(:,start:end_n,:),body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss(start:end_n));
            elseif name_phases(k) == "ssr"
               [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = reaction_force_ssr(body_xi(:,start:end_n,:),body_yi(:,start:end_n,:),body_zi(:,start:end_n,:),joint_poses(:,start:end_n,:),joint_rot_matrices(:,start:end_n,:),body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss(start:end_n));
            else %double support -> no efforts
                z = zeros(1,end_n-start+1);
                [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = deal(z,z,z,z,z,z,z,z,z,z,z,z);
            end
            Fx1_model(start:end_n)=Fx1_m;
            Fy1_model(start:end_n)=Fy1_m;
            Fz1_model(start:end_n)=Fz1_m;
            Fx2_model(start:end_n)=Fx2_m;
            Fy2_model(start:end_n)=Fy2_m;
            Fz2_model(start:end_n)=Fz2_m;
            Mx1_model(start:end_n)=Mx1_m;
            My1_model(start:end_n)=My1_m;
            Mz1_model(start:end_n)=Mz1_m;
            Mx2_model(start:end_n)=Mx2_m;
            My2_model(start:end_n)=My2_m;
            Mz2_model(start:end_n)=Mz2_m;
        else
            end_n = N;
            if name_phases(k) == "ssl"
               [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = reaction_force_ssl(body_xi(:,start:end_n,:),body_yi(:,start:end_n,:),body_zi(:,start:end_n,:),joint_poses(:,start:end_n,:),joint_rot_matrices(:,start:end_n,:),body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss(start:end_n));
            elseif name_phases(k) == "ds"
               [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = reaction_force_ds(body_xi(:,start:end_n,:),body_yi(:,start:end_n,:),body_zi(:,start:end_n,:),joint_poses(:,start:end_n,:),joint_rot_matrices(:,start:end_n,:),body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss(start:end_n));
            elseif name_phases(k) == "ssr"
               [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = reaction_force_ssr(body_xi(:,start:end_n,:),body_yi(:,start:end_n,:),body_zi(:,start:end_n,:),joint_poses(:,start:end_n,:),joint_rot_matrices(:,start:end_n,:),body_joints,l_list,xg_list,xxi,yyi,zzi,mi,tss(start:end_n));
            else %double support -> no efforts
                z = zeros(1,end_n-start+1);
                [Fx1_m,Fy1_m,Fz1_m,Mx1_m,My1_m,Mz1_m,Fx2_m,Fy2_m,Fz2_m,Mx2_m,My2_m,Mz2_m] = deal(z,z,z,z,z,z,z,z,z,z,z,z);
            end
            Fx1_model(start:end_n)=Fx1_m;
            Fy1_model(start:end_n)=Fy1_m;
            Fz1_model(start:end_n)=Fz1_m;
            Fx2_model(start:end_n)=Fx2_m;
            Fy2_model(start:end_n)=Fy2_m;
            Fz2_model(start:end_n)=Fz2_m;
            Mx1_model(start:end_n)=Mx1_m;
            My1_model(start:end_n)=My1_m;
            Mz1_model(start:end_n)=Mz1_m;
            Mx2_model(start:end_n)=Mx2_m;
            My2_model(start:end_n)=My2_m;
            Mz2_model(start:end_n)=Mz2_m;
            break
        end
    end
end