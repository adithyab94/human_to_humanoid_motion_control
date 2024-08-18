%% Recup right elbow angle
span = 20;
body1 = find(body_ids == "right lower arm");
body2 = find(body_ids == "right upper arm");
psi_elbow = zeros(1,N);
theta_elbow = zeros(1,N);
phi_elbow = zeros(1,N);
axis = zeros(3,N);
angle = zeros(1,N);
for k = 1:N
    R1 = get_R(body1,k,body_xi,body_yi,body_zi);
    R2 = get_R(body2,k,body_xi,body_yi,body_zi);
    R_tot = R1'*R2;
    eul_angle = rotm2eul(R_tot,'ZYZ');
    psi_elbow(k) = eul_angle(1);
    theta_elbow(k) = eul_angle(2);
    phi_elbow(k) = eul_angle(3);
    axangl = rotm2axang(R_tot);
    axis(:,k) = axangl(1:3);
    angle(k) = axangl(4);
end
psi_elbow = unwrap(psi_elbow);
theta_elbow = unwrap(theta_elbow);
phi_elbow = unwrap(phi_elbow);

dtss = diff(tss);

dpsi_elbow = diff(psi_elbow)./dtss;
dpsi_elbow = smooth(dpsi_elbow,span)';
dtheta_elbow = diff(theta_elbow)./dtss;
dtheta_elbow = smooth(dtheta_elbow,span)';
dphi_elbow = diff(phi_elbow)./dtss;
dphi_elbow = smooth(dphi_elbow,span)';

ddpsi_elbow = diff(dpsi_elbow)./dtss(1:end-1);
ddpsi_elbow = smooth(ddpsi_elbow,span)';
ddtheta_elbow = diff(dtheta_elbow)./dtss(1:end-1);
ddtheta_elbow = smooth(ddtheta_elbow,span)';
ddphi_elbow = diff(dphi_elbow)./dtss(1:end-1);
ddphi_elbow = smooth(ddphi_elbow,span)';

dangle = diff(angle)./dtss;
dangle = smooth(dangle,span)';

ddangle = diff(dangle)./dtss(1:end-1);
ddangle = smooth(ddangle,span)';

% figure,
% plot(tss(start:end),psi_elbow(start:end))
% hold on
% plot(tss(start:end),theta_elbow(start:end))
% plot(tss(start:end),phi_elbow(start:end))
% legend("\psi","\theta","\phi")
% xlabel("t (s)")
% ylabel("angles (rad)")
% title(global_title + " - Right elbow relative angles")
% 
% figure,
% plot(tss(start:end-1),dpsi_elbow(start:end))
% hold on
% plot(tss(start:end-1),dtheta_elbow(start:end))
% plot(tss(start:end-1),dphi_elbow(start:end))
% legend("$\dot{\psi}$","$\dot{\theta}$","$\dot{\phi}$",'interpreter','latex')
% xlabel("t (s)")
% ylabel("angle velocities (rad/s)")
% title(global_title + " - Right elbow relative angles velocities")
% 
% figure,
% plot(tss(start:end-2),ddpsi_elbow(start:end))
% hold on
% plot(tss(start:end-2),ddtheta_elbow(start:end))
% plot(tss(start:end-2),ddphi_elbow(start:end))
% legend("$\ddot{\psi}$","$\ddot{\theta}$","$\ddot{\phi}$",'interpreter','latex')
% xlabel("t (s)")
% ylabel("angle accelerations (rad/s^2)")
% title(global_title + " - Right elbow relative angles accelerations")

figure,
plot(tss(start:end),angle(start:end))
xlabel("t (s)")
ylabel("angle (rad)")
title(global_title + " - Right elbow relative angle")

figure,
plot(tss(start:end-1),dangle(start:end))
xlabel("t (s)")
ylabel("angle velocity (rad/s)")
title(global_title + " - Right elbow relative angle velocity")

figure,
plot(tss(start:end-2),ddangle(start:end))
xlabel("t (s)")
ylabel("angle acceleration (rad/s^2)")
title(global_title + " - Right elbow relative angle acceleration")

% x_rot = axis(1,:);
% y_rot = axis(2,:);
% z_rot = axis(3,:);
% figure,
% plot(tss(start:end),x_rot(start:end))
% hold on
% plot(tss(start:end),y_rot(start:end))
% plot(tss(start:end),z_rot(start:end))
% legend("x","y","z")