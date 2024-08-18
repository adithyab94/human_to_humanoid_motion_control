%% Help to synchronize
test = body_poses(1,:,:);
z_right_foot = test(1,:,3);
test = body_poses(2,:,:);
z_left_foot = test(1,:,3);
figure,
plot(tss,z_right_foot)
hold on
plot(tss,z_left_foot)
legend("right foot","left foot")

figure,
plot(Seq,Fz1)
hold on
plot(Seq,Fz2)
legend("Fz1","Fz2")
