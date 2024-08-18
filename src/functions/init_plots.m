%% Initializing plot limits
after_synchro = find(tss>t0);
after_synchro_fp = find(t_force_plate>t0);
stopping_fp = find(t_force_plate>max(tss));

start = after_synchro(1);
start_fp = after_synchro_fp(1);  % -> uncomment those 5 lines if you want the plots to start after the synchronization (from t = t0)
end_fp = stopping_fp(1);
xmin = t0;
xmax = tss(end);


% start = 1;
% start_fp = 1; % -> uncomment those 5 lines if you want the plots to start from the beginning
% end_fp = N_fp;
% xmin = t_force_plate(1);
% xmax = t_force_plate(end);



a = 1.2;
ymaxFz = max(max(Fz1(start_fp:end_fp)),max(Fz2(start_fp:end_fp)));
% ymaxFz = max(max(ymaxFz,Fz1_model(start:end)),max(Fz2_model(start:end)))*a;

yminFz = min(min(Fz1(start_fp:end_fp)),min(Fz2(start_fp:end_fp)));
% yminFz = min(min(yminFz,Fz1_model(start:end)),min(Fz2_model(start:end)))*a;

ymaxFx = max(max(Fx1(start_fp:end_fp)),max(Fx2(start_fp:end_fp)))*a;
yminFx = min(min(Fx1(start_fp:end_fp)),min(Fx2(start_fp:end_fp)))*a;

ymaxFy = max(max(Fy1(start_fp:end_fp)),max(Fy2(start_fp:end_fp)));
ymaxFy = max(max(ymaxFy,Fy1_model(start:end)),max(Fy2_model(start:end)))*a;

yminFy = min(min(Fy1(start_fp:end_fp)),min(Fy2(start_fp:end_fp)));
yminFy = min(min(yminFy,Fy1_model(start:end)),min(Fy2_model(start:end)))*a;



% start = 1;
% start_fp = find(t_force_plate>0);
% start_fp = start_fp(1);
% end_fp = after_synchro_fp(1);
% xmin = 1;
% xmax = t0;
