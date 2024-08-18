%%% Force plate data extraction %%%
%% Getting the data
Seq = Tb.Seq;
Fx1 = Tb.x1_Fx;
Fy1 = Tb.x1_Fy;
Fz1 = Tb.x1_Fz;
Fx2 = Tb.x2_Fx;
Fy2 = Tb.x2_Fy;
Fz2 = Tb.x2_Fz;
Mx1 = Tb.x1_Mx;
My1 = Tb.x1_My;
Mz1 = Tb.x1_Mz;
Mx2 = Tb.x2_Mx;
My2 = Tb.x2_My;
Mz2 = Tb.x2_Mz;
COPx1 = Tb.x1_COPx;
COPy1 = Tb.x1_COPy;
COPx2 = Tb.x2_COPx;
COPy2 = Tb.x2_COPy;

%% Force plate
[Px_mes,Py_mes] = CoP(COPx1,COPy1,COPx2,COPy2,Fz1,Fz2);

%% Data synchro
y = Tb_synchro.t;
x = Tb_synchro.index;
p = polyfit(x,y,1);
p1 = p(1);
p0 = p(2);

t_force_plate = p1*Seq + p0;

t0 = Tb_synchro.t0(1);
n0_fp = find(t_force_plate>t0);
% n0_fp = n0_fp(1);
n0_fp = 1;
%% Check
z_left_foot = body_poses(2,:,3);

% figure,
% subplot(2,2,1)
% plot(Fz2)
% xlabel("index")
% ylabel("Fz1 (N - Left force plate)")
% title("Initial force plate data")
% 
% subplot(2,2,3)
% plot(tss,z_left_foot,'Color','red')
% xlabel("t (s)")
% ylabel("left foot z coordinate (m)")
% title("Initial motion capture data")
% 
% subplot(2,2,[2 4])
% plot(t_force_plate,Fz2)
% hold on
% plot(tss,z_left_foot*1000,'-r')
% xlabel("t (s)")
% title("Synchronized data")
% legend("Fz2 (N)","left foot z (m.10^3)")

%% Single vs double support
no_support_index = find(Fz1<2 & Fz2<2);
no_support_time = t_force_plate(no_support_index);
ss_left_index = find(Fz1<2 & Fz2>2);
ss_left_time = t_force_plate(ss_left_index);
ss_right_index = find(Fz2<2 & Fz1>2);
ss_right_time = t_force_plate(ss_right_index);

%% Getting the phase indexes 
phases = [];
name_phases = [];
new_phase = [];
k = 1;
N_fp = numel(Fz2);
th = 5; %N (threashold)
while k<N_fp
    if Fz1(k)<th && Fz2(k)<th
        while Fz1(k)<th && Fz2(k)<th && k<N_fp
           % No support
           new_phase = [new_phase, k];
           k=k+1;
        end
        name_phases = [name_phases, "ns"];
        phases = [phases, 0, new_phase];
        new_phase = [];
    elseif Fz1(k)>th && Fz2(k)<th
        while Fz1(k)>th && Fz2(k)<th && k<N_fp
           % Single support left
           new_phase = [new_phase, k];
           k=k+1;
        end
        name_phases = [name_phases, "ssr"];
        phases = [phases, 0, new_phase];
        new_phase = [];
    elseif Fz1(k)<th && Fz2(k)>th
        while Fz1(k)<th && Fz2(k)>th && k<N_fp
           % Single support right
           new_phase = [new_phase, k];
           k=k+1;
        end
        name_phases = [name_phases, "ssl"];
        phases = [phases, 0, new_phase];
        new_phase = [];
    else
        while Fz1(k)>th && Fz2(k)>th && k<N_fp
           % Double support
           new_phase = [new_phase, k];
           k=k+1;
        end
        name_phases = [name_phases, "ds"];
        phases = [phases, 0, new_phase];
        new_phase = [];
    end
    k = k+1;
end
phases = [phases, 0];

%% Getting the start and end phase indexes
start_end_phases = zeros(numel(name_phases),2);
start_end_phases(1,1) = 1;
i = 1;
for k=2:numel(phases)
    if phases(k) == 0
        start_end_phases(i,2) = k;
        i=i+1;
        start_end_phases(i,1) = k+1;
    end
end
start_end_phases = start_end_phases(1:end-1,:);
start_end_phases(end,2) = start_end_phases(end,2)-1;

