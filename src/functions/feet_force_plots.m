%% Feet force plots
figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,2,1)
plot(t_force_plate(start_fp:end_fp),Fx1(start_fp:end_fp))
hold on
plot(tss(start:end),Fx1_model(start:end))
legend("Fx1","Fx1_{model}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Effort (N)")
xlim([xmin,xmax])
y=get(gca,'YLim');
for k=1:numel(name_phases)
    x_start = start_end_phases(k,1);
    x_start = t_force_plate(x_start);
    x_end = start_end_phases(k,2);
    x_end = t_force_plate(x_end);
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"n")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"l")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"r")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"d")
    end
end
ylim(y)

subplot(3,2,2)
plot(t_force_plate(start_fp:end_fp),Fx2(start_fp:end_fp))
hold on
plot(tss(start:end),Fx2_model(start:end))
legend("Fx2","Fx2_{model}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Effort (N)")
xlim([xmin,xmax])
y=get(gca,'YLim');
for k=1:numel(name_phases)
    x_start = start_end_phases(k,1);
    x_start = t_force_plate(x_start);
    x_end = start_end_phases(k,2);
    x_end = t_force_plate(x_end);
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"n")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"l")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"r")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"d")
    end
end
ylim(y)

subplot(3,2,3)
plot(t_force_plate(start_fp:end_fp),Fy1(start_fp:end_fp))
hold on
plot(tss(start:end),Fy1_model(start:end))
legend("Fy1","Fy1_{model}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Effort (N)")
xlim([xmin,xmax])
% ylim([yminFy,ymaxFy])
y=get(gca,'YLim');
for k=1:numel(name_phases)
    x_start = start_end_phases(k,1);
    x_start = t_force_plate(x_start);
    x_end = start_end_phases(k,2);
    x_end = t_force_plate(x_end);
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"n")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"l")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"r")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"d")
    end
end
ylim(y)

subplot(3,2,4)
plot(t_force_plate(start_fp:end_fp),Fy2(start_fp:end_fp))
hold on
plot(tss(start:end),Fy2_model(start:end))
legend("Fy2","Fy2_{model}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Effort (N)")
xlim([xmin,xmax])
% ylim([yminFy,ymaxFy])
y=get(gca,'YLim');
for k=1:numel(name_phases)
    x_start = start_end_phases(k,1);
    x_start = t_force_plate(x_start);
    x_end = start_end_phases(k,2);
    x_end = t_force_plate(x_end);
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"n")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"l")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"r")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"d")
    end
end
ylim(y)

subplot(3,2,5)
plot(t_force_plate(start_fp:end_fp),Fz1(start_fp:end_fp))
hold on
plot(tss(start:end),Fz1_model(start:end))
legend("Fz1","Fz1_{model}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Effort (N)")
xlim([xmin,xmax])
% ylim([yminFz,ymaxFz])
y=get(gca,'YLim');
for k=1:numel(name_phases)
    x_start = start_end_phases(k,1);
    x_start = t_force_plate(x_start);
    x_end = start_end_phases(k,2);
    x_end = t_force_plate(x_end);
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"n")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"l")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"r")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"d")
    end
end
ylim(y)

subplot(3,2,6)
plot(t_force_plate(start_fp:end_fp),Fz2(start_fp:end_fp))
hold on
plot(tss(start:end),Fz2_model(start:end))
legend("Fz2","Fz2_{model}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Effort (N)")
xlim([xmin,xmax])
% ylim([yminFz,ymaxFz])
y=get(gca,'YLim');
for k=1:numel(name_phases)
    x_start = start_end_phases(k,1);
    x_start = t_force_plate(x_start);
    x_end = start_end_phases(k,2);
    x_end = t_force_plate(x_end);
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"n")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"l")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"r")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
%         x_text = (x_start+x_end)/2;
%         y_text = mean(y);
%         text(x_text,y_text,"d")
    end
end
ylim(y)

sgtitle(global_title)