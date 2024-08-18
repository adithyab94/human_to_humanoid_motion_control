%% Feet moment plots
figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,2,1)
plot(t_force_plate(start_fp:end_fp),Mx1(start_fp:end_fp))
hold on
plot(tss(start:end),Mx1_model(start:end))
legend("Mx1","Mx1_{model}",'AutoUpdate','off')
xlabel("t(s)")
ylabel("Moment (Nm)")
xlim([xmin,xmax])
y=get(gca,'Ylim');
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
plot(t_force_plate(start_fp:end_fp),Mx2(start_fp:end_fp))
hold on
plot(tss(start:end),Mx2_model(start:end))
legend("Mx2","Mx2_{model}",'AutoUpdate','off')
xlabel("t(s)")
ylabel("Moment (Nm)")
xlim([xmin,xmax])
y=get(gca,'Ylim');
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
plot(t_force_plate(start_fp:end_fp),My1(start_fp:end_fp))
hold on
plot(tss(start:end),My1_model(start:end))
legend("My1","My1_{model}",'AutoUpdate','off')
xlabel("t(s)")
ylabel("Moment (Nm)")
xlim([xmin,xmax])
% ylim([yminMy,ymaxMy])
y=get(gca,'Ylim');
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
plot(t_force_plate(start_fp:end_fp),My2(start_fp:end_fp))
hold on
plot(tss(start:end),My2_model(start:end))
legend("My2","My2_{model}",'AutoUpdate','off')
xlabel("t(s)")
ylabel("Moment (Nm)")
xlim([xmin,xmax])
% ylim([yminMy,ymaxMy])
y=get(gca,'Ylim');
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
plot(t_force_plate(start_fp:end_fp),Mz1(start_fp:end_fp))
hold on
plot(tss(start:end),Mz1_model(start:end))
legend("Mz1","Mz1_{model}",'AutoUpdate','off')
xlabel("t(s)")
ylabel("Moment (Nm)")
xlim([xmin,xmax])
% ylim([yminMz,ymaxMz])
y=get(gca,'Ylim');
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
plot(t_force_plate(start_fp:end_fp),Mz2(start_fp:end_fp))
hold on
plot(tss(start:end),Mz2_model(start:end))
legend("Mz2","Mz2_{model}",'AutoUpdate','off')
xlabel("t(s)")
ylabel("Moment (Nm)")
xlim([xmin,xmax])
% ylim([yminMz,ymaxMz])
y=get(gca,'Ylim');
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