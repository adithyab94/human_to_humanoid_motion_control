%% Feet Fz force error plots
figure,

subplot(2,2,1)
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

subplot(2,2,2)
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

Fz1_c = interp1(t_force_plate(start_fp:end_fp),Fz1(start_fp:end_fp),tss(start:end));
rel_err_Fz1 = abs(Fz1_model(start:end)-Fz1_c)./Fz1_c*100;
subplot(2,2,3)
plot(tss(start:end),rel_err_Fz1)
hold on
% legend("error Fz1",'AutoUpdate','off')
ylabel("relative error (%)")
title("Error Fz1")
xlabel("t (s)")
xlim([xmin,xmax])
ylim([-110,400])
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

Fz2_c = interp1(t_force_plate(start_fp:end_fp),Fz2(start_fp:end_fp),tss(start:end));
rel_err_Fz2 = abs(Fz2_model(start:end)-Fz2_c)./Fz2_c*100;
subplot(2,2,4)
plot(tss(start:end),rel_err_Fz2)
hold on
% legend("error Fz2",'AutoUpdate','off')
ylabel("relative error (%)")
title("Error Fz2")
xlabel("t (s)")
xlim([xmin,xmax])
ylim([-110,400])
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