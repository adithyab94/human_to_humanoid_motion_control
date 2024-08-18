%% Energy + power
figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,1,1)
plot(tss(start:end),E(start:end))
hold on
plot(tss(start:end),T(start:end))
plot(tss(start:end),U(start:end))
legend("T+U","T (kinetic energy)","U (potential energy)",'AutoUpdate','off')
xlabel("t (s)")
ylabel("Energy (J)")
title("Mechanical energy of Robin along the time")
xlim([tss(start),tss(end)])
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

subplot(2,1,2)
plot(tss(start:end-1),power(start:end))
xlabel("t (s)")
ylabel("Power (W)")
title("Mechanical power of Robin along the time")
xlim([tss(start),tss(end)])
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
