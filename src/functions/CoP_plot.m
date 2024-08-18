%% Center of pressure
figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,1)
plot(t_force_plate(start_fp:end_fp),Px_mes(start_fp:end_fp))
hold on
plot(tss(start:end-2),Px(start:end))
legend("Px_{mes}","Px_{mod}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("x (m)")
title('X coordinate')
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

subplot(2,2,3)
plot(t_force_plate(start_fp:end_fp),Py_mes(start_fp:end_fp))
hold on
plot(tss(start:end-2),Py(start:end))
legend("Py_{mes}","Py_{mod}",'AutoUpdate','off')
xlabel("t (s)")
ylabel("y (m)")
title('Y coordinate')
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

subplot(2,2,[2 4])
plot(Px_mes(start_fp:end_fp),Py_mes(start_fp:end_fp))
hold on
plot(Px(start:end),Py(start:end))
legend("P_{mes}","P_{mod}",'AutoUpdate','off')
xlabel("x (m)")
ylabel("y (m)")
title('XY plot')

% Py_c = interp1(t_force_plate(start_fp:end_fp-2),Py_mes(start_fp:end_fp-2),tss(start:end-2));
% figure,
% plot(tss(start:end-2),Py(start:end))
% hold on
% plot(tss(start:end-2),Py_c)
% plot(tss(start:end-2),Py_c-Py(start:end))