%% Phase plots (single / double support) 
% ssl = single support left
% ssr = single support right
% ds = double support
% ns = no support (in the air)

figure,
plot(Fz1(n0_fp:end))
hold on
plot(Fz2(n0_fp:end))
y=get(gca,'YLim');
legend("Fz1 (right foot)","Fz2 (left foot)",'AutoUpdate','off')
ylabel("Force (N)")
xlabel("t (s)")
title("Checking the detection of phases")
k0 = find(start_end_phases(:,1)>n0_fp);
k0 = max(1,k0(1)-1);
for k=k0:numel(name_phases)
    x_start = max(1,start_end_phases(k,1)-n0_fp);
    x_end = start_end_phases(k,2)-n0_fp;
    if name_phases(k) == "ns"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','k','FaceAlpha',0.1,'EdgeColor','none')
        x_text = (x_start+x_end)/2;
        y_text = mean(y);
        text(x_text,y_text,"ns")
    elseif name_phases(k) == "ssl"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none')
        x_text = (x_start+x_end)/2;
        y_text = mean(y);
        text(x_text,y_text,"ssl")
    elseif name_phases(k) == "ssr"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','g','FaceAlpha',0.1,'EdgeColor','none')
        x_text = (x_start+x_end)/2;
        y_text = mean(y);
        text(x_text,y_text,"ssr")
    elseif name_phases(k) == "ds"
        patch('XData',[x_start x_start x_end x_end],'YData',[y fliplr(y)],'FaceColor','b','FaceAlpha',0.1,'EdgeColor','none')
        x_text = (x_start+x_end)/2;
        y_text = mean(y);
        text(x_text,y_text,"ds")
    end
end