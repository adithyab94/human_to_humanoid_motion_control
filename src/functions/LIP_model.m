function [Px, Py] = LIP_model(xg,yg,zg,tss,t0)
    g=9.81;
    span = 20;
    N = numel(tss);
    dtss = diff(tss);
%     offset_y = 0.1275;
    
    n0 = find(tss>t0);
    n0 = n0(1);
    z = mean(zg(n0:end)); % Ignoring the syncho motion
    xg = smooth(xg,span)';
    dxg = diff(xg)./dtss;
    dxg = smooth(dxg,span)';
    ddxg = diff(dxg)./dtss(1:end-1);
    ddxg = smooth(ddxg,span)';
    yg = smooth(yg,span)';
    dyg = diff(yg)./dtss;
    dyg = smooth(dyg,span)';
    ddyg = diff(dyg)./dtss(1:end-1);
    ddyg = smooth(ddyg,span)';
    
    Px = xg(3:end)-z/g.*ddxg;
    Py = yg(3:end)-z/g.*ddyg;
end