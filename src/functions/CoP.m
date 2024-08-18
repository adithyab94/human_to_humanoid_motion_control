function [Px_mes,Py_mes] = CoP(COPx1,COPy1,COPx2,COPy2,Fz1,Fz2)
    l = 40e-2; %force plate width (x coordinate) (m)
    L = 60e-2; %force plate length (y coordinate) (m)
    x1 = l/2 + COPx1;
    y1 = L/2 + COPy1;
    x2 = 3*l/2 + COPx2;
    y2 = L/2 + COPy2;
%     x1 = COPx1;
%     y1 = COPy1;
%     x2 = l + COPx2;
%     y2 = COPy2;

    Px_mes = (x1.*Fz1+x2.*Fz2)./(Fz1+Fz2);
    Py_mes = (y1.*Fz1+y2.*Fz2)./(Fz1+Fz2);
end