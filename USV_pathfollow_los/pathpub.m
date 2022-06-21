clear path_circleR10.mat path
for i = 0:157
    Xr = 50 * sin(0.04 * i);
    Yr = 50 * (1-cos(0.04 * i));
    path(i+1,1) = Xr;
    path(i+1,2) = Yr;  
end
save path_circleR50.mat path