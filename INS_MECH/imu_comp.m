function [a, w] = imu_comp(ar, wr, ab_dyn, wb_dyn, CTMbn)
%IMU_COMP Summary of this function goes here
%   Detailed explanation goes here
a = ar - ab_dyn;
w = wr - wb_dyn;

end

