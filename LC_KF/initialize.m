function [outputArg1,outputArg2] = initialize(ins, imu, gnss)
%INITIALIZE : Initialize Loosely Coupled Error State Kalman Filter (Closed)

kf.x = [gnss.p', gnss.v', zeros(1,3)', ins.ab_dyn', ins.gb_dyn', zeros(1,3)]';
kf.P = diag([gnss.stdp, gnss.stdv, imu.stdalign, imu.stda, imu.stdw, veh.stdlg].^2);

kf.R = diag([gnss.stdp, gnss.stdm].^2);
kf.Q = diag([imu.stda, imu.stdw, imu.stdab, imu.stdwb]);
end

