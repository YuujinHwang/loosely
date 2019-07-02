function kf = predict(kf, dt)
%PREDICT Summary of this function goes here
%   Detailed explanation goes here

kf.A = expm(kf.F * dt);
kf.Qd = (kf.G * kf.Q * kf.G')*dt;
kf.x_ = kf.A * kf.x;
kf.P_ = (kf.A * kf.P * kf.A') + kf.Qd;
kf.P_ = (kf.P_+kf.P_')/2;
end

