function kf = correct(kf)
%PREDICT Summary of this function goes here
%   Detailed explanation goes here
kf.S = (kf.R + kf.H*kf.P_*kf.H');
kf.err = kf.z - kf.H*kf.x_;
kf.K = kf.P_*kf.H'/kf.S;

kf.x = kf.x_ + kf.K*kf.err;
kf.P = kf.P_ - kf.K*kf.S*kf.K';
kf.P = (kf.P+kf.P')/2;
end

