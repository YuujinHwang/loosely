function ins = pvt_comp(error_s, ins)
%PVT_COMP Summary of this function goes here
%   Detailed explanation goes here

dp = error_s(1:3);
dv = error_s(4:6);
dr = error_s(7:9);
dab = error_s(10:12);
dwb = error_s(13:15);


%% Attitude Correction
antm = [0 ins.qua(3) -ins.qua(2); -ins.qua(3) 0 ins.qua(1); ins.qua(2) -ins.qua(1) 0];
ins.qua = ins.qua + 0.5 .* [ins.qua(4)*eye(3) + antm; -1.*[ins.qua(1) ins.qua(2) ins.qua(3)]] * dr(1:3);
ins.qua = ins.qua / norm(ins.qua);       % Brute-force normalization

ins.CTMbn = Qua_to_CTM(ins.qua);
ins.r = ins.r - dr;

%% Velocity Correction
% v : velocity NED m/s^2
v_n = ins.v - dv;
ins.v = v_n;
%% Position Correction
% p : Local NAV frame position NED m
p_n = ins.p - dp;
ins.p = p_n;

ins.ab_dyn = ins.ab_dyn + dab;
ins.wb_dyn = ins.wb_dyn + dwb;
end

