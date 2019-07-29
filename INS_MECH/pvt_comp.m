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

% dq = Euler_to_Qua(dr);
% ins.qua = attqua_update(qua, dq);

ins.CTMbn = Qua_to_CTM(ins.qua);
ins.r = ins.r + dr;

%% Velocity Correction
% v : velocity NED m/s^2
v_n = ins.v - dv;
ins.v = v_n;
%% Position Correction
% p : Local NAV frame position NED m
p_n = ins.p - dp;
ins.p = p_n;
ins.va = ins.CTMbn'*ins.v;

ins.ab_dyn = ins.ab_dyn + dab;
ins.wb_dyn = ins.wb_dyn + dwb;
end

function qua_n = attqua_update(qua, dq)
    dqnorm = norm(dq);

    if dqnorm == 0
        
        qua_n = qua;
    else
        
        co=cos(0.5*dqnorm);
        si=sin(0.5*dqnorm);
        
        n1 = dq(1)/dqnorm;
        n2 = dq(2)/dqnorm;
        n3 = dq(3)/dqnorm;
        
        qw1 = n1*si;
        qw2 = n2*si;
        qw3 = n3*si;
        qw4 = co;
        
        Om=[ qw4,  qw3, -qw2, qw1;
            -qw3,  qw4,  qw1, qw2;
            qw2, -qw1,  qw4, qw3;
            -qw1, -qw2, -qw3, qw4];
        
        qua_n = Om * qua;
    end

end