function [CTMbn_n, qua_n, euler] = att_update(CTMbn, qua, dt, w, update_mode)
%ATT_UPDATE : Update attitude
%
% Input :
%   CTMbn,  3x3 body-to-nav CTM
%   dt,     1x1 IMU Sampling Time (Interval)
%   w,     3x1 IMU gyro turn-rates (Estimated)
%
% Output :
%   CTMbn_n,    3x3 updated body-to-nav CTM
%   euler,      3x1 updated euler angle corresponding to CTMbn_n

if nargin < 5, update_mode = 'euler';end

    if strcmp(update_mode, 'euler')

        deuler = w * dt;                        % Euler angle Incremental
        CTMbn_n = attmat_update(CTMbn, deuler);    % Update CTM
        euler = CTM_to_Euler(CTMbn_n');
        qua_n = Euler_to_Qua(euler);
    elseif strcmp(update_mode, 'quaternion')
        qua_n = attqua_update(qua, w, dt);
        qua_n = qua_n/norm(qua_n);
        CTMbn_n = Qua_to_CTM(qua_n);
        euler = Qua_to_Euler(qua_n);

    end
end

function CTMbn_n = attmat_update(CTMbn,euler)
    S = skew(euler);
    magn = norm(euler);
    
    if magn == 0
        A = eye(3);
    else
        A = eye(3) + (sin(magn)/magn)*S + ((1-cos(magn))/(magn^2)) * S * S;
    end
%     A = eye(3)+S;
    
    CTMbn_n = CTMbn * A;
end

function qua_n = attqua_update(qua, wb_n, dt)
    wnorm = norm(wb_n);

    if wnorm == 0
        
        qua_n = qua;
    else
        
        co=cos(0.5*wnorm*dt);
        si=sin(0.5*wnorm*dt);
        
        n1 = wb_n(1)/wnorm;
        n2 = wb_n(2)/wnorm;
        n3 = wb_n(3)/wnorm;
        
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