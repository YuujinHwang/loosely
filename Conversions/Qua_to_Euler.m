function euler = Qua_to_Euler(qin)
% qua2euler: transforms quaternion to Euler angles.
%
% INPUT
%   qin: 4x1 quaternion.
%
% OUTPUT
%   euler: 3x1 Euler angles [roll pitch yaw] (rad, rad, rad).
%

DCMbn = Qua_to_CTM(qin);

phi   = atan2( DCMbn(3,2), DCMbn(3,3) );    % roll
theta = asin (-DCMbn(3,1) );                % pitch
psi   = atan2( DCMbn(2,1), DCMbn(1,1) );    % yaw

euler = [phi theta psi]';

end
