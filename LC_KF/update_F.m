function [F, G] = update_F(ins, imu)
%LC_KF_F : Pre-calculate Transition Matrix F
%          kf - Kalman Filter Object
%               x - State Vector (18x1)
%                   dP - Position Error (3x1)
%                   dV - Velocity Error (3x1)
%                   dr - Attitude Error (3x1)
%                   ab_dyn - Accelerometer Bias (3x1)
%                   w_dyn - Gyroscope Bias (3x1)
%                   dl - GPS Lever Arm (3x1)
%               F = Transition Matrix
%               H = Measurement Jacobian
%               t = Update Timestamp
%
%           ins - Mechanized Inertial Navigation Solution
%               p - Position Solution (3x1)
%               v - Velocity Solution (3x1)
%               CTMbn - Body to Navigation Coordinate Transfom Matrix
%               f - Specific Force (Acceleration without Gravity)
%               a - Acceleration Compensated Signal
%               w - Gyroscope Compensated Signal
%               ab_dyn - Acceleration Dynamic Bias
%               wb_dyn - Gyroscope Dynamic Bias
%           imu - IMU sensor measurement
%               ar - Acceleration Raw Signal
%               wr - Gyroscope Raw Signal
%               temp - Gyroscope temperature


%% Position Transition Matrices
Fpp = zeros(3);
Fpv = eye(3);
Fpr = zeros(3);

%% Velocity Transition Matrices
Fvp = zeros(3);
Fvv = zeros(3);
Fvr = skew(ins.f);

%% Attitude Transition Matrices
Frp = zeros(3);
Frv = zeros(3);
% Frr = ins.CTMbn*skew(ins.w)*ins.CTMbn';
Frr = zeros(3);

%% VBF Misalignment Matrices
% Fvbvb = -skew(vbf.CTMab*ins.w);
% Fvbr = skew(vbf.v)*skew(vbf.CTMab*ins.w)+skew(vbf.CTMab*ins.f);
% Fvbdf = vbf.CTMab;
% Fvbdw = -skew(vbf.v)*vbf.CTMab;

Fvbdf = zeros(3);
Fvbdw = zeros(3);

%% Foam Transition Matrix F
O = zeros(3);

%%%%%%%%%%%%%%CHECK CHECK CHECK 
F = [ Fpp,      Fpv,        Fpr,        O,          O,          O;
      Fvp,      Fvv,        Fvr,        ins.CTMbn,  O,          O;
      Frp,      Frv,        Frr,        O,          -ins.CTMbn, O;
      O,        O,          O,          O,          O,          O;
      O,        O,          O,          O,          O,          O;
      O,        O,          O,          O,          O,          O;];

% F = [ Fpp,      Fpv,        Fpr,        O,          O,          O,     O;
%       Fvp,      Fvv,        Fvr,        ins.CTMbn,  O,          O,     O;
%       Frp,      Frv,        Frr,        O,          -ins.CTMbn, O,     O;
%       O,        O,          O,          O,          O,          O,     O;
%       O,        O,          O,          O,          O,          O,     O;
%       O,        O,          O,          O,          O,          O,     O;
%       O,        O,          O,          O,          O,          O,     O; ];
  
% G = [  O,       O,          O,          O,      O,      O;
%        ins.CTMbn,  O,       O,          O,      O,      O;
%        O,       -ins.CTMbn,  O,          O,     O,      O;
%        O,       O,          eye(3),     O,      O,      O;
%        O,       O,          O,          eye(3),     O,      O;
%        O,       O,          O,          O,      O,      O;
%        0*vbf.CTMab,       0*skew(vbf.v)*vbf.CTMab,O          O,          eye(3),      O;
%        O,       O,          O,          O,      O,      eye(3); ]; % Q order -> Acc std, Gyro std, Vel Random Walk, Ang Random Walk, 

G = [  O,       O,          O,          O;
       ins.CTMbn,  O,       O,          O;
       O,       -ins.CTMbn,  O,          O;
       O,       O,          eye(3),     O;
       O,       O,          O,          eye(3);
       O,       O,          O,          O;];
   
% G = [  O,       O,          O,          O;
%        ins.CTMbn,  O,       O,          O;
%        O,       -ins.CTMbn,  O,          O;
%        O,       O,          eye(3),     O;
%        O,       O,          O,          eye(3);
%        O,       O,          O,          O;
%        vbf.CTMab,       skew(vbf.v)*vbf.CTMab,          O,          O;
%        O,       O,          O,          O; ];

end

% % Attitude Transition Matrix
% Frp = zeros(3);
% Frv = zeros(3);
% Frr = zeros(3);

% % Velocity Transition Matrix
% Fvp = zeros(3);
% Fvv = zeros(3);
% Fvr = 
