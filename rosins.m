
bag = rosbag("2020-06-16-15-30-21.bag");
bag = select(bag,'Topic', {'/imu/imu', '/gps/navpvt'});
msgs = readMessages(bag,'DataFormat','struct');
rostime = bag.MessageList.Time;

%% Initial alignment (optional, not realtime)
% calib = struct;
% j=1;
% for i = 10:length(msgs)
%     if msgs.MessageType == 'sensor_msgs/Imu'
%         continue
%     end
%     calib.Timestamp(j) = rostime(i);
%     calib.ax(j) = msgs{i-1,1}.LinearAcceleration.X;
%     calib.ay(j) = msgs{i-1,1}.LinearAcceleration.Y;
%     calib.az(j) = msgs{i-1,1}.LinearAcceleration.Z;
%     calib.wx(j) = msgs{i-1,1}.AngularVelocity.X;
%     calib.wy(j) = msgs{i-1,1}.AngularVelocity.Y;
%     calib.wz(j) = msgs{i-1,1}.AngularVelocity.Z;
%     
%     calib.px(j) = msgs{i,1}.LinearAcceleration.X;
%     calib.py(j) = msgs{i,1}.LinearAcceleration.X;
%     calib.pz(j) = msgs{i,1}.LinearAcceleration.X;
%     calib.vx(j) = msgs{i,1}.LinearAcceleration.X;
% end

%% Set IMU characteristics

% Std acceleration 0.1 m/s2
imu.stda = [1.5, 1.5, 1.5]*1e-1;
% Std turn-rate 0.01 rad/s
imu.stdw = [1.0, 1.0, 1.0]*1e-2;
% Std acceleration bias 
imu.stdab = [1, 1, 1]*1e-4;
% Std turn-rate bias
imu.stdwb = [1, 1, 1]*1e-4;

% imu.stda = ones(1,3)*0.0007*9.8*sqrt(100);
% imu.stdw = ones(1,3)*0.0038*sqrt(100);
% imu.stdab = ones(1,3)*sqrt(0.002*9.8);
% imu.stdwb = ones(1,3)*sqrt(deg2rad(0.5));

% imu.stda = ones(1,3)*(0.0007*9.8*100).^2;
% imu.stdw = ones(1,3)*(deg2rad(0.0038)*100).^2;
% imu.stdab = ones(1,3)*1e-10;
% imu.stdwb = ones(1,3)*1e-10;

%% Set GNSS characteristics
% Position Dilution of Precision
gnss.stdp = [1.5, 1.5, 2.5];
% Velocity Dilution of Precision
gnss.stdv = [0.2, 0.2, 0.2];
% GPS ant position (approx)
gnss.lg = [-0.2,0.0,0.7];
% GPS ant position deviation
gnss.stdlg = 0.1*[0.2, 0.2, 0.2];


%% Initialize State

INITLEN = 0;

while (~isfield(gnss,'Lat'))
    INITLEN = INITLEN+1;
    [imu, gnss, type] = rosframe(imu, gnss, msgs{INITLEN,1});
end

DLEN = length(msgs)-INITLEN+1;

sol.t = zeros(1,DLEN);
sol.p = zeros(3,DLEN);
sol.v = zeros(3,DLEN);
sol.r = zeros(3,DLEN);
sol.dp = zeros(3,DLEN);
sol.dv = zeros(3,DLEN);
sol.dr = zeros(3,DLEN);
sol.a = zeros(3,DLEN);
sol.w = zeros(3,DLEN); 
sol.da = zeros(3,DLEN);
sol.dw = zeros(3,DLEN);
sol.lg = zeros(3,DLEN);
sol.dlg = zeros(3,DLEN);
sol.vb = zeros(3,DLEN);
sol.rb = zeros(3,DLEN);
sol.dvb = zeros(3,DLEN);
sol.drb = zeros(3,DLEN);
sol.P = zeros(18*18, DLEN);
sol.K = zeros(18*6, DLEN);
sol.z = zeros(6, DLEN);
sol.ar = zeros(3,DLEN);
sol.wr = zeros(3,DLEN);
sol.pr = zeros(3,DLEN);
sol.vr = zeros(3,DLEN);
sol.cog = zeros(1,DLEN);
sol.dop = zeros(3,DLEN);


sol.t(1) = rostime(1);

ins.ab_dyn = [0, 0, 0]';
ins.wb_dyn = [0, 0, 0]';

ins.r = [0,0, gnss.cog];
ins.CTMnb = Euler_to_CTM(ins.r);
ins.qua = CTM_to_Qua(ins.CTMnb');
ins.CTMbn = ins.CTMnb';
ins.p = gnss.p - ins.CTMbn*gnss.lg';
ins.v = gnss.v;
ins.g = [0,0,9.8]';


kf.x = [ zeros(1,9), ins.ab_dyn', ins.wb_dyn', zeros(1,3)]';
kf.P = diag([gnss.stdp, gnss.stdv, 0.1*[1,1,1], 1*imu.stdab, 1*imu.stdwb, gnss.stdlg].^2);

% kf.P = diag([gnss.stdp, gnss.stdv, 0.1*[1,1,1], ones(1,3)*0.0002*9.8, ones(1,3)*deg2rad(360)/3600, gnss.stdlg].^2);
kf.t = rostime(INITLEN);
kf.K = zeros(18,6);

kf.R = diag([gnss.dop', gnss.stdv].^2);
kf.Q = diag([imu.stda, imu.stdw, 1*imu.stdab, 1*imu.stdwb].^2);

kf.z = zeros(6,1);

%% INS loop
for i = 2:DLEN
    sol.t(i) = rostime(i);
    dt = sol.t(i) - sol.t(i-1);
    [imu, gnss, type] = rosframe(imu, gnss, msgs{INITLEN+i-1,1});
    
    % INS Mechanization
    if strcmp(type, 'imu')
        dt = 0.01;
        [ins.a, ins.w] = imu_comp(imu.ar, imu.wr, ins.ab_dyn, ins.wb_dyn, ins.CTMbn);
        [ins.CTMbn, ins.qua, ins.r] = att_update(ins.CTMbn, ins.qua, dt, ins.w, 'quaternion');
        ins.f = ins.CTMbn*ins.a - ins.g;
        ins.v = vel_update(ins.f, ins.v, dt);
        ins.p = pos_update(ins.p, ins.v, dt);
        [kf.F, kf.G] = update_F(ins, imu);
        kf = predict(kf, dt);
    
    end
        
    % Kalman Filtering
    
    if strcmp(type, 'gnss')
        kf.H = update_H(ins, gnss);
        kf.R = diag([gnss.dop', gnss.stdv].^2);
        kf = update_z(kf, ins, gnss);
        kf = correct(kf);
%         display(gnss.p)
        
        ins = pvt_comp(kf.x, ins);
        gnss.lg = gnss.lg - kf.x(16:18)';
        
        kf.x(1:end) = 0;
    end
    
    
    
    sol.z(:,i) = kf.z;
    sol.p(:,i) = ins.p;
    sol.pg(:,i) = gnss.p;
    sol.K(:,i) = reshape(kf.K', 18*6, 1);
    sol.dp(:,i) = kf.x(1:3);
    sol.dv(:,i) = kf.x(4:6);
    sol.dr(:,i) = kf.x(7:9);
    sol.da(:,i) = kf.x(10:12);
    sol.dw(:,i) = kf.x(13:15);
    sol.v(:,i) = ins.v;
    sol.r(:,i) = ins.r;
    sol.a(:,i) = ins.f;
    sol.w(:,i) = ins.w;
    sol.ab(:,i) = ins.ab_dyn;
    sol.wb(:,i) = ins.wb_dyn;
    sol.lg(:,i) = gnss.lg;
    sol.dlg(:,i) = kf.x(16:18);
    sol.P(:,i) = reshape(kf.P, 18*18, 1);
    sol.ar(:,i) = imu.ar;
    sol.wr(:,i) = imu.wr;
    sol.cog(:,i) = gnss.cog;
    sol.pr(:,i) = gnss.p;
    sol.vr(:,i) = gnss.v;
    sol.dop(:,i) = gnss.dop;
    if (mod(i,1000)==0), fprintf('.');end
    if (mod(i,20000)==0), fprintf('\n');end
end
