
%% Set Time Stamp

%% Set IMU characteristics
% Std acceleration 0.1 m/s2
imu.stda = [0.15, 0.15, 0.15];
% Std turn-rate 0.01 rad/s
imu.stdw = [0.01, 0.01, 0.01];
% Std acceleration bias 
imu.stdab = [1, 1, 1]*1e-6;
% Std turn-rate bias
imu.stdwb = [1, 1, 1]*1e-6;



%% Set GNSS characteristics
% Position Dilution of Precision
gnss.stdp = [1.5, 1.5, 2.5];
% Velocity Dilution of Precision
gnss.stdv = [0.2, 0.2, 0.3];
% GPS ant position (approx)
gnss.lg = [0,-0.4,0];
% GPS ant position deviation
gnss.stdlg = [0.2, 0.2, 0.2];


%% Initialize States

DLEN = length(carsim_data.wsx.time);

sol.t = zeros(1,DLEN);
sol.p = zeros(3,DLEN);
sol.v = zeros(3,DLEN);
sol.r = zeros(3,DLEN);
sol.a = zeros(3,DLEN);
sol.w = zeros(3,DLEN); 
sol.da = zeros(3,DLEN);
sol.dw = zeros(3,DLEN);
sol.dl = zeros(3,DLEN);
sol.P = zeros(18*18, DLEN);

% [MisSensX, MisSensY, MisSensZ, OffSensAX, OffSensAY, OffSensAZ, OffSensRX, OffSensRY, OffSensRZ, CalibInitErr] = calibInit(carsim_data.wsx.Time, carsim_data.asx.data, carsim_data.asy.data, carsim_data.asz.data, carsim_data.wsx.data, carsim_data.wsy.data, carsim_data.wsz.data, carsim_data.Vx.data, carsim_data.Psi.data, carsim_data.Pgx.data, carsim_data.Pgy.data);

% ins.ab_dyn = [OffSensAX,OffSensAY,OffSensAZ]';
ins.ab_dyn = [0,0,0]';
% ins.wb_dyn = [OffSensRX,OffSensRY,OffSensRZ]';
ins.wb_dyn = [0,0,0]';


ins.p = [carsim_data.Pgn.data(1), carsim_data.Pge.data(1), carsim_data.Pgd.data(1)]';
ins.v = [carsim_data.Vgn.data(1), carsim_data.Vge.data(1), carsim_data.Vgd.data(1)]';
% ins.r = [MisSensX, MisSensY, MisSensZ]';
ins.r = [0,0,carsim_data.Psi.data(1)]';

ins.CTMnb = Euler_to_CTM(ins.r);
ins.qua = Euler_to_Qua(ins.r);
ins.CTMbn = ins.CTMnb';
ins.g = -[0,0,9.8]';

kf.x = [ zeros(1,9), ins.ab_dyn', ins.wb_dyn', zeros(1,3)]';
kf.P = diag([gnss.stdp, gnss.stdv, 0.2*[1,1,1], imu.stdab, imu.stdwb, gnss.stdlg].^2);

kf.R = diag([gnss.stdp, gnss.stdv].^2);
kf.Q = diag([imu.stda, imu.stdw, imu.stdab, imu.stdwb].^2);

sol.t(1) = carsim_data.wsx.time(1);

for i = 2:DLEN
    %% Get Data Frame
    sol.t(i) = carsim_data.wsx.time(i);
    dt = sol.t(i) - sol.t(i-1);
    [imu, gnss] = simulationframe(imu, gnss, carsim_data, i);
    
    %% INS
    % Print procedure
    if (mod(i,1000)==0), fprintf('.');end
    if (mod(i,20000)==0), fprintf('\n');end
    
    [ins.a, ins.w] = imu_comp(imu.ar, imu.wr, ins.ab_dyn, ins.wb_dyn, ins.CTMbn);
    [ins.CTMbn, ins.qua, ins.r] = att_update(ins.CTMbn, ins.qua, dt, ins.w, 'quaternion');
    ins.f = ins.CTMbn*ins.a - ins.g;
    ins.v = vel_update(ins.f, ins.v, dt);
    ins.p = pos_update(ins.p, ins.v, dt);
    
    %% Kalman Filter
    
    % kf.x(1:9) = 0;
    
    % [kf.F, kf.G] = update_F(ins,imu);
    % kf.H = update_H(ins.CTMbn, gnss.lg, ins.w);
    
    % kf = predict(kf, dt);
    % kf = update_z(kf, ins, gnss);
    % kf = correct(kf);
    
    %% INS Correction
    
    ins = pvt_comp(kf.x(1:9), ins);
    ins.CTMbn = Euler_to_CTM(ins.r)';
    ins.ab_dyn = kf.x(10:12);
    ins.wb_dyn = kf.x(13:15);
    sol.p(:,i) = ins.p;
    sol.dp(:,i) = kf.x(1:3);
    sol.dv(:,i) = kf.x(4:6);
    sol.dr(:,i) = kf.x(7:9);
    sol.v(:,i) = ins.v;
    sol.r(:,i) = ins.r;
    sol.a(:,i) = ins.a;
    sol.f(:,i) = ins.f;
    sol.w(:,i) = ins.w;
    sol.da(:,i) = ins.ab_dyn;
    sol.dw(:,i) = ins.wb_dyn;
    sol.dl(:,i) = kf.x(16:18);
    sol.P(:,i) = reshape(kf.P, 18*18, 1);
    
end
