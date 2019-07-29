
%% Set Time Stamp

%% Set IMU characteristics
% Std acceleration 0.1 m/s2
imu.stda = [0.15, 0.15, 0.15];
% Std turn-rate 0.01 rad/s
imu.stdw = [0.01, 0.01, 0.01];
% Std acceleration bias 
imu.stdab = 1000*[1, 1, 1]*1e-6;
% Std turn-rate bias
imu.stdwb = 100*[1, 1, 1]*1e-6;



%% Set GNSS characteristics
% Position Dilution of Precision
gnss.stdp = [1.5, 1.5, 2.5];
% Velocity Dilution of Precision
gnss.stdv = [0.2, 0.2, 0.3];
% GPS ant position (approx)
gnss.lg = [0.0,0.0,0.0];
% GPS ant position deviation
gnss.stdlg = [0.2, 0.2, 0.2];

vbf.stdnhc = 5*[1, 1, 1];
vbf.stdkin = 1*[1, 1, 1];
vbf.stdmis = 0.01*[0*1, 1, 1];
vbf.stdlo = [0.5, 0.2, 0.2];

%% Initialize States

DLEN = length(carsim_data.wgx.time);

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
sol.vb = zeros(1,DLEN);
sol.rb = zeros(3,DLEN);
sol.dvb = zeros(3,DLEN);
sol.drb = zeros(3,DLEN);
sol.P = zeros(24*24, DLEN);
sol.K = zeros(24*9, DLEN);

% [MisSensX, MisSensY, MisSensZ, OffSensAX, OffSensAY, OffSensAZ, OffSensRX, OffSensRY, OffSensRZ, CalibInitErr] = calibInit(carsim_data.wgx.Time, carsim_data.asx.data, carsim_data.asy.data, carsim_data.asz.data, carsim_data.wsx.data, carsim_data.wsy.data, carsim_data.wsz.data, carsim_data.Vx.data, carsim_data.Psi.data, carsim_data.Pgx.data, carsim_data.Pgy.data);

% ins.ab_dyn = [OffSensAX,OffSensAY,OffSensAZ]';
ins.ab_dyn = [0, 0, -0]';
ins.ab_dyn_init = [0.2,0.3,-0.2]';
% ins.wb_dyn = [OffSensRX,OffSensRY,OffSensRZ]';
ins.wb_dyn = [0.0,0.0,-0.0]';
ins.wb_dyn_init = [-0.02,0.03,0.05]';


% ins.r = [MisSensX, MisSensY, MisSensZ]';
ins.r = [0,0,-2.5]';
[imu, gnss] = simulationframe_old(imu, gnss, carsim_data, 1, ins.ab_dyn_init, ins.wb_dyn_init);
% ins.r = [0,0,0]';
ins.CTMnb = Euler_to_CTM(ins.r);
ins.qua = Euler_to_Qua(ins.r);
ins.CTMbn = ins.CTMnb';
ins.p = gnss.p + ins.CTMbn*gnss.lg';
ins.v = gnss.v - ins.CTMbn*skew(gnss.lg')*imu.wr;

ins.g = [0,0,9.8]';

% vbf.v = [norm(ins.v); 0; 0];
vbf.v = norm(ins.v);
vbf.r = [-0.0, -0.05, 0.05]';
vbf.lo = [-1.8, 0.0, 0.0]';
vbf.CTMab = Euler_to_CTM(vbf.r)';
vbf.qua = Euler_to_Qua(vbf.r);

kf.x = [ zeros(1,9), ins.ab_dyn', ins.wb_dyn', zeros(1,3), zeros(1,3)]';
kf.P = diag([10*gnss.stdp, 5*gnss.stdv, 0.1*[1,1,1], 1000*imu.stdab, 1000*imu.stdwb, 5*gnss.stdlg, 100*vbf.stdmis, vbf.stdlo].^2);

kf.R = diag([gnss.stdp, gnss.stdv, vbf.stdnhc].^2);
% kf.Q = diag([imu.stda, imu.stdw, imu.stdab, imu.stdwb].^2);
kf.Q = diag([imu.stda, imu.stdw, imu.stdab, imu.stdwb, 0.01*vbf.stdmis, 0.05*vbf.stdlo].^2);
sol.t(1) = carsim_data.wgx.time(1);

for i = 2:DLEN
    %% Get Data Frame
    sol.t(i) = carsim_data.wgx.time(i);
    dt = sol.t(i) - sol.t(i-1);
    [imu, gnss] = simulationframe_old(imu, gnss, carsim_data, i, ins.ab_dyn_init, ins.wb_dyn_init);
    
    %% INS
    % Print procedure
    if (mod(i,1000)==0), fprintf('.');end
    if (mod(i,20000)==0), fprintf('\n');end
    
    [ins.a, ins.w] = imu_comp(imu.ar, imu.wr, ins.ab_dyn, ins.wb_dyn, ins.CTMbn);
    [ins.CTMbn, ins.qua, ins.r] = att_update(ins.CTMbn, ins.qua, dt, ins.w, 'quaternion');
    ins.f = ins.CTMbn*ins.a - ins.g;
    ins.v = vel_update(ins.f, ins.v, dt);
    ins.p = pos_update(ins.p, ins.v, dt);
    vbf = vbf_update(vbf, ins, dt);    
    %% Kalman Filter
    
    kf.x(1:24) = 0;
    
    [kf.F, kf.G] = update_F(ins,imu, vbf);
    kf.H = update_H(ins, gnss, vbf);
    
    kf = predict(kf, dt);
    kf = update_z(kf, ins, gnss, vbf);
    kf = correct(kf);
    
    % %% INS Correction
    
    ins = pvt_comp(kf.x, ins);
    ins.CTMbn = Euler_to_CTM(ins.r)';
    gnss.lg = gnss.lg - kf.x(16:18)';
    vbf = vbf_comp(kf.x, vbf, ins);

    sol.z(:,i) = kf.z;
    sol.p(:,i) = ins.p;
    sol.K(:,i) = reshape(kf.K', 24*9, 1);
    sol.dp(:,i) = kf.x(1:3);
    sol.dv(:,i) = kf.x(4:6);
    sol.dr(:,i) = kf.x(7:9);
    sol.da(:,i) = kf.x(10:12);
    sol.dw(:,i) = kf.x(13:15);
    sol.v(:,i) = ins.v;
    sol.va(:,i) = ins.va;
    sol.r(:,i) = ins.r;
    sol.a(:,i) = ins.f;
    sol.w(:,i) = ins.w;
    sol.ab(:,i) = ins.ab_dyn;
    sol.wb(:,i) = ins.wb_dyn;
    sol.lg(:,i) = gnss.lg;
    sol.dlg(:,i) = kf.x(16:18);
    sol.P(:,i) = reshape(kf.P, 24*24, 1);
    sol.vb(:,i) = vbf.v;
    sol.rb(:,i) = vbf.r;
    sol.lo(:,i) = vbf.lo;
    sol.drb(:,i) = kf.x(19:21);
    sol.dlo(:,i) = kf.x(22:24);
    % sol.drb(:,i) = kf.x(22:24);
    sol.avbf(:,i) = vbf.f;
    sol.wvbf(:,i) = vbf.w;
    sol.dvvbf(:,i) = vbf.dv;
    sol.res(:,i) = vbf.res;
    sol.alpha1(:,i) = vbf.alpha1;
    sol.alpha2(:,i) = vbf.alpha2;
    sol.sig3(:,i) = abs(sol.P(1:25:end,i).^(0.5)).*3;
    
end
