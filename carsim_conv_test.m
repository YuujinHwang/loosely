
%% Set Time Stamp

%% Set IMU characteristics
% Std acceleration 0.1 m/s2
imu.stda = [1.5, 1.5, 1.5]*1e-1;
% Std turn-rate 0.01 rad/s
imu.stdw = [1.0, 1.0, 1.0]*1e-3;
% Std acceleration bias 
imu.stdab = [1, 1, 1]*1e-4;
% Std turn-rate bias
imu.stdwb = [1, 1, 1]*1e-4;



% MTi Xsens

% Std acceleration 0.1 m/s2
imu.stda = [1.5, 1.5, 1.5]*1e-1;
% Std turn-rate 0.01 rad/s
imu.stdw = [1.0, 1.0, 1.0]*1e-3;
% Std acceleration bias 
imu.stdab = [1, 1, 1]*1e-5;
% Std turn-rate bias
imu.stdwb = [1, 1, 1]*1e-5;



%% Set GNSS characteristics
% Position Dilution of Precision
gnss.stdp = 0.25*[2.5, 2.5, 3.5];
% Velocity Dilution of Precision
gnss.stdv = 0.5*[0.2, 0.2, 0.3];
% GPS ant position (approx)
gnss.lg = [-0.3,0.0,0.2];
% GPS ant position deviation
gnss.stdlg = 0.1*[0.2, 0.2, 0.2];

vbf.stdnhc = [1, 1, 1];
vbf.stdkin = [1, 1, 1];
vbf.stdmis = 0.01*[1, 1, 1];
vbf.stdlo = 0.5*[0.2, 0.2, 0.2];

%% Initialize States

% DLEN = length(carsim_data.wgx.time);
DLEN = length(inertial.TOW)
gnss.eps = 0.005;

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

% [MisSensX, MisSensY, MisSensZ, OffSensAX, OffSensAY, OffSensAZ, OffSensRX, OffSensRY, OffSensRZ, CalibInitErr] = calibInit(carsim_data.wgx.Time, carsim_data.asx.data, carsim_data.asy.data, carsim_data.asz.data, carsim_data.wsx.data, carsim_data.wsy.data, carsim_data.wsz.data, carsim_data.Vx.data, carsim_data.Psi.data, carsim_data.Pgx.data, carsim_data.Pgy.data);



lpos = geo_to_lin(GPS.Lat, GPS.Lon, [GPS.Lat(1), GPS.Lon(1)]);
GPS.PosLocalX = lpos(:,1);
GPS.PosLocalY = lpos(:,2);
for i = 1:length(GPS.GPSTOW)
    [~,idx(i)] = min(abs(inertial.TOW - GPS.GPSTOW(i)));
    GPS.AccelXr(i) = inertial.AccelXr(idx(i));
    GPS.AccelYr(i) = inertial.AccelYr(idx(i));
    GPS.AccelZr(i) = inertial.AccelZr(idx(i));
    GPS.AngleRateXr(i) = inertial.AngleRateXr(idx(i));
    GPS.AngleRateYr(i) = inertial.AngleRateYr(idx(i));
    GPS.AngleRateZr(i) = inertial.AngleRateZr(idx(i));
end

[imu.Rinit,imu.t,imu.Err] = calibInit2(GPS.GPSTOW,GPS.AccelXr*9.8,GPS.AccelYr*9.8,GPS.AccelZr*9.8,GPS.AngleRateXr,GPS.AngleRateYr,GPS.AngleRateZr,GPS.Speed,deg2rad(GPS.AngleTrack),GPS.PosLocalX,GPS.PosLocalY);
vbf.misalign = [-0.0,-0.0,0.0];
vbf.CTMma = Euler_to_CTM(vbf.misalign);
vbf.CTMma = imu.Rinit';

% ins.ab_dyn = [OffSensAX,OffSensAY,OffSensAZ]';
ins.ab_dyn = [-0.0, -0.0, -0]';
ins.ab_dyn_init = [0.0,0.0,-0.0]';
% ins.wb_dyn = [OffSensRX,OffSensRY,OffSensRZ]';
ins.wb_dyn = [-0.0,-0.0,-0.0]';
% ins.wb_dyn = [imu.Rinit*imu.t(:,2)];
ins.wb_dyn_init = [-0.00,0.00,0.00]';
% ins.r = [MisSensX, MisSensY, MisSensZ]';
% ins.r = [0,0.0, -2.5]';
% ins.r = [0,0, carsim_data.Psi.data(1)]';
ins.r = [0,0,0]';

% [imu, gnss] = simulationframe_old(imu, gnss, carsim_data, 1, ins.ab_dyn_init, ins.wb_dyn_init, vbf.CTMma);
[imu,gnss] = rtframe(imu, gnss, inertial, GPS, vbf.CTMma, 1, 'IMU');
[imu,gnss] = rtframe(imu, gnss, inertial, GPS, vbf.CTMma,1, 'GPS');
% ins.r = [0,0,0]';
ins.r = deg2rad([atan2d(imu.ar(2),9.8),atan2d(imu.ar(1),9.8),-GPS.AngleTrack(1)+90])';
ins.CTMnb = Euler_to_CTM(ins.r);
ins.qua = CTM_to_Qua(ins.CTMnb');
ins.CTMbn = ins.CTMnb';
ins.p = gnss.p - ins.CTMbn*gnss.lg';
ins.v = gnss.v - ins.CTMbn*skew(gnss.lg')*imu.wr;
ins.g = [0,0,9.8]';

gnss.vg = ins.CTMbn'*gnss.v;
gnss.cogp = atan2d(gnss.v(2),gnss.v(1));

cf.vg = gnss.vg;
cf.cog = atan2d(gnss.v(2),gnss.v(1));
cf.CTMbn = ins.CTMbn;
cf.q = CTM_to_Qua(cf.CTMbn);
cf.v = [2*(cf.q(1)*cf.q(3)+cf.q(4)*cf.q(2));
        2*(cf.q(2)*cf.q(3)+cf.q(4)*cf.q(1));
        cf.q(4)^2-cf.q(1)^2-cf.q(2)^2-cf.q(3)^2];
cf.d = [0;0;0];
cf.ei = [0;0;0];
cf.r = [ins.r(1:2);0];

% vbf.v = [norm(ins.v); 0; 0];

vbf.v = norm(ins.v);
vbf.r = [-0.00, -0.00, 0.00]';
vbf.lo = [-2.0, 0.3, -0.7]';
vbf.CTMab = Euler_to_CTM(vbf.r)';
vbf.qua = Euler_to_Qua(vbf.r);
vbf.vb = [vbf.v;0;0];

kf.x = [ zeros(1,9), ins.ab_dyn', ins.wb_dyn', zeros(1,3)]';
kf.P = diag([20*gnss.stdp, 5*gnss.stdv, 0.1*[1,1,1], 100*imu.stdab, 100*imu.stdwb, gnss.stdlg].^2);
kf.t = 0;
kf.K = zeros(18,6);

MAkf.x = [zeros(1,3), zeros(1,3)]';
MAkf.P = diag([vbf.stdmis, vbf.stdlo].^2);

kf.R = 1*diag([gnss.stdp, gnss.stdv].^2);
% kf.Q = diag([imu.stda, imu.stdw, imu.stdab, imu.stdwb].^2);
kf.Q = diag([imu.stda, imu.stdw, imu.stdab, imu.stdwb].^2);

MAkf.Q = diag(1*[1*vbf.stdmis, 1*vbf.stdlo].^2);
MAkf.R = diag([vbf.stdnhc].^2);
% sol.t(1) = carsim_data.wgx.time(1);
sol.t(1) = inertial.TOW(1);
MAmode = 'horizontal';
MAmode = 'kinematic';
if strcmp(MAmode,'horizontal');
    MAkf.R = diag([vbf.stdnhc, 0.01*vbf.stdnhc].^2);
elseif strcmp(MAmode, 'kinematic');
    MAkf.P = diag(1*[0.3*vbf.stdmis, vbf.stdlo, vbf.stdmis].^2);
    MAkf.Q = diag(1*[0.1*vbf.stdmis, vbf.stdlo, vbf.stdkin].^2);
    MAkf.R = diag([10*vbf.stdnhc, 0.5*vbf.stdnhc, 1000000*vbf.stdkin].^2);
    MAkf.x = [MAkf.x;zeros(3,1)];
end


kf.z = zeros(6,1);

for i = 2:DLEN
    %% Get Data Frame
    % sol.t(i) = carsim_data.wgx.time(i);
    sol.t(i) = inertial.TOW(i);
    if sol.t(i) < GPS.GPSTOW(1)
        disp('init')
        continue;
    end
    dt = sol.t(i) - sol.t(i-1);
    [imu, ~] = rtframe(imu, gnss, inertial, GPS,vbf.CTMma, i, 'IMU');
    
    %% INS
    % Print procedure
    if (mod(i,1000)==0), fprintf('.');end
    if (mod(i,20000)==0), fprintf('\n');end
    if norm(ins.v) > 0.0
        [ins.a, ins.w] = imu_comp(imu.ar, imu.wr, ins.ab_dyn, ins.wb_dyn, ins.CTMbn);
        [ins.CTMbn, ins.qua, ins.r] = att_update(ins.CTMbn, ins.qua, dt, ins.w, 'quaternion');
        ins.f = ins.CTMbn*ins.a - ins.g;
        ins.v = vel_update(ins.f, ins.v, dt);
        ins.p = pos_update(ins.p, ins.v, dt);
    end

    %% Kalman Filter
    
    kf.x(1:end) = 0;
    MAkf.x(1:end) = 0;

    lpgain = 0.02;

    phiw = (1-lpgain)*(phiw + dt*(ins.w(1)+ins.theta2*ins.w(3)));
    thetaw = (1-lpgain)*(thetaw + dt*(ins.w(2)-ins.phi2*ins.w(3)));

    % [kf.F, kf.G] = update_F(ins,imu, dt);
    % kf.H = update_H(ins, gnss, vbf);
        
    % kf = predict(kf, dt);
    % kf = update_z(kf, ins, gnss, vbf);
    % kf = correct(kf);
    gdx =  find (GPS.GPSTOW >= (sol.t(i) - gnss.eps) & GPS.GPSTOW < (sol.t(i) + gnss.eps));
    % [F, G] = update_F(ins,imu);
    % kf.F = kf.F + dt*F;
    % kf.G = kf.G + dt*G;
    % kf.dt = kf.dt + dt;
    if ( ~isempty(gdx) && gdx > 1)
        if gdx == 2
            kf.t = sol.t(i);
            continue
        else    
            [~,gnss] = rtframe(imu,gnss,inertial,GPS,vbf.CTMma,gdx,'GPS');
            kf.dt = sol.t(i) - kf.t;


            % TEST AREA
            gnss.ag = (ins.CTMbn'*gnss.v-gnss.vg)/(sol.t(i)-kf.t);
            gnss.hr = diff(unwrap([deg2rad(gnss.cogp),deg2rad(gnss.cog)]))/(sol.t(i)-kf.t);
            gnss.cogp = gnss.cog;
            gnss.ag(2) = gnss.ag(2) + gnss.hr*norm(gnss.vg);
            gnss.vg = ins.CTMbn'*gnss.v;
            ins.phi = 0.9*ins.phi + 0.1*atan2((ins.a(2) - gnss.ag(2)),ins.a(3));
            ins.theta = 0.9*ins.theta + 0.1*atan2(-(ins.a(1)-gnss.ag(1)),norm([ins.a(2), ins.a(3)]));
            
            thetag = lpgain*(gnss.ag(1) - ins.a(1))/ins.a(3) + (1-lpgain)*thetag;
            phig = -lpgain*(norm(gnss.vg)*gnss.hr - ins.a(2)) / ins.a(3) + (1-lpgain)*phig;
            chk = sum(ins.a.^2)/(ins.a'*[gnss.ag(1), gnss.hr*norm(gnss.vg), 9.8]');

            % TEST END


            [kf.F, kf.G] = update_F(ins,imu);
            % kf.F = kf.F/kf.dt;
            % kf.G = kf.G/kf.dt;
            kf.H = update_H(ins, gnss, vbf);
            
            kf = predict(kf, kf.dt);
            kf = update_z(kf, ins, gnss, vbf);
            if kf.dt<0.5
                % kf.z = kf.z/100000;
                kf = correct(kf);
            end
            % kf = correct(kf);
            kf.t = sol.t(i);
            kf.F = zeros(size(kf.F));
            kf.G = zeros(size(kf.G));
            cf = comp_filter(cf,gnss,ins,kf.dt,0.05,0.3);

            kf.dt = 0;
            vbf.vb = vbf.CTMab*ins.CTMbn'*ins.v;            
        end
    end
    ins.phi2 = phiw+phig;
    ins.theta2 = thetaw+thetag;
    
    ins = pvt_comp(kf.x, ins);
    gnss.lg = gnss.lg - kf.x(16:18)';

    vbf = vbf_update(vbf, ins, dt, kf.dt);

    [MAkf.F, MAkf.G] = MA_update_F(ins,imu, vbf,MAmode);
    MAkf.H = MA_update_H(ins, gnss, vbf, MAmode);
    MAkf = predict(MAkf, dt);
    MAkf = MA_update_z(MAkf, ins, gnss, vbf, MAmode);
    MAkf = correct(MAkf);
    
    % %% INS Correction
    
    vbf = vbf_comp(MAkf.x, vbf, ins);

    sol.MAz(:,i) = MAkf.z;
    sol.MAk(:,i) = reshape(MAkf.K',9*9,1);
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
    sol.va(:,i) = ins.va;
    sol.r(:,i) = ins.r;
    sol.a(:,i) = ins.f;
    sol.w(:,i) = ins.w;
    sol.ab(:,i) = ins.ab_dyn;
    sol.wb(:,i) = ins.wb_dyn;
    sol.lg(:,i) = gnss.lg;
    sol.dlg(:,i) = kf.x(16:18);
    sol.P(:,i) = reshape(kf.P, 18*18, 1);
    sol.v3(:,i) =  vbf.CTMab*ins.CTMbn'*ins.v;
    sol.vb(:,i) = vbf.vb;
    sol.rb(:,i) = vbf.r;
    sol.lo(:,i) = vbf.lo;
    sol.drb(:,i) = MAkf.x(1:3);
    sol.dlo(:,i) = MAkf.x(4:6);
    % sol.drb(:,i) = kf.x(22:24);
    sol.avbf(:,i) = vbf.f;
    sol.wvbf(:,i) = vbf.w;
    sol.dvvbf(:,i) = vbf.dv;
    % sol.res(:,i) = vbf.res;
    % sol.alpha1(:,i) = vbf.alpha1;
    % sol.alpha2(:,i) = vbf.alpha2;
    sol.sig3(:,i) = [abs(sol.P(1:19:end,i).^(0.5)).*3; MAkf.P(1:10:end)'];
    sol.beta(:,i) = vbf.beta;
    sol.va(:,i) = vbf.va;

    sol.rc(:,i) = cf.r;
    % sol.hr(:,i) = gnss.hr;
    sol.rg(:,i) = [ins.phi, ins.theta, ins.phi2, ins.theta2]';
    sol.chk(:,i) = chk;
    
end
