function [imu,gnss] = rtframe(imu, gnss, inertial, GPS,CTMma, idx, mode)
    %SIMULATION : Set imu and gnss with given time frame
    % imu.ar = [carsim_data.aax.data(time), carsim_data.aay.data(time), carsim_data.aaz.data(time)]'+ab_dyn;
    % imu.wr = [carsim_data.wax.data(time), carsim_data.way.data(time), carsim_data.waz.data(time)]'+wb_dyn;
    % gnss.p = [carsim_data.Pgx.data(time), carsim_data.Pgy.data(time), carsim_data.Pgz.data(time)]';
    % gnss.v = [carsim_data.Vgx.data(time), carsim_data.Vgy.data(time), carsim_data.Vgz.data(time)]';
    if strcmp(mode, 'IMU')
        imu.ar = 9.8*CTMma*[inertial.AccelXr(idx), inertial.AccelYr(idx), inertial.AccelZr(idx)]';
        imu.wr = CTMma*[inertial.AngleRateXr(idx), inertial.AngleRateYr(idx), inertial.AngleRateZr(idx)]'-imu.t(:,2);
    elseif strcmp(mode,'GPS')
        gnss.p = [GPS.PosLocalX(idx), GPS.PosLocalY(idx), GPS.Height(idx)]';
        gnss.v = [GPS.VelE(idx), GPS.VelN(idx), -GPS.VelD(idx)]';
        if (norm(gnss.v)<0.1)
            gnss.v = [0,0,0]';
        else
            gnss.cog = atan2d(gnss.v(2),gnss.v(1));
        end
        
    else
        pass
    end
    % imu.ar = imu.fb(i,:);
    % imu.wr = imu.wb(i,:);
    % gnss.p = gnss.P(i,:);

    end
    