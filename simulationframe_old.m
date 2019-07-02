function [imu,gnss] = simulationframe(imu, gnss, carsim_data, time, ab_dyn, wb_dyn)
    %SIMULATION : Set imu and gnss with given time frame
    
    imu.ar = [carsim_data.asx.data(time), carsim_data.asy.data(time), carsim_data.asz.data(time)]'+ab_dyn;
    imu.wr = [carsim_data.wsx.data(time), carsim_data.wsy.data(time), carsim_data.wsz.data(time)]'+wb_dyn;
    gnss.p = [carsim_data.Pgx.data(time), carsim_data.Pgy.data(time), carsim_data.Pgz.data(time)]';
    gnss.v = [carsim_data.Vgx.data(time), carsim_data.Vgy.data(time), carsim_data.Vgz.data(time)]';
    
    
    end
    
    