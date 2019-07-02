function [imu,gnss] = simulationframe(imu, gnss, carsim_data, time)
%SIMULATION : Set imu and gnss with given time frame

imu.ar = [carsim_data.asx.data(time), carsim_data.asy.data(time), carsim_data.asz.data(time)]';
imu.wr = [carsim_data.wsx.data(time), carsim_data.wsy.data(time), carsim_data.wsz.data(time)]';
gnss.p = [carsim_data.Pgn.data(time), carsim_data.Pge.data(time), carsim_data.Pgd.data(time)]';
gnss.v = [carsim_data.Vgn.data(time), carsim_data.Vge.data(time), carsim_data.Vgd.data(time)]';


end

