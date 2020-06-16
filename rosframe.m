function [imu, gnss, type] = rosframe(imu, gnss, msg)
    if strcmp(msg.MessageType, 'sensor_msgs/Imu')
        imu.ar = [msg.LinearAcceleration.X, msg.LinearAcceleration.Y, msg.LinearAcceleration.Z]';
        imu.wr = [msg.AngularVelocity.X, msg.AngularVelocity.Y, msg.AngularVelocity.Z]';
        type = 'imu';
    elseif strcmp(msg.MessageType, 'ublox_msgs/NavPVT')
        if ~isfield(gnss,'Lat')
            gnss.Lat = double(msg.Lat)/1e7;
            gnss.Lon = double(msg.Lon)/1e7;            
        end
        type = 'gnss';
            gnss.p = [geo_to_lin(double(msg.Lat)/1e7, double(msg.Lon)/1e7, [gnss.Lat, gnss.Lon]), double(msg.Height)/1e3]';
            gnss.v = [double(msg.VelE)/1e3, double(msg.VelN)/1e3, -double(msg.VelD)/1e3]';
            gnss.cog = deg2rad((-double(msg.Heading)/1e5+90));
            gnss.dop = [double(msg.HAcc), double(msg.HAcc), double(msg.VAcc)]'/1e3;
            if (norm(gnss.v)<0.5)
                gnss.v = [0,0,0]';
            end
    end
        