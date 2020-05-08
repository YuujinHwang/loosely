figure;
title('Position XY')
grid on
plot(GPS.Lon, GPS.Lat,'*'); hold;
[PosLat, PosLon] = lin_to_geo(sol.p(1:2,:)', [GPS.Lat(1), GPS.Lon(1)]);
plot(PosLon, PosLat,'.');hold;
plot_google_map('MapType','satellite')

figure;
title('Heading Attitude')
grid on
plot(GPS.GPSTOW, deg2rad(-GPS.AngleTrack+90));hold;
plot(sol.t, sol.r(3,:));hold;
legend('GPS Heading', 'Estimated');
figure;
title('Roll/Pitch Attitude')
grid on
plot(sol.t, sol.r(1,:), sol.t, sol.r(2,:));
legend('Roll', 'Pitch')
figure;
title('Estimated Velocity');
grid on
plot(sol.t, sol.vb)
figure;
title('Estimated Slipangle')
grid on
plot(sol.t, sol.beta);