figure;
title('Position XY')
grid on
plot(sol.p(1,:), sol.p(2,:)); hold;
plot(carsim_data.Pgx.data, carsim_data.Pgy.data);hold;
legend('X', 'Y')
figure;
title('Position Error')
grid on
hold;
plot(sol.t, sol.p(1,:)-carsim_data.Pgx.data', 'b', sol.t, sol.sig3(1,:), '--b',sol.t,- sol.sig3(1,:), '--b');
plot(sol.t, sol.p(2,:)-carsim_data.Pgy.data', 'r', sol.t, sol.sig3(2,:), '--r',sol.t, -sol.sig3(2,:), '--r');
plot(sol.t, sol.p(3,:)-carsim_data.Pgz.data', 'g', sol.t, sol.sig3(3,:), '--g',sol.t, -sol.sig3(3,:), '--g');
hold;
legend('X', '\sigma_X', '\sigma_X','Y',  '\sigma_Y', '\sigma_Y','Z','\sigma_Z', '\sigma_Z')
figure;

title('Velocity Error')
grid on
hold;
plot(sol.t, sol.v(1,:)-carsim_data.Vsx.data', 'b', sol.t, sol.sig3(4,:), '--b',sol.t,- sol.sig3(4,:), '--b');
plot(sol.t, sol.v(2,:)-carsim_data.Vsy.data', 'r', sol.t, sol.sig3(5,:), '--r',sol.t,- sol.sig3(5,:), '--r');
plot(sol.t, sol.v(3,:)-carsim_data.Vsz.data', 'g', sol.t, sol.sig3(6,:), '--g', sol.t,- sol.sig3(6,:), '--g');
hold;
legend('X', '\sigma_X', '\sigma_X','Y',  '\sigma_Y', '\sigma_Y','Z','\sigma_Z', '\sigma_Z')
figure;
title('Attitude Error')
grid on
hold;
plot(sol.t, sol.r(1,:)-carsim_data.Phi.data'-vbf.misalign(1), 'b', sol.t, sol.sig3(7,:), '--b', sol.t, -sol.sig3(7,:), '--b');
plot(sol.t, sol.r(2,:)-carsim_data.Theta.data'-vbf.misalign(2), 'r', sol.t, sol.sig3(8,:), '--r', sol.t, -sol.sig3(8,:), '--r');
plot(sol.t, unwrap(sol.r(3,:))-carsim_data.Psi.data'-vbf.misalign(3), 'g', sol.t, sol.sig3(9,:), '--g', sol.t,- sol.sig3(9,:), '--g');
hold;
legend('X', '\sigma_X', '\sigma_X','Y',  '\sigma_Y', '\sigma_Y','Z','\sigma_Z', '\sigma_Z')
figure;
title('Accelerometer Bias')
grid on
hold;
plot(sol.t, sol.ab(1,:), 'b', sol.t, sol.sig3(10,:), '--b');
plot(sol.t, sol.ab(2,:), 'r', sol.t, sol.sig3(11,:), '--r');
plot(sol.t, sol.ab(3,:), 'g', sol.t, sol.sig3(12,:), '--g');
hold;
legend('X', '\sigma_X','Y',  '\sigma_Y','Z','\sigma_Z')
figure;
title('Gyro Bias')
grid on
hold;
plot(sol.t, sol.wb(1,:), 'b', sol.t, sol.sig3(13,:), '--b');
plot(sol.t, sol.wb(2,:), 'r', sol.t, sol.sig3(14,:), '--r');
plot(sol.t, sol.wb(3,:), 'g', sol.t, sol.sig3(15,:), '--g');
hold;
legend('X', '\sigma_X','Y',  '\sigma_Y','Z','\sigma_Z')
figure;
title('GPS Lever Arm')
grid on
hold;
plot(sol.t, sol.lg(1,:), 'b', sol.t, sol.sig3(16,:), '--b');
plot(sol.t, sol.lg(2,:), 'r', sol.t, sol.sig3(17,:), '--r');
plot(sol.t, sol.lg(3,:), 'g', sol.t, sol.sig3(18,:), '--g');
hold;
legend('X', '\sigma_X','Y',  '\sigma_Y','Z','\sigma_Z')
figure;
title('IMU Misalignment')
grid on
hold;
plot(sol.t, sol.rb(1,:), 'b', sol.t, sol.sig3(19,:), '--b');
plot(sol.t, sol.rb(2,:), 'r', sol.t, sol.sig3(20,:), '--r');
plot(sol.t, sol.rb(3,:), 'g', sol.t, sol.sig3(21,:), '--g');
hold;
legend('X', '\sigma_X','Y',  '\sigma_Y','Z','\sigma_Z')
figure;
title('Axle Lever Arm')
grid on
hold;
plot(sol.t, sol.lo(1,:), 'b', sol.t, sol.sig3(22,:), '--b');
plot(sol.t, sol.lo(2,:), 'r', sol.t, sol.sig3(23,:), '--r');
plot(sol.t, sol.lo(3,:), 'g', sol.t, sol.sig3(24,:), '--g');
hold;
legend('X', '\sigma_X','Y',  '\sigma_Y','Z','\sigma_Z')