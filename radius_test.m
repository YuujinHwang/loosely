function res = radius_test(lox, loy, carsim_data)

betaa = unwrap(atan2(carsim_data.Vgy.data, carsim_data.Vgx.data)-carsim_data.Psi.data);
Vsx = sqrt(carsim_data.Vgx.data.^2 + carsim_data.Vgy.data.^2).*cos(betaa);
Vsy = sqrt(carsim_data.Vgx.data.^2 + carsim_data.Vgy.data.^2).*sin(betaa);
betab = betaa+carsim_data.wsz.data*lox./Vsx;
Vb_cent = sqrt((Vsx-carsim_data.wsz.data*loy).^2 + (Vsy+carsim_data.wsz.data*lox).^2);

acenta_true = -sin(betaa).*carsim_data.agx.data + cos(betaa).*carsim_data.agy.data;

acenta = acenta_true - 9.8*carsim_data.Phi.data - sqrt(carsim_data.Vgx.data.^2+carsim_data.Vgy.data.^2).*gradient(betaa)*100;
acentb = acenta_true - 9.8*carsim_data.Phi.data - carsim_data.wsz.data.^2.*loy - gradient(betaa)*100.*Vb_cent;

Ra_cent = abs(acenta)./carsim_data.wsz.data.^2;
Rb_cent = abs(acentb)./carsim_data.wsz.data.^2;

Va = sqrt(carsim_data.Vgx.data.^2 +carsim_data.Vgy.data.^2);


Ra_vel = Va./abs(carsim_data.wsz.data);
Rb_vel = Vb_cent./abs(carsim_data.wsz.data);

sol.t = carsim_data.wsz.time;

% figure;plot(t, Ra_vel, t, Ra_cent,'--',t, Rb_vel, t, Rb_cent,'--');

% plot(sol.t, Rb_cent, sol.t, Rb_vel, sol.t, sqrt(Ra_cent.^2+lox^2-2*lox*cos(pi/2-betaa).*Ra_cent),'--',sol.t, sqrt(Ra_vel.^2+4+2*lox*cos(pi/2-betaa).*abs(Ra_vel)),'--')
% plot(sol.t, Rb_vel.^2 + Ra_vel.^2 - 2*cos(carsim_data.wsz.data*lox./Va).*Rb_vel.*Ra_vel, sol.t, Rb_cent.^2 + Ra_cent.^2 - 2*cos(carsim_data.wsz.data*lox./Va).*Rb_cent.*Ra_cent)
% res = mean(Rb_vel - Rb_cent);
% figure; plot(sol.t, 1./Rb_cent - 1./Rb_vel);
res = mean(1./Rb_cent - 1./Rb_vel);
% plot(sol.t, acentb- Vb_cent.*(carsim_data.wsz.data));
end
