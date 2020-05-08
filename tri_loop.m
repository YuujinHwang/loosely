function err = tri_loop(carsim_data,lo_est)
    

    betab = atan2(carsim_data.Vay.data, carsim_data.Vax.data);
    betaa = unwrap(atan2(carsim_data.Vgy.data,carsim_data.Vgx.data)-carsim_data.Psi.data);
    
    vela = sqrt(carsim_data.Vgx.data.^2 + carsim_data.Vgy.data.^2);
    velb = sqrt(carsim_data.Vax.data.^2 + carsim_data.Vay.data.^2);
    
    w = carsim_data.wsz.data;
    coeff = w./vela;
    
    Ra = abs(vela./w);
    start_point = [0,2];
    err_dist = 0*betaa;
    parfor i = 1:length(betaa)
        err_dist(i) = tri_test(start_point, Ra(i), lo_est, betaa(i), -coeff(i));
    end
    err = err_dist;
end