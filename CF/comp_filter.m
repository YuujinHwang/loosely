function cf = comp_filter(cf, gnss, ins, dt, ki, kp)
    
    cf.hr = diff(unwrap([deg2rad(cf.cog),deg2rad(gnss.cog)]))/dt;
    cf.cog = gnss.cog;
    
    cf.f = (ins.CTMbn'*gnss.v - cf.vg)/dt;
    cf.f(2) = cf.f(2) + cf.hr*norm(cf.vg);
    cf.vg = ins.CTMbn'*gnss.v;
    % cf.phi = atan2((ins.a(2)-cf.a(2)),ins.a(3));
    % cf.theta = atan2(-(ins.a(1)-cf.a(1)),norm([ins.a(2), ins.a(3)]));
    cf.w = cf.CTMbn*ins.w;
    cf.g = (ins.a - cf.f);
    cf.e = skew(cf.g/9.8)*cf.v;
    cf.e(3) = 0;
    cf.ei = 0.98*cf.ei + cf.e;
    cf.d = ki*cf.ei + kp*cf.e;   
    cf.r = [atan2(cf.g(2),cf.g(3));atan2(-cf.g(1),norm(cf.g(2:3)));0];
    cf.q = attqua_update(cf.q, [cf.w(1:2);0]+[cf.d(1:2);0], dt);
    cf.q = cf.q/norm(cf.q);
    % cf.q = ins.qua;
    cf.v = [2*(cf.q(1)*cf.q(3)+cf.q(4)*cf.q(2));
            2*(cf.q(2)*cf.q(3)+cf.q(4)*cf.q(1));
            cf.q(4)^2-cf.q(1)^2-cf.q(2)^2+cf.q(3)^2];
    cf.r = Qua_to_Euler(cf.q);
    cf.CTMbn = Qua_to_CTM(cf.q);

end

function qua_n = attqua_update(qua, wb_n, dt)
    wnorm = norm(wb_n);

    if wnorm == 0
        
        qua_n = qua;
    else
        
        co=cos(0.5*wnorm*dt);
        si=sin(0.5*wnorm*dt);
        
        n1 = wb_n(1)/wnorm;
        n2 = wb_n(2)/wnorm;
        n3 = wb_n(3)/wnorm;
        
        qw1 = n1*si;
        qw2 = n2*si;
        qw3 = n3*si;
        qw4 = co;
        
        Om=[ qw4,  qw3, -qw2, qw1;
            -qw3,  qw4,  qw1, qw2;
            qw2, -qw1,  qw4, qw3;
            -qw1, -qw2, -qw3, qw4];
        
        qua_n = Om * qua;
    end

end