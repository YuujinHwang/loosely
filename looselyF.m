function xf = looselyF(x, R, f, w)
    
    %% State Variables
    Px = x(1);
    Py = x(2);
    Pz = x(3);
    P = [Px, Py, Pz]';
    Vx = x(4);
    Vy = x(5);
    Vz = x(6);
    V = [Vx, Vy, Vz]';
    rx = x(7);
    ry = x(8);
    rz = x(9);
    r = [rx, ry, rz]';
    egx = x(10);
    egy = x(11);
    egz = x(12);
    eg = [egx, egy, egz]';
    eax = x(13);
    eay = x(14);
    eaz = x(15);
    ea = [eax, eay, eaz]';
    lgx = x(16);
    lgy = x(17);
    lgz = x(18);
    lg = [lgx, lgy, lgz]';
    
    %% Pre-processing cross product
    F = skew(f);
    W = skew(w);
    
    %% Transition Model
    dP = V;
    dV = -R*F*r + R*ea;
    dr = -W*r + eg;
    
    xf = [P+dP; V+dV; r+dr; eg; ea; lg];
    
    
    
    