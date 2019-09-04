function vbf = vbf_update(vbf, ins, dt)


    vbf.f = vbf.CTMab*(ins.CTMbn'*ins.f);
    vbf.w = vbf.CTMab*(ins.w);
    try
        vbf.dv = -skew(vbf.w)*vbf.vb + vbf.f;
        vbf.dv = vbf.dv*dt;
        vbf.vb = vbf.vb + vbf.dv;
    catch
        vbf.dv = ins.v;

    end
    % vbf.v = vbf.v + vbf.dv*dt;

    % a = vbf.dv(1);
    % b = vbf.dv(2);
    % c = vbf.dv(3);

    % d = norm(ins.v)-vbf.v;
    % vbf.res = d-a+vbf.r(2)*b-vbf.r(3)*c;
    % vbf.alpha1 = (a*vbf.r(3) - b)/c;
    % vbf.alpha2 = (a*vbf.r(2) + c)/b;

    % vbf.v = vbf.CTMab*ins.CTMbn'*ins.v;
