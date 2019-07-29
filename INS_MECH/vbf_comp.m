function vbf = vbf_comp(error_s, vbf, ins)

    % dv = error_s(19:21);
    dr = error_s(19:21);
    dlo = error_s(22:24);
    %% Velocity Correction
    vbf.v = norm(ins.v);
    vbf.lo = vbf.lo - dlo;
    %% Attitude Correction
    antm = [0 vbf.qua(3) -vbf.qua(2); -vbf.qua(3) 0 vbf.qua(1); vbf.qua(2) -vbf.qua(1) 0];
    vbf.qua = vbf.qua + 0.5 .* [vbf.qua(4)*eye(3) + antm; -1.*[vbf.qua(1) vbf.qua(2) vbf.qua(3)]] * dr(1:3);
    vbf.qua = vbf.qua / norm(vbf.qua);       % Brute-force normalization

    vbf.CTMab = Qua_to_CTM(vbf.qua);
    vbf.r = vbf.r + dr;