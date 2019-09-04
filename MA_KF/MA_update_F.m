function [F, G] = update_F(ins, imu, vbf, update_mode)

    I = eye(3);
    O = zeros(3);

    wv = vbf.CTMab*ins.w;
    fv = vbf.CTMab*ins.f;
    if nargin<4, update_mode = 'nhc'; end    
        if strcmp(update_mode, 'nhc')
            F = [O, O; O, O];
            G = [I, O; O, I];
        elseif strcmp(update_mode, 'horizontal')
            F = [O, O; O, O];
            G = [I, O; O, I];
        elseif strcmp(update_mode, 'kinematic')
            F = [O, O, O;
                 O, O, O;
                 -skew(vbf.vb)*skew(wv)+skew(fv), O, -skew(wv)];
            G = [I, O, O; O, I, O; O, O, I];
        elseif strcmp(update_mode, 'zaru')
        end
    end