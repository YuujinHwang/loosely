function H = update_H(ins, gnss, vbf, update_mode)
    %UPDATE_H Summary of this function goes here
    %   Detailed explanation goes here
    O = zeros(3);
    I = eye(3);
    lg = gnss.lg;
    w = ins.w;
    CTMbn = ins.CTMbn;
    CTMab = vbf.CTMab;
    vnhc = [vbf.v;0;0];
    if nargin<4, update_mode = 'nhc'; end    
        if strcmp(update_mode, 'nhc')
            Hvb = [skew(CTMab*(CTMbn'*ins.v+skew(ins.w)*vbf.lo)), CTMab*skew(ins.w)];
            H = Hvb;
        elseif strcmp(update_mode, 'horizontal')
            Hvb = [skew(CTMab*(CTMbn'*ins.v+skew(ins.w)*vbf.lo)), CTMab*skew(ins.w)];
            Hpsi = [skew(vbf.CTMab*(ins.CTMbn'*ins.w)), O];
            H = [Hvb;
                Hpsi;];
        elseif strcmp(update_mode, 'kinematic')
            Hvb = [skew(CTMab*(CTMbn'*ins.v+skew(ins.w)*vbf.lo)), CTMab*skew(ins.w), O];
            Hpsi = [skew(vbf.CTMab*(ins.CTMbn'*ins.w)), O, O];
            Hkin = [-skew(vbf.CTMab*ins.CTMbn'*ins.v), O, O];
            
            % Hkin = [O, O, -skew(vbf.CTMab*ins.CTMbn'*ins.v)];
            H = [Hvb;
                Hpsi;
                Hkin];
        elseif strcmp(update_mode, 'zaru')
        end
    end
    