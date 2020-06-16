function kf = update_z(kf, ins, gnss, vbf, update_mode)
    %UPDATE_Z Summary of this function goes here
    %   Detailed explanation goes here
    dt = 0.01;
    CTMbeta = [cos(ins.beta) sin(ins.beta) 0;
               -sin(ins.beta) cos(ins.beta) 0;
               0 0 1];
    if nargin<5, update_mode = 'nhc';end
        if strcmp(update_mode, 'nhc')
            zvb = vbf.CTMab*(ins.CTMbn'*ins.v) + skew(vbf.w)*vbf.lo - [norm(ins.v);0;0];
            kf.z = [zvb];
        elseif strcmp(update_mode, 'horizontal')
            zvb = vbf.CTMab*(ins.CTMbn'*ins.v) + skew(vbf.w)*vbf.lo - [norm(ins.v);0;0];
            zpsi = vbf.CTMab*(ins.w) - [0;0;sign(ins.w(3))*norm(ins.w)];
            kf.z = [zvb;
                    zpsi];
        elseif strcmp(update_mode, 'kinematic')
            zvb = vbf.CTMab*(ins.CTMbn'*ins.v) + skew(vbf.w)*vbf.lo - [norm(ins.v);0;0];
            % zvb = vbf.vb + skew(vbf.w)*vbf.lo-[norm(ins.v);0;0];
            zpsi = vbf.CTMab*(ins.w) - [0;0;sign(ins.w(3))*norm(ins.w)];
            % if (kf.dt == 0)
            zv3 = vbf.dv/dt-[ins.dV/dt, (-norm(ins.v)*ins.dbeta)/dt, 0]';
            zv3 = vbf.dv/dt-[vbf.dvins(1)/dt, (-vbf.va(1)*ins.dbeta)/dt, 0]';   
            % else
            %     zv3 = [0,0,0]';
            % end
            % zv3 = vbf.vb - vbf.CTMab*ins.CTMbn'*ins.v;

            kf.z = [zvb;
                    zpsi;
                    zv3];

            % dv êµ¬í•´?„œ deltaV, beta?— ?˜?•œ ê°??†?„ë¡? [ax_, ay_, 0] ?— ë§¤ì¹­?• ê²? (plannar dynamics)

        elseif strcmp(update_mode, 'zaru')
        end
    end
    