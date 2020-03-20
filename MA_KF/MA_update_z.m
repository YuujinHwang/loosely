function kf = update_z(kf, ins, gnss, vbf, update_mode)
    %UPDATE_Z Summary of this function goes here
    %   Detailed explanation goes here
    dt = 0.01;
    if nargin<5, update_mode = 'nhc';end
        if strcmp(update_mode, 'nhc')
            zvb = vbf.CTMab*(ins.CTMbn'*ins.v) + skew(vbf.w)*vbf.lo - [norm(ins.v);0;0];
            kf.z = [zvb];
        elseif strcmp(update_mode, 'horizontal')
            zvb = vbf.CTMab*(ins.CTMbn'*ins.v) + skew(vbf.w)*vbf.lo - [norm(ins.v);0;0];
            zpsi = vbf.CTMab*(ins.CTMbn'*ins.w) - [0;0;sign(ins.w(3))*norm(ins.w)];
            kf.z = [zvb;
                    zpsi];
        elseif strcmp(update_mode, 'kinematic')
            zvb = vbf.CTMab*(ins.CTMbn'*ins.v) + skew(vbf.w)*vbf.lo - [norm(ins.v);0;0];
            % zvb = vbf.vb + skew(vbf.w)*vbf.lo-[norm(ins.v);0;0];
            zpsi = vbf.CTMab*(ins.CTMbn'*ins.w) - [0;0;sign(ins.w(3))*norm(ins.w)];
            % if (kf.dt == 0)
            zv3 = [sum(ins.v.*ins.f)./norm(ins.v), -norm(ins.v)*ins.dbeta/dt, 0]'-vbf.dv/dt;
                
            % else
            %     zv3 = [0,0,0]';
            % end
            % zv3 = vbf.vb - vbf.CTMab*ins.CTMbn'*ins.v;

            kf.z = [zvb;
                    zpsi;
                    zv3];

            % dv 구해서 deltaV, beta에 의한 가속도로 [ax_, ay_, 0] 에 매칭할것 (plannar dynamics)

        elseif strcmp(update_mode, 'zaru')
        end
    end
    