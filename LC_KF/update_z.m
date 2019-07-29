function kf = update_z(kf, ins, gnss, vbf, update_mode)
%UPDATE_Z Summary of this function goes here
%   Detailed explanation goes here
if nargin<5, update_mode = 'all';end
    if strcmp(update_mode, 'all')
        zp = ins.p - gnss.p + ins.CTMbn*gnss.lg';
        zv = ins.v - gnss.v + ins.CTMbn*skew(ins.w)*gnss.lg';
        % zvb = vbf.dv-[norm(ins.v)-vbf.v;0;0];
        % zvb = vbf.CTMab*ins.CTMbn'*ins.v;
        kf.z = [zp;zv];
    elseif strcmp(update_mode, 'zupt')

    elseif strcmp(update_mode, 'zaru')
    end
end

