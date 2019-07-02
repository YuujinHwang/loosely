function kf = update_z(kf, ins, gnss, update_mode)
%UPDATE_Z Summary of this function goes here
%   Detailed explanation goes here
if nargin<4, update_mode = 'all';end
    if strcmp(update_mode, 'all')
        zp = ins.p - gnss.p + ins.CTMbn*gnss.lg';
        zv = ins.v - gnss.v + ins.CTMbn*skew(ins.w)*gnss.lg';
        
        kf.z = [zp;zv];
    elseif strcmp(update_mode, 'zupt')

    elseif strcmp(update_mode, 'zaru')
    end
end

