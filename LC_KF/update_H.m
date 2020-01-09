function H = update_H(ins, gnss, vbf, CTMupdate_mode)
%UPDATE_H Summary of this function goes here
%   Detailed explanation goes here

lg = gnss.lg;
w = ins.w;
CTMbn = ins.CTMbn;
CTMab = vbf.CTMab;
vb = [vbf.v;0;0];

    if nargin<4, update_mode = 'all'; end

    O = zeros(3);
    I = eye(3);
    Lg = skew(lg);
    W = skew(w);

    if strcmp(update_mode, 'all')
        Hins = [I, O, CTMbn*Lg*CTMbn', O, O, CTMbn;
            O, I, -CTMbn*(Lg*W - W*Lg)*CTMbn', O, -CTMbn*Lg, CTMbn*W];
        % Hins = [I, O, CTMbn*Lg, O, O, CTMbn;
            % O, I, -CTMbn*W*Lg, O, -CTMbn*Lg, CTMbn*W];
        H = [Hins];
        % H = [I, O, O, O, O, CTMbn;
        %      O, I, O, O, O, O];
    elseif strcmp(update_mode, 'zupt')
        H = [I, O, -CTMbn*Lg, O, O, CTMbn];
    elseif strcmp(update_mode, 'zaru')
    end
end

