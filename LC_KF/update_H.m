function H = update_H(CTMbn, lg, w,update_mode)
%UPDATE_H Summary of this function goes here
%   Detailed explanation goes here

    if nargin<4, update_mode = 'all'; end

    O = zeros(3);
    I = eye(3);
    Lg = skew(lg);
    W = skew(w);

    if strcmp(update_mode, 'all')
        H = [I, O, -CTMbn*Lg, O, O, -CTMbn;
            O, I, CTMbn*(Lg*W - W*Lg), O, CTMbn*Lg, -CTMbn*W];
        % H = [I, O, O, O, O, CTMbn;
        %      O, I, O, O, O, O];
    elseif strcmp(update_mode, 'zupt')
        H = [I, O, -CTMbn*Lg, O, O, CTMbn];
    elseif strcmp(update_mode, 'zaru')

end

