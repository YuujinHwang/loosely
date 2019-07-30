function qua = CTM_to_Qua(CTMbn)
    %UNTITLED3 Summary of this function goes here
    %   Detailed explanation goes here
        c11 = CTMbn(1,1);
        c12 = CTMbn(1,2);
        c13 = CTMbn(1,3);
        c21 = CTMbn(2,1);
        c22 = CTMbn(2,2);
        c23 = CTMbn(2,3);
        c31 = CTMbn(3,1);
        c32 = CTMbn(3,2);
        c33 = CTMbn(3,3);

        a = 0.5*sqrt(1+c11+c22+c33);
        b = (c32 - c23)/(4*a);
        c = (c13 - c31)/(4*a);
        d = (c21 - c12)/(4*a);

        qua = [b, c, d, a]';
    end
    
    