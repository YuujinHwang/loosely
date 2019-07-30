function [F, G] = update_F(ins, imu, vbf)

    I = eye(3);
    O = zeros(3);

    
    F = [O, O; O, O];
    G = [I, O; O, I];