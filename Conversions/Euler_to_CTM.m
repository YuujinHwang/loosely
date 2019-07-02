function CTM = Euler_to_CTM(eul)
%EULER_TO_CTM Convert Euler Angle to Matrix
%   Rotation Order
%   1 : X
%   2 : Y
%   3 : Z
    x = eul(1);
    y = eul(2);
    z = eul(3);
    
    Cx = [1,0,0;0,cos(x),sin(x);0,-sin(x),cos(x)];
    Cy = [cos(y),0,-sin(y);0,1,0;sin(y),0,cos(y)];
    Cz = [cos(z),sin(z),0;-sin(z),cos(z),0;0,0,1];
    
    CTM = Cx*Cy*Cz;
    % sin_phi = sin(eul(1));
    % cos_phi = cos(eul(1));
    % sin_theta = sin(eul(2));
    % cos_theta = cos(eul(2));
    % sin_psi = sin(eul(3));
    % cos_psi = cos(eul(3));

    % % Calculate coordinate transformation matrix using (2.22)
    % C(1,1) = cos_theta * cos_psi;
    % C(1,2) = cos_theta * sin_psi;
    % C(1,3) = -sin_theta;
    % C(2,1) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
    % C(2,2) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
    % C(2,3) = sin_phi * cos_theta;
    % C(3,1) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
    % C(3,2) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
    % C(3,3) = cos_phi * cos_theta;
    % CTM = C;
end

