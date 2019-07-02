function eul = CTM_to_Euler(CTM)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    phi = atan2(CTM(2,3), CTM(3,3));
    theta = -asin(CTM(1,3));
    psi = atan2(CTM(1,2),CTM(1,1));
    
    eul = [phi;theta;psi];
end

