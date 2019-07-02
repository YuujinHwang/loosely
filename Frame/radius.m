function [RM, RN, RA] = radius(lat)
%RADIUS : calculate WGS84 radius of local area
if nargin < 1
    RM = 6378137.0;RN = 6378137.0;RA=6378137.0;
    return
end

    a = 6378137.0;
    e = 1/298.257223563;
    e2 = e.^2;
    frac = 1-e2.*(sind(lat)).^2;

    RM = a * (1-e)/((frac).^(1.5));
    RN = a/sqrt(frac);
    RA = RN*RM / sqrt((RN*sind(lat)).^2 + (RM*cosd(lat)).^2) ;

end

