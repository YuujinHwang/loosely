function g_n = gravity(lat, h)
%GRAVITY : calulate gravity vector in current Local area

h = abs(h);
sin1 = sind(lat);
sin2 = sind(2.*lat);

g0 = 9.780318 * ( 1 + 5.3024e-03.*(sin1).^2 - 5.9e-06.*(sin2).^2 );

[RM, RN] = radius(lat);
R0 = sqrt(RN.*RM);

g = (g0 ./ (1+(h./R0)).^2);

g_n = [0, 0, g]';

end

