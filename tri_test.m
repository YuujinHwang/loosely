function err_dist = tri_test(start_point, primary_len, secondary_len, beta, coeff)
%TRI_TEST 이 함수의 요약 설명 위치
%   자세한 설명 위치

start_x = start_point(1);
start_y = start_point(2);

tri_x = -primary_len*sin(pi/2-beta);
tri_y = start_y - primary_len*cos(pi/2-beta);

end_x = 0;
end_y = start_y + secondary_len;

%%%test%%%
% coeff = 1/primary_len;
%%%%%%%%%%

betab = beta + coeff*secondary_len

third_len = sqrt((tri_x -end_x).^2 + (tri_y - end_y).^2);

tri2_x = end_x - third_len*cos(betab);
tri2_y = end_y - third_len*sin(betab);

plot([start_x, tri_x],[start_y,tri_y], [start_x,end_x], [start_y, end_y], [end_x, tri2_x], [end_y, tri2_y]);


err_dist = sqrt((tri_x-tri2_x)^2 + (tri_y-tri2_y).^2);

end

