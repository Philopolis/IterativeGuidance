%% 方程组 a
function [phi1, v_xieta, costheta_Hxieta] = EquationsA(x, y, dot_x, dot_y)
r_xieta = sqrt(x^2 + y^2);
phi1 = atan2(x, y);
v_xieta = sqrt(dot_x^2 + dot_y^2);
costheta_Hxieta = abs(x * dot_y - dot_x * y) / (r_xieta * v_xieta);
end

