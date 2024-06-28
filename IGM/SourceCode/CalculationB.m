%% 不动点迭代求解 phi2
function phi = CalculationB(v_xieta, tg, costheta_Hxieta, A3, B3, V, DV, Vc, phi1, phi2_0, Task)
a = Task.Orbit.a * 1e3;
e = Task.Orbit.ecc;
k = (v_xieta * tg * (costheta_Hxieta - 1) + A3 * cos(Task.Orbit.theta_HC) - B3 ) / tg^2;
func = @(phi) cos(Task.Orbit.theta_HC) / (a * (1 - e^2)) * (1 + e * cos(phi + phi1 - Task.Orbit.omega)) ...
              * (V * tg + B3 - k * tg * (V + DV - Vc));
phi = fixedPointIter(func, phi2_0, 1e-4);
end

