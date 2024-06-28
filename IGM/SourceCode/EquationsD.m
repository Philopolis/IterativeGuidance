%% 方程组 d
function [g, Rc, Vc, R, V, Q] = EquationsD(phi1, phi2, R0, V0, Task, Q)
% 地心角
phi = phi1 + phi2;
omega = Task.Orbit.omega;
f = phi - omega;
a = Task.Orbit.a * 1e3;
e = Task.Orbit.ecc;
mu = Task.Earth.mu * 1e9;
% 制导坐标系下入轨点状态参数
Rc = [0; a * (1 - e^2) / (1 + e * cos(f)); 0];
Vc = sqrt(mu / (a * (1 - e^2))) * [1 + e * cos(f); e * sin(f); 0];
% 变换矩阵
Q.xieta_A = Trans.Mz(-phi) * Q.A_Omega';
% 制导坐标系下瞬时点状态参数
R = Q.xieta_A * R0;
V = Q.xieta_A * V0;
% 常值引力假设
g0 = -mu / norm(R)^3 * R;
gc = -mu / norm(Rc)^3 * Rc;
g = (g0 + gc) / 2;
end

