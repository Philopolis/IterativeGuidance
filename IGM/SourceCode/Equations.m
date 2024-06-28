%% 动力学模型
function dot_state = Equations(state, t, mat)
%% 相关变量提取
R = state(1:3);
m = state(7);
mu = mat.Task.Earth.mu * 1e9;
varphi_xi = mat.Control.bar_varphi_xi + mat.Control.k2 * t - mat.Control.k1;
psi_zeta = mat.Control.bar_psi_zeta + mat.Control.e2 * t - mat.Control.e1;
g = - mu / norm(R)^3 * R;
U = mat.Q.xieta_A' * [cos(varphi_xi) * cos(psi_zeta); sin(varphi_xi) * cos(psi_zeta); -sin(psi_zeta)];
%% 动力学方程计算
dot_state = state * 0;
dot_state(1) = state(4);
dot_state(2) = state(5);
dot_state(3) = state(6);
dot_state(4) = g(1) + mat.Task.Vehicle.T_vac / m * U(1);
dot_state(5) = g(2) + mat.Task.Vehicle.T_vac / m * U(2);
dot_state(6) = g(3) + mat.Task.Vehicle.T_vac / m * U(3);
dot_state(7) = -mat.Task.Vehicle.m_s;
end

