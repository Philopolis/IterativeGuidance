%% 四阶龙格库塔法
function [X, t, control] = RK4(X0, control0, t0, tf, h, mat)
n = floor((tf - t0) / h);
t = zeros(1, n + 1);
X = zeros(length(X0), n + 1);
control = zeros(2, n + 1);
t(1) = t0;
X(:, 1) = X0;
control(:, 1) = control0;
for i = 1:n
    K1 = Equations(X(:, i), t(i), mat);
    K2 = Equations(X(:, i) + K1 * h/2, t(i) + h/2, mat);
    K3 = Equations(X(:, i) + K2 * h/2, t(i) + h/2, mat);
    K4 = Equations(X(:, i) + K3 * h, t(i) + h, mat);
    X(:, i + 1) = X(:, i) + (K1 + 2 * K2 + 2 * K3 + K4) * h/6;
    t(i + 1) = t(i) + h;
    varphi_xi = mat.Control.bar_varphi_xi + mat.Control.k2 * t(i + 1) - mat.Control.k1;
    psi_zeta = mat.Control.bar_psi_zeta + mat.Control.e2 * t(i + 1) - mat.Control.e1;
    U = mat.Q.xieta_A' * [cos(varphi_xi) * cos(psi_zeta); sin(varphi_xi) * cos(psi_zeta); -sin(psi_zeta)];
    psi = asin(-U(3));
    varphi = atan2(U(2), U(1));
    % sinPhi = U(2) / cos(psi);
    % if sinPhi >= 0
    %     varphi = acos(U(1) / cos(psi));
    % else
    %     varphi = -acos(U(1) / cos(psi));
    % end
    control(1, i + 1) = varphi;
    control(2, i + 1) = psi;
end
end

