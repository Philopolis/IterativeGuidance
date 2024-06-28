%% 方程组 e
function Control = EquationsE(Rc, R, Vc, V, g, tg, F1, F2, F3, F4, Task)
bar_varphi_xi = atan2(Vc(2) - V(2) - g(2) * tg, Vc(1) - V(1) - g(1) * tg);
P = F3 * cos(bar_varphi_xi);
Q = F4 * cos(bar_varphi_xi);
S = Rc(2) - R(2) - V(2) * tg - g(2) * tg^2 / 2 - F3 * sin(bar_varphi_xi);
Deltak = F1 * Q - F2 * P;
if tg <= Task.Guidance.T2
    k1 = 0;
    k2 = 0;
else
    k1 = F2 * S / Deltak;
    k2 = F1 * S / Deltak;
end
Control.bar_varphi_xi = bar_varphi_xi;
Control.k1 = k1;
Control.k2 = k2;
end

