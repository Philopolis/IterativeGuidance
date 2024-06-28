function Control = EquationsF(Rc, R, Vc, V, g, tg, F1, F2, F3, F4, Task, Control)
bar_psi_zeta = asin((V(3) - Vc(3) + g(3) * tg) / F1);
E = F3 * cos(bar_psi_zeta);
G = F4 * cos(bar_psi_zeta);
H = Rc(3) - R(3) - V(3) * tg - g(3) * tg^2 / 2 + F3 * sin(bar_psi_zeta);
Deltae = F2 * E - F1 * G;
if tg <= Task.Guidance.T1
    e1 = 0;
    e2 = 0;
else
    e1 = F2 * H / Deltae;
    e2 = F1 * H / Deltae;
end
Control.bar_psi_zeta = bar_psi_zeta;
Control.e1 = e1;
Control.e2 = e2;
end

