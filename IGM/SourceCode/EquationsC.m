%% 方程组 c
function [F1, F2, F3, F4] = EquationsC(tau, tg, Task)
F1 = Task.Vehicle.Ve * log(tau / (tau - tg));
F2 = F1 * tau - Task.Vehicle.Ve * tg;
F3 = F1 * tg - F2;
F4 = F3 * tau - Task.Vehicle.Ve * tg^2 / 2;
end

