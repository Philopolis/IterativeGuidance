%% 不动点迭代求解 tg
function tg = CalculationA(Vc, V0, g, tg0, tau, Task)
DeltaV = @(t) Vc - V0 - g * t;
func = @(t) tau * (1 - exp(-norm(DeltaV(t)) / Task.Vehicle.Ve));
tg = fixedPointIter(func, tg0, 1e-5);
end

