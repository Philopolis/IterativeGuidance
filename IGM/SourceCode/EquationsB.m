%% 方程组 b
function [tau, tau_xieta] = EquationsB(W_x, W_y, W_z, Task)
W_xieta = sqrt(W_x^2 + W_y^2);
W = sqrt(W_xieta^2 + W_z^2);
tau = Task.Vehicle.Ve / W;
tau_xieta = Task.Vehicle.Ve / W_xieta;
end

