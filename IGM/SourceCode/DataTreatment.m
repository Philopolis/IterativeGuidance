%% 数据处理模块
function [structState] = DataTreatment(state_storage, t_storage, control_storage, Task, Q, result)
N = length(t_storage);
Index = 1:N;
% Index = [1:n:N,N]; 数据点间隔n的数组

% 数据处理
structState.Q = Q;
structState.Task = Task;
structState.t = t_storage;
structState.A.x = state_storage(1, :);
structState.A.y = state_storage(2, :);
structState.A.z = state_storage(3, :);
structState.v_x = state_storage(4, :);
structState.v_y = state_storage(5, :);
structState.v_z = state_storage(6, :);
structState.varphi = control_storage(1, :);
structState.psi = control_storage(2, :);

structState.I.x = zeros(length(Index), 1);
structState.I.y = zeros(length(Index), 1);
structState.I.z = zeros(length(Index), 1);
structState.I.v_x = zeros(length(Index), 1);
structState.I.v_y = zeros(length(Index), 1);
for i = 1:length(Index)
    R_I = Q.I_Omega * Q.A_Omega' * state_storage(1:3, Index(i)) * 1e-3;
    structState.I.x(i) = R_I(1);
    structState.I.y(i) = R_I(2);
    structState.I.z(i) = R_I(3);
    V_I = Q.I_Omega * Q.A_Omega' * state_storage(4:6, Index(i)) * 1e-3;
    structState.I.v_x(i) = V_I(1);
    structState.I.v_y(i) = V_I(2);
    structState.I.v_z(i) = V_I(3);
end

n = 1000;
COE = zeros(6, n);
for i = 0 : 1 / n : 1
    k = floor(n * i + 1);
    TA = i * 2 * pi;
    COE(:, k) = [result.h; result.ecc; result.RA; result.incl; result.omega; TA];
    [R, ~] = SV_COE(COE(:, k));
    structState.real.x(k) = R(1);
    structState.real.y(k) = R(2);
    structState.real.z(k) = R(3);
end

COE = zeros(6, n);
for i = 0 : 1 / n : 1
    k = floor(n * i + 1);
    TA = i * 2 * pi;
    COE(:, k) = [structState.Task.Orbit.h; structState.Task.Orbit.ecc; structState.Task.Orbit.RA; structState.Task.Orbit.incl; structState.Task.Orbit.omega; TA];
    [R, ~] = SV_COE(COE(:, k));
    structState.Task.Orbit.x(k) = R(1);
    structState.Task.Orbit.y(k) = R(2);
    structState.Task.Orbit.z(k) = R(3);
end
end


