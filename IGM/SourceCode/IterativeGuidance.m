%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 迭代制导仿真程序，导航输入以动力学方程积分求得的值代替
% 
% References:
% [1] 韩祝斋. 用于大型运载火箭的迭代制导方法[J]. 宇航学报, 1983, 4(1):12-24.
% [2] 李伟. 基于精确控制解的运载火箭迭代制导自适应性分析研究[D]. 哈尔滨工业大学, 2012.
% [3] 鲁鹏. 迭代制导总结. 北京理工大学, 2019.
% 
% 刘喆禹
% 2024.06.03
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% main func
close all; clear all; clc;
format compact; format long;
%% 预处理
global degree
degree = pi / 180;
% 装订参数
[Task, Q] = TaskParameters;

% 初始条件
R0 = [2.9058e5; 92126; -33401] + Task.Launch.Rl; % 初始位置在地心发射惯性坐标系中的分量 m
R_Omega0 = Q.A_Omega' * R0; % 初始位置在地心升交点轨道坐标系中的分量 m
V0 = [2815.75; 494.703; 70.953]; % 初始速度在发射惯性坐标系的初始速度向量 m/s
V_Omega0 = Q.A_Omega' * V0; % 初始速度在地心升交点轨道坐标系中的分量 m/s

varphi0 = 0 * degree; % 发射惯性坐标系下的初始俯仰角
psi0 = 0 * degree; % 发射惯性坐标系下的初始偏航角
U0 = [cos(varphi0) * cos(psi0); sin(varphi0) * cos(psi0); -sin(psi0)]; % 地心发射惯性系下，初始单位推力方向
W0 = Task.Vehicle.T_vac / Task.Vehicle.m0 * U0; % 初始视加速度
W_Omega0 = Q.A_Omega' * W0;

t0 = 0;
tg0 = Task.Vehicle.m0 * 0.8 / Task.Vehicle.m_s; % 估计剩余飞行时间初值
step = .01; % 动力学时间间隔（制导周期）

% 求解剩余飞行地心角
phi1_0 = atan2(R_Omega0(1), R_Omega0(2));
phi2_0 = Task.Orbit.omega + Task.Orbit.f - phi1_0;
[~, Rc, Vc, R, V, Q] = EquationsD(phi1_0, phi2_0, R0, V0, Task, Q); % 估计入轨点位置

% 初始状态量
state0 = [R0; V0; Task.Vehicle.m0];
control0 = [varphi0; psi0];

% 初始化储存数组
state_storage = zeros(7,ceil((tg0 - t0) / step));
t_storage = zeros(1,ceil((tg0 - t0) / step));
control_storage = zeros(2,ceil((tg0 - t0) / step));

% 赋初值
count = 1;
state_storage(:, count) = state0;
t_storage(count) = t0;
control_storage(:, count) = control0;
%% 迭代制导
while(1)
    % 迭代，启动！
    % 输入导航信息
    x_Omega = R_Omega0(1);
    y_Omega = R_Omega0(2);
    z_Omega = R_Omega0(3);
    v_x_Omega = V_Omega0(1);
    v_y_Omega = V_Omega0(2);
    v_z_Omega = V_Omega0(3);
    w_x_Omega = W_Omega0(1);
    w_y_Omega = W_Omega0(2);
    w_z_Omega = W_Omega0(3);

    [phi1, v_xieta, costheta_Hxieta] = EquationsA(x_Omega, y_Omega, v_x_Omega, v_y_Omega);

    [tau, tau_xieta] = EquationsB(w_x_Omega, w_y_Omega, w_z_Omega, Task);

    while(1)
        [~, ~, A3, ~] = EquationsC(tau_xieta, tg0, Task);
        [B1, ~, B3, ~] = EquationsC(tau, tg0, Task);

        % 数值求解 phi2；要求初值 phi2_0
        phi2 = CalculationB(v_xieta, tg0, costheta_Hxieta, A3, B3, norm(V0), B1, norm(Vc), phi1, phi2_0, Task);
        phi2_0 = phi2;
        [g, Rc, Vc, R, V, Q] = EquationsD(phi1, phi2, R0, V0, Task, Q);
        % 不动点求 tg；要求初值 tg0
        tg = CalculationA(Vc, V, g, tg0, tau, Task);

        if abs(tg - tg0) <= 1e-5
            break;
        else
            tg0 = tg;
        end
    end
    % 计算控制参数
    [B1, B2, B3, B4] = EquationsC(tau, tg, Task);
    Control = EquationsE(Rc, R, Vc, V, g, tg, B1, B2, B3, B4, Task); % varphi_xi
    Control = EquationsF(Rc, R, Vc, V, g, tg, B1, B2, B3, B4, Task, Control); % psi_eta
    mat.Task = Task;
    mat.Q = Q;
    mat.Control = Control;

    % 递推
    [state1, t1, control1] = RK4(state0, control0, 0, Task.Vehicle.DeltaT, step, mat);

    % 检查程序
    if(sum(isnan(state1)))
        error('程序出错，检查代码');
        % break;
    end

    % 迭代赋值
    l = length(t1);
    state0 = state1(:, end);
    control0 = control1(:,end);
    count = count + 1;

    % 记录数据
    state_storage(:, (count - 2) * (l - 1) + 1: (count - 1) * (l - 1) + 1) = state1;
    t1 = Task.Vehicle.DeltaT * (count - 2) * ones(1, l) + t1;
    t_storage((count - 2) * (l - 1) + 1: (count - 1) * (l - 1) + 1) = t1;
    control_storage(:, (count - 2) * (l - 1) + 1: (count - 1) * (l - 1) + 1) = control1;

    % 输出导航信息
    R0 = state0(1:3);
    V0 = state0(4:6);
    m0 = state0(7);
    varphi0 = control0(1);
    psi0 = control0(2);
    U0 = [cos(varphi0) * cos(psi0); sin(varphi0) * cos(psi0); -sin(psi0)];
    W0 = Task.Vehicle.T_vac / m0 * U0;

    R_Omega0 = Q.A_Omega' * R0;
    V_Omega0 = Q.A_Omega' * V0;
    W_Omega0 = Q.A_Omega' * W0;
    tg0 = tg - Task.Vehicle.DeltaT; % 瞬时点的 tg 初值

    % 仿真终止判断
    if  tg <= 1 || t_storage(end) >= Task.Vehicle.m0 / Task.Vehicle.m_s
        break;
    end
end

% 清除多余存储空间
state_storage(:, (count - 1) * (l - 1) + 2:end) = [];
t_storage((count - 1) * (l - 1) + 2:end) = [];
control_storage(:, (count - 1) * (l - 1) + 2:end) = [];
%% 数据处理与绘图
% 获取实际入轨轨道要素
R_I = Q.I_Omega * Q.A_Omega' * state_storage(1:3, end) * 1e-3;
V_I = Q.I_Omega * Q.A_Omega' * state_storage(4:6, end) * 1e-3;
COE = COE_SV(R_I,V_I);
result.h = COE(1);
result.ecc = COE(2);
result.RA = COE(3);
result.incl = COE(4);
result.omega = COE(5);
result.TA = COE(6);
result.a = COE(7);
% 误差计算
result.error.absolute.a = abs(Task.Orbit.a - result.a);
result.error.relative.a = result.error.absolute.a / Task.Orbit.a;
result.error.absolute.ecc = abs(Task.Orbit.ecc - result.ecc);
result.error.relative.ecc = result.error.absolute.ecc / Task.Orbit.ecc;
result.error.absolute.RA = abs(Task.Orbit.RA - result.RA);
result.error.relative.RA = result.error.absolute.RA / Task.Orbit.RA;
result.error.absolute.incl = abs(Task.Orbit.incl - result.incl);
result.error.relative.incl = result.error.absolute.incl / Task.Orbit.incl;
result.error.absolute.omega = abs(Task.Orbit.omega - result.omega);
result.error.relative.omega = result.error.absolute.omega / Task.Orbit.omega;
result.error.absolute.TA = abs(Task.Orbit.f - result.TA);
result.error.relative.TA = result.error.absolute.TA / Task.Orbit.f;
% 绘图
[structCal] = DataTreatment(state_storage, t_storage, control_storage, Task, Q, result);
Plot(structCal);