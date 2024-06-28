function [Task, Q] = TaskParameters
global degree mu
%% 目标轨道及入轨要求
mu = 398600.44; % 地球引力常数 km^3/s^2
Task.Orbit.a = 6622.78534; % 半长轴 km
Task.Orbit.ecc = 0; % 偏心率
RA = (-9.964 + 360) * degree; % 升交点赤经 rad
Task.Orbit.RA = RA;
incl = 68.846 * degree; % 轨道倾角 rad
Task.Orbit.incl = incl;
Task.Orbit.omega = 0 * degree; % 近地点幅角 rad
Task.Orbit.f = 58.015 * degree; % 入轨点对应的真近点角
Task.Orbit.theta_HC = 0 * degree; % 入轨点当地速度倾角
Task.Orbit.h = sqrt(mu * Task.Orbit.a * (1 - Task.Orbit.ecc^2));
%% 地理常数
Task.Earth.mu = mu;
R = 6371; % 地球半径 km
B0 = 41.1906 * degree; % 发射点地理纬度
Task.Launch.B0 = B0;
alpha_e = 1 / 298.257; % 地球扁率
mu0 = alpha_e * sin(2 * B0); % 引力方向与地心矢径的夹角
lambda0 = 100 * degree; % 发射场地理经度
phi0 = atan(tan(B0) * (1 - alpha_e)^2); % 发射点的地心纬度
A0 = asin(cos(incl)/cos(phi0)); % 发射方位角
Task.Launch.A0 = A0;
Task.Launch.Rl = [-R * sin(mu0) * cos(A0); R * cos(mu0); R * sin(mu0) * sin(A0)] * 1e3; % 发射点地心矢径在发射惯性坐标系中的分量
year = 2019; % 发射时间 以此日作为发射时间是为感谢鲁鹏的工作
month = 5;
day = 17;
UT = 4.78; % 发射时世界时（Greenwich 时间）
LST = LocalSiderealTime(year,month,day,UT,lambda0); % 发射时发射点当地恒星时（赤经）
Task.Launch.LST = LST;
%% 运载火箭参数
Task.Vehicle.m0 = 86500; % 开始迭代制导时火箭质量 kg
T_vac = 8e5; % 火箭真空推力
Task.Vehicle.T_vac = T_vac;
m_s = 272; % 秒耗量 kg/s
Task.Vehicle.m_s = m_s;
Task.Vehicle.Ve = T_vac / m_s; % 喷气速度 m/s
Task.Vehicle.DeltaT = 1; % 导航周期 s
%% 迭代制导控制参数
Task.Guidance.T1 = 0; % e1 = e2 = 0
Task.Guidance.T2 = 0; % k1 = k2 = 0
Task.Guidance.T3 = 0; % 常值偏航角 TODO
Task.Guidance.T4 = 0; % 常值俯仰角 TODO
%% 坐标系方向余弦阵
Q.I_Omega = Trans.Mz(pi/2 - RA) * Trans.My(pi - incl); % 地心升交点轨道坐标系到地心赤道坐标系
Q.A_I = Trans.My(-A0 - pi/2) * Trans.Mx(B0) * Trans.Mz(LST - pi/2); % 地心赤道坐标系到地心发射惯性系
Q.A_Omega = Q.A_I * Q.I_Omega; % 地心升交点轨道坐标系到地心发射惯性系
% Delta_lambda = -19.964 * degree;  % 升交点与发射点经度差(升交点-发射点)
% Q.Omega_A = Trans.My(incl) * Trans.Mz(-Delta_lambda) * Trans.My(-pi/2) * Trans.Mz(B0) * Trans.My(A0); % 地心发射惯性系到地心升交点轨道坐标系（绝对经差表示）
% Q.A_Omega = Q.Omega_A';
end

