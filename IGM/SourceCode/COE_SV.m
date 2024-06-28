%% 状态向量计算开普勒轨道要素
function COE = COE_SV(R,V)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数说明
% mu - 引力常数 km^3/s^2
% R,V - 状态向量 km,km/s
% H - 比角动量 km^2/s
% incl - 轨道倾角（弧度）
% N - 升交线矢量 km^2/s
% RA - 升交点赤经（弧度）
% E - 偏心率矢量
% eps - 低于该数值的偏心率会被考虑为0
% omega - 近地点幅角（弧度）
% TA - 真近点角（弧度）
% a - 半长轴 km
% COE - 开普勒轨道要素[h;e;RA;incl;omega;TA;a]
%
% References:
% Curtis, H. D. (2021). Orbital Mechanics for Engineering Students: Fourth Edition. Butterworth-Heinemann.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 初始化
global mu;
eps = 1.e-10;
r = norm(R);
v = norm(V);
v_r = dot(V,R)/r;

% 计算比角动量
H = cross(R,V);
h = norm(H);

% 计算轨道倾角
incl = acos(H(3)/h);

% 计算升交线矢量
N = cross([0;0;1],H);
n = norm(N);

% 计算升交点赤经
if n ~= 0
    RA = acos(N(1)/n);
    if N(2) < 0
        RA = 2 * pi - RA;
    end
else
    RA = 0;
end

% 计算偏心率矢量
E = 1/mu * ((v^2 - mu/r) * R - r * v_r * V);
e = norm(E);

% 计算近地点幅角
if n ~= 0
    if e > eps
        omega = acos(dot(N,E)/(n * e));
        if E(3) < 0
            omega = 2 * pi - omega;
        end
    else
        omega = 0;
    end
else
    omega = 0;
end

% 计算真近点角
if e > eps
    TA = acos(dot(E,R)/(e * r));
    if v_r < 0
        TA = 2 * pi - TA;
    end
else
    cp = cross(N,R);
    if cp(3) >= 0
        TA = acos(dot(N,R)/(n * r));
    else
        TA = 2 * pi - acos(dot(N,R)/(n * r));
    end
end

% 计算半长轴
a = h^2/mu * 1/(1 - e^2);

% 轨道要素汇总
COE = [h;e;RA;incl;omega;TA;a];
end

