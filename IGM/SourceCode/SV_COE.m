%% 开普勒轨道要素计算状态向量
function [R,V] = SV_COE(COE)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数说明
% mu - 引力常数 km^3/s^2
% -------------------------------------------------------------------------------------------------
% COE - 轨道六要素 [h;e;RA;incl;omega;TA] - ！！！！！！严格按照该顺序排列轨道要素！！！！！！！！！
% -------------------------------------------------------------------------------------------------
% Q - 近焦点向地心赤道坐标系变换矩阵
% 长度单位均为 km
%
% References:
% Curtis, H. D. (2021). Orbital Mechanics for Engineering Students: Fourth Edition. Butterworth-Heinemann.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 初始化
global mu;
h = COE(1);
e = COE(2);
RA = COE(3);
incl = COE(4);
omega = COE(5);
TA = COE(6);

% 计算近焦点状态向量
R_p = h^2/mu * 1/(1 + e * cos(TA)) * (cos(TA) * [1;0;0] + sin(TA) * [0;1;0]);
V_p = mu/h * (-sin(TA) * [1;0;0] + (e + cos(TA)) * [0;1;0]);

% 计算变换矩阵
R3_RA = [ cos(RA), sin(RA), 0;
         -sin(RA), cos(RA), 0;
                0,       0, 1];
R1_i = [1,          0,         0;
        0,  cos(incl), sin(incl);
        0, -sin(incl), cos(incl)];
R3_omega = [ cos(omega), sin(omega), 0;
            -sin(omega), cos(omega), 0;
                      0,          0, 1];
Q = (R3_omega * R1_i * R3_RA)';

% 计算状态向量
R = Q * R_p;
V = Q * V_p;
end

