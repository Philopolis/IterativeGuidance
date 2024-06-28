%% 计算当地恒星时
function LST = LocalSiderealTime(y,m,d,UT,EL)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数说明
% J0 - 世界时零时刻的儒略日
% y - 1901 至 2099 之间的年份
% m - 1 至 12 月
% d - 1 至 31 日
% UT - 世界时
% EL - 东经（角度）
% T0 - 自 2000 年的儒略世纪（角度）
% G0 - 世界时零时格林威治恒星时（角度）
% GST - 格林威治恒星时（角度）
% 需要函数文件 JulianDay.m 支持
%
% References:
% Curtis, H. D. (2021). Orbital Mechanics for Engineering Students: Fourth Edition. Butterworth-Heinemann.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 计算儒略日
J0 = JulianDay(y,m,d);
T0 = (J0 - 2451545)/36525;

% 计算格林威治恒星时
G0 = 100.4606184 + 36000.77004 * T0 + 0.000387933 * T0^2 - 2.583e-8 * T0^3;
G0 = ZeroTo360(G0);
GST = G0 + 360.98564724 * UT/24;

% 计算当地恒星时
LST = GST + EL;
LST = LST - 360 * fix(LST/360);

%% 函数计算支持模块
    function dum = ZeroTo360(x)
        if x >= 360
            x = x - fix(x/360) * 360;
        elseif x < 0
            x = x - (fix(x/360) - 1) * 360;
        end
        dum = x;
    end
end

