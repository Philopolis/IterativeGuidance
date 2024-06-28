function [zeropt, iteration] = fixedPointIter(func, x0, tol)
%FIXEDPOINTITER 不动点迭代法（Fixed-Point Iteration）解非线性方程组
%   迭代公式：x_{k+1} = g(x_{k})，收敛条件：g(x)的雅克比矩阵G(x)的谱半径小于1
% 
% Input：
%   func    迭代公式
%   x0      初始值
%   tol     精度
% Output:
%   zeropt      满足精度tol的近似根
%   iteration   迭代次数

    prev = x0;
    next = func(prev);
    iteration = 1;
    while abs(next - prev) >= tol && iteration < 50
        prev = next;
        next = func(prev);
        iteration = iteration + 1;
    end
    zeropt = next;
    if iteration >= 50
        error('Method fails to converge.');
    end
end

