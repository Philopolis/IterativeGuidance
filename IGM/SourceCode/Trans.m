classdef Trans
    % TRANS a matlab class for coordinate transformation
    % Trans类包含了坐标转换相关函数，坐标转换（coordinate transformation）在航空航天、机器人等领域被广泛使用
    
    properties
        
    end
    
    methods(Static)  % 声明静态方法（Static Methods），静态方法不需要类的对象obj作为参数
        function mx = Mx(alpha)
            % 绕x轴旋转的alpha弧度的坐标转换矩阵
            mx = [1,      0     ,     0     ;
                  0,  cos(alpha), sin(alpha);
                  0, -sin(alpha), cos(alpha)];
        end
        function my = My(alpha)
            % 绕y轴旋转的alpha弧度的坐标转换矩阵
            my = [cos(alpha), 0 ,-sin(alpha);
                       0    , 1 ,     0     ;
                  sin(alpha), 0 , cos(alpha)];
        end
        function mz = Mz(alpha)
            % 绕z轴旋转的alpha弧度的坐标转换矩阵
            mz = [ cos(alpha), sin(alpha), 0;
                  -sin(alpha), cos(alpha), 0;
                       0     ,     0,      1;];
        end
    end
end

