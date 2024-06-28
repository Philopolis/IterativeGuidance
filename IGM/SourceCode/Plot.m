%% 绘制图像
function Plot(structCal)
% 曲线参数
global degree;
LW = 1.0; % 线粗
FS = 10; % 字号
EFN = 'Times New Roman'; % 英文字体

figure(1)
plot3(structCal.A.x, structCal.A.y, structCal.A.z, 'black-', 'LineWidth', LW);
hold on;
plot3(structCal.A.x(end), structCal.A.y(end), structCal.A.z(end), 'blacko', 'LineWidth', LW);
xlabel('{\it x} /m', 'FontName',EFN, 'FontSize', FS);
ylabel('{\it y} /m', 'FontName',EFN, 'FontSize', FS);
zlabel('{\it z} /m', 'FontName',EFN, 'FontSize', FS);
title('地心发射惯性坐标系火箭轨迹', 'FontSize', FS);
grid on;
legend('', '实际入轨点');

figure(2)
plot(structCal.t(2:end), structCal.varphi(2:end) / degree, 'black-', 'LineWidth', LW);
xlabel('{\it t} /s', 'FontName',EFN, 'FontSize', FS);
ylabel('{\it φ} /°', 'FontName',EFN, 'FontSize', FS);
title('俯仰角', 'FontSize', FS);

figure(3)
plot(structCal.t(2:end), structCal.psi(2:end) / degree, 'black-', 'LineWidth', LW);
xlabel('{\it t} /s', 'FontName',EFN, 'FontSize', FS);
ylabel('{\it \psi} /°', 'FontName',EFN, 'FontSize', FS);
title('偏航角', 'FontSize', FS);

figure(4)
plot3(structCal.I.x, structCal.I.y, structCal.I.z, 'black-', 'LineWidth', LW);
hold on;
plot3(structCal.I.x(end), structCal.I.y(end), structCal.I.z(end), 'blacko', 'LineWidth', LW);
plot3(structCal.real.x, structCal.real.y, structCal.real.z, 'black--', 'LineWidth', LW - 0.5);
plot3(structCal.Task.Orbit.x, structCal.Task.Orbit.y, structCal.Task.Orbit.z, 'black:', 'LineWidth', LW);
xlabel('{\it x} /km', 'FontName',EFN, 'FontSize', FS);
ylabel('{\it y} /km', 'FontName',EFN, 'FontSize', FS);
zlabel('{\it z} /km', 'FontName',EFN, 'FontSize', FS);
legend('火箭轨迹', '实际入轨点', '实际轨道', '任务轨道');
title('地心赤道坐标系', 'FontSize', FS);
grid on;
end

