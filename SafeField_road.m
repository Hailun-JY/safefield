%% Risk field of non-moving object
% road
clc
clear
close all

%% 参数初始化
% 参数定义
R = 1;
K1 = 3;
K2 = 2;
D = 3.5; % 代表某个区域的宽度，例如道路宽度、两车之间的安全距离或其他影响范围。D/2 和 3/2*D 是在计算该区域的上限和下限。
k = 4;

% 定义 x 和 y 的范围及网格步长
x_range = 0:0.01:10;
y_range = -D/2:0.01:(3/2)*D;
[X, Y] = meshgrid(x_range, y_range);

% 计算 d1 和 d2, 这两个距离帮助确定物体/车辆处于该区域的哪个位置，并依据这些距离计算它受到的场强或影响力。
d1 = Y - D/2; % 表示当前位置 Y 与区域上边界 D/2 之间的距离。如果车辆/物体在区域内（0 <= d1 <= D/2），这个距离用于计算场的强度。
d2 = D/2 - Y; % 表示当前位置 Y 与区域下边界 D/2 之间的距离。与 d1 类似，d2 用于计算车辆/物体距离区域下边界的场强度。

% 预分配 E1 和 E2
E1 = zeros(size(X));
E2 = zeros(size(X));

%% 计算两个不同区域的奉献场
% 计算 E1，表示在给定位置 (X, Y) 时，由于靠近路口一侧的影响（或能量）引发的场强。
% 当 d1（即 Y - D/2）位于某个区间（0 到 D/2）时，E1 按照 (d1^(k+1)) 的形式计算，即能量场随着 d1 的幂级数增长；
% 当 Y > D 时，E1 计算方式转换为一种减弱的形式，用 (3/2 * D - Y)^(k+1) 来表示距离更大时的能量分布。
E1 = K1 * R .* (d1.^(k+1)) .* (d1 >= 0 & d1 <= D/2) -  K1 * R .* ((3/2*D - Y).^(k+1)) .* (Y > D);

% 计算 E2，表示在给定位置 (X, Y) 时，由于远离路口一侧的影响（或能量）引发的场强。
% 当 d2（即 D/2 - Y）在某个范围内（0 到 D/2）时，E2 按照 (d2^(k+1)) 的形式计算，即在这个区域的场强随着 d2 的幂级数增长。
% 当 Y < 0 时，E2 计算方式改变为 (Y + D/2)^(k+1)，表示车辆远离特定位置时能量场的变化。
E2 = K2 * R .* (d2.^(k+1)) .* (d2 >= 0 & d2 <= D/2) -  K2 * R .* ((Y + D/2).^(k+1)) .* (Y < 0);

% 计算最终的 E
E = abs(E2 - E1); % 代表车辆处于不同位置时，所感受到的场强差异或影响力的变化。这通常应用于交通流、风险场或能量场建模等领域。

%% 绘制三维图形
figure;
mesh(X, Y, E);
xlabel('x [m]', 'Fontname', 'Times New Roman');
ylabel('y [m]', 'Fontname', 'Times New Roman');
xlim([0 10]);
ylim([-2 5]);
zlim([0 200]);
h = colorbar;
set(get(h, 'YLabel'), 'string', 'E_d [G N/kg]', 'Fontname', 'Times New Roman');

% 绘制侧视图
figure;
mesh(X, Y, E);
view([-90, 0]);
xlabel('x [m]', 'Fontname', 'Times New Roman');
ylabel('y [m]', 'Fontname', 'Times New Roman');
h = colorbar;
set(get(h, 'YLabel'), 'string', 'E_d [G N/kg]', 'Fontname', 'Times New Roman');



