%% 风险场模型用于交叉口驶入
% Hailun Zhang 2024 版权所有
clc;
clear;
close all;

%% 数据输入
% 每行代表一个对象，列依次为:
%  1. 编号id
%  2. x位置(m)
%  3. y位置(m)
%  4. 质量(kg)
%  5. 车速(m/s)
%  6. 航偏角(°)
%  7. 可行的最大速度(km/h)
%  8. 车辆长度(m)
%  9. 车辆宽度(m)
% 10. 车头时距(s)
% 11. 车道宽度(m)

Obj_Info = [
    1, -30, 0, 1500, 20, 0, 200, 5, 2, 1.5, 3.5;
    2,  20, 0, 1500, 20, 0, 200, 5, 2, 1.5, 3.5;
    3,  40, -7, 1500, 10, 0, 200, 5, 2, 1.5, 3.5;
    4, -20, -7, 1000, 10, 0, 200, 5, 2, 1.5, 3.5;
    5,  40, 3.5, 1500, 0, 0, 200, 5, 2, 1.5, 3.5
];

resolution = 0.05;  % 网格分辨率
x0 = -50:resolution:50;  % x坐标范围
y0 = -10:resolution:10;  % y坐标范围
range = ones(length(y0), length(x0));  % 创建网格范围

%% 计算平均车速及交通环境的势能
car_obj_id = find(Obj_Info(:, 4) > 500);  % 筛选质量大于500kg的车辆
car_obj_id = car_obj_id(Obj_Info(car_obj_id, 5) > 0);  % 筛选车速大于0的车辆
car_obj_id = car_obj_id(Obj_Info(car_obj_id, 5) < 30);  % 筛选车速小于30 m/s的车辆
v_ave = mean(Obj_Info(car_obj_id, 5));  % 计算平均车速

% 初始化势能U
U = 0;

% 遍历所有物体，计算势能叠加
for j = 1:max(Obj_Info(:, 1))
    tmpU = RiskField(Obj_Info(j, :), x0, y0, resolution, v_ave);
    U = U + tmpU;
end

%% 调整势能显示的最大值
Ej_all = 1/2 * Obj_Info(car_obj_id, 4) .* Obj_Info(car_obj_id, 5) .^ 2;  % 动能
Aj_all = Obj_Info(car_obj_id, 10) .* Obj_Info(car_obj_id, 5) + Obj_Info(car_obj_id, 8);  % 半长轴
distance = 10;  % 关注距离
Ej_max = max(Ej_all) * Aj_all(Ej_all == max(Ej_all)) / distance;
Ej_max = Ej_max(1);  % 获取最大值

% 将超过最大势能的值设为最大势能
U(U > Ej_max) = Ej_max;

%% 绘图
% 绘制三维网格
figure(1);
mesh(x0 .* range, y0' .* range, U);
hold on;

% 绘制车道线
lw = Obj_Info(1, 11);  % 车道宽度
lx = linspace(-50, 50, 1000);
ly1 = ones(size(lx)) * (-lw / 2);
ly2 = ones(size(lx)) * (lw / 2);
ly3 = ones(size(lx)) * (-lw / 2 - lw);
ly4 = ones(size(lx)) * (lw / 2 + lw);
lz = ones(size(lx)) * Ej_max;

% 绘制虚线
plot3(lx, ly1, lz, 'w--', 'linewidth', 2);
plot3(lx, ly2, lz, 'w--', 'linewidth', 2);
plot3(lx, ly3, lz, 'w--', 'linewidth', 2);
plot3(lx, ly4, lz, 'w--', 'linewidth', 2);

colormap(jet);
set(gca, 'DataAspectRatio', [1 1 Ej_max / 20]);

% 绘制等高线图及势场梯度图
figure(2);
h = contour(x0 .* range, y0' .* range, U);
colormap(jet);
set(gca, 'DataAspectRatio', [1 1 Ej_max / 20]);

% 计算并绘制势场梯度
inc = 10;
[dx, dy] = gradient(U(1:inc:end, 1:inc:end), 1, 1);
hold on;
XX = x0 .* range;
YY = y0' .* range;
quiver(XX(1:inc:end, 1:inc:end), YY(1:inc:end, 1:inc:end), -dx, -dy, 5);

% 绘制第二个等高线图及势场梯度
figure(3);
h = contour(x0 .* range, y0' .* range, U);
colormap(jet);
set(gca, 'DataAspectRatio', [1 1 Ej_max / 20]);

[dx, dy] = gradient(U);
hold on;
quiver(XX, YY, -dx, -dy, 1);

%% 风险场函数
% 计算对象的风险场
% 输入：
%   XX：对象信息
%   x0, y0：网格坐标
%   resolution：分辨率
%   v_ave：平均速度
% 输出：
%   U：势能
function U = RiskField(XX, x0, y0, resolution, v_ave)
    % 提取对象信息
    xj = XX(1, 2);  % x位置
    yj = XX(1, 3);  % y位置
    m = XX(1, 4);  % 质量
    v = XX(1, 5);  % 速度
    theta = XX(1, 6) / 360 * 2 * pi;  % 航偏角
    v_max = XX(1, 7) / 3.6;  % 最大速度（转换为m/s）
    l1 = XX(1, 8) / 2;  % 半车长
    l2 = XX(1, 9) / 2;  % 半车宽
    THW = XX(1, 10);  % 车头时距
    lw = XX(1, 11);  % 车道宽度

    % 确保速度不低于合理值
    if v < lw / THW
        v = lw / THW;
    end

    % 计算车辆的A和B参数
    A = THW * v + l1;
    B = lw + l2;
    A_ave = THW * abs(v_ave - v) + l1;
    B_ave = B;

    % 创建坐标差矩阵
    x01 = (x0 - xj) .* ones(length(y0), length(x0));
    y01 = ones(length(y0), length(x0)) .* (y0' - yj);

    % 旋转坐标系
    x = x01 .* cos(theta) + y01 .* sin(theta);
    y = y01 .* cos(theta) - x01 .* sin(theta);

    % 计算距离和角度
    r = sqrt(x.^2 + y.^2);
    sin_theta = abs(y) ./ r;
    sin_theta(isnan(sin_theta)) = 0;
    cos_theta = abs(x) ./ r;
    cos_theta(isnan(cos_theta)) = 1;

    % 计算风险源（直接风险和潜在风险）
    E = 0.5 * m * v^2;  % 车辆动能
    E_p = 0.5 * m * (v - v_ave)^2;  % 潜在风险能量

    % 缩放系数
    k_j1 = 1;
    k_j2 = 1;
    f_zoom_1 = k_j1 * (v_max - v) / v_max;
    f_zoom_2 = k_j2 * v_max / (v_max - v);

    % 计算直接风险场
    U1 = f_zoom_1 .* E .* A ./ r ./ sqrt((A / B).^2 .* sin_theta.^2 + f_zoom_1 .* cos_theta.^2);
    U2 = f_zoom_1 .* E .* A ./ r ./ sqrt((A / B).^2 .* sin_theta.^2 + f_zoom_2 .* cos_theta.^2);
    U_real = [U2(:, 1:(xj - x0) / resolution), U1(:, (xj - x0) / resolution + 1:end)];

    % 计算潜在风险场
    f_zoom_f = k_j1 * (v_max - v_ave) / v_max;
    f_zoom_r = k_j2 * v_max / (v_max - v_ave);

    U_p1 = f_zoom_f .* E_p .* A_ave ./ r ./ sqrt((A_ave / B_ave).^2 .* sin_theta.^2 + f_zoom_f .* cos_theta.^2);
    U_p2 = f_zoom_f .* E_p .* A_ave ./ r ./ sqrt((A_ave / B_ave).^2 .* sin_theta.^2 + f_zoom_r .* cos_theta.^2);

    if v >= v_ave
        U_p = [U_p2(:, 1:(xj - x0) / resolution), U_p1(:, (xj - x0) / resolution + 1:end)];
    else
        U_p = [U_p1(:, 1:(xj - x0) / resolution), U_p2(:, (xj - x0) / resolution + 1:end)];
    end

    % 合并直接风险和潜在风险
    U = U_real + U_p;
end
