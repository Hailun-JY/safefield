% Hailun Zhang 2024 版权所有
clc;
clear;

%% 参数设置
R = 1; T = 1; G = 0.5; k1 = 1; k3 = 70;
D = 3.5; para = [G, R, T, k1, k3];
m = [2000, 2200, 2400, 1800, 800, 1000]; % 车辆和障碍物质量
v = [0, 20, 25, 15, 0, 0];              % 车辆速度
pose = [
    26, D/2+1; 
    5, 3/2*D+1; 
    12, 5/2*D+1; 
    20, 7/2*D+1; 
    35, 5/2*D+1]; % 物体初始位置
k_values = [2*3, 1*3, 1*3, 1*3, 2*3];   % 道路线的系数

% 绘图范围
X = [0, 40]; Y = [-5, 4*D+1.5];
res = 0.015;
x_range = 0:res:X(2);
y_range = 0:res:Y(2);

% 初始化矩阵
[x, y] = meshgrid(x_range, y_range);
E = zeros(size(x)); Ex = E; Ey = E;

% 计算每个点的场强
for j = 1:length(y_range)
    for i = 1:length(x_range)
        obj_pose = [x(j, i), y(j, i)];
        total_Ex = 0; total_Ey = 0;

        % 计算每个物体的场强分量
        for n = 1:5
            [Ex_n, Ey_n] = Get_Str(para, m(n), v(n), 0, pose(n, :), obj_pose);
            total_Ex = total_Ex + Ex_n;
            total_Ey = total_Ey + Ey_n;
        end
        
        % 计算路边树木的场强分量
        [Ex_tree, Ey_tree] = Get_Str(para, m(6), 0, 0, [x(j, i), 1+4*D+1], obj_pose);
        total_Ex = total_Ex + Ex_tree;
        total_Ey = total_Ey + Ey_tree;
        
        % 计算道路线的场强分量
        E7 = compute_lane_force(k_values, R, D, y_range(j));

        % 总场强
        Ex(j, i) = total_Ex;
        Ey(j, i) = total_Ey + E7;
        E(j, i) = sqrt(Ex(j, i)^2 + Ey(j, i)^2);
        
        % 场强限值
        if E(j, i) >= 400
            E(j, i) = 400;
            Ex(j, i) = 400;
            Ey(j, i) = 400;
        end
    end
end

E = rescale(E);
%% 绘图
figure;
mesh(x, y, E);
view(30, 30);
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Field Strength');
title('Traffic Participants Field Strength Map');
% % set(gca, 'DataAspectRatio', [8, 8, 400]);
daspect([10 10 1])
colormap(jet);
h = colorbar;
set(get(h, 'YLabel'), 'string', 'Risk field quantification');

%% 计算道路线的场强函数
function E7 = compute_lane_force(k_values, R, D, Y)
    k = 2;
    if Y <= 1
        E7 = -k_values(1) * R * ((Y + D/2 - 1)^(k + 1));
    elseif Y <= D/2 + 1
        E7 = k_values(1) * R * ((D/2 + 1 - Y)^(k + 1));
    elseif Y <= D + 1
        E7 = -k_values(2) * R * ((Y - D/2 - 1)^(k + 1));
    elseif Y <= 3/2 * D + 1
        E7 = k_values(2) * R * ((3/2 * D + 1 - Y)^(k + 1));
    elseif Y <= 2 * D + 1
        E7 = -k_values(3) * R * ((Y - 3/2 * D - 1)^(k + 1));
    elseif Y <= 5/2 * D + 1
        E7 = k_values(3) * R * ((5/2 * D + 1 - Y)^(k + 1));
    elseif Y <= 3 * D + 1
        E7 = -k_values(4) * R * ((Y - 5/2 * D - 1)^(k + 1));
    elseif Y <= 7/2 * D + 1
        E7 = k_values(4) * R * ((7/2 * D + 1 - Y)^(k + 1));
    else
        E7 = -k_values(5) * R * ((Y - 7/2 * D - 1)^(k + 1));
    end
end

%% 场强计算函数
function [Ex, Ey] = Get_Str(para, m, vx, vy, pose, obj_pose)
    R = para(2); T = para(3); G = para(1); k1 = para(4); k3 = para(5);
    dr = obj_pose - pose;
    th = atan2(vy, vx) - atan2(dr(2), dr(1));
    r = norm(dr);
    v = hypot(vx, vy);
    M = T * m * (1.566e-14 * (v / 3.6)^6.687 + 0.3345);
    E = G * R * M * k3 / (k3 - v * cos(th)) / (r^k1);
    
    EX = ((k1 - 1) * v * cos(th) - k1 * k3) * sin(th) / (k3 - v * cos(th))^2 / r^(k1 + 1);
    EY = (k1 * v * cos(th)^2 - k1 * k3 * cos(th) + v * sin(th)^2) / (k3 - v * cos(th))^2 / r^(k1 + 1);

    if v ~= 0
        Ex = (EX * vx / v - EY * vy / v) / hypot(EX, EY) * E;
        Ey = (EX * vy / v + EY * vx / v) / hypot(EX, EY) * E;
    else
        Ex = E * dr(1) / r;
        Ey = E * dr(2) / r;
    end
end
