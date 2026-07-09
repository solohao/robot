%% 路径追踪测试 —— 直线路径
clear; clc;

%% 1. 起点 & 终点位姿 (m, rad)
start_pose = [-0.4,  0.1, -0.1, deg2rad(-90), 0, 0];
end_pose   = [-0.2,  0.3, -0.2, deg2rad(-90), 0, 0];

%% 2. 路径插值 (100 个点)
N = 100;
path = zeros(N, 6);
for i = 1:N
    t = (i-1) / (N-1);
    path(i, :) = (1-t) * start_pose + t * end_pose;
end

%% 3. 路径 IK 求解
traj = path_ik(path);   % traj: N x 7, 关节角 (rad)

%% 4. 正运动学验证 & 绘图
pos = zeros(N, 3);
for i = 1:N
    pos(i, :) = fk_space_robot(traj(i, :)');
end

figure('Position', [100 100 1200 500]);

% —— 子图 1：笛卡尔路径 ——
subplot(1, 2, 1); hold on; grid on; axis equal;
plot3(pos(:,1)*1000, pos(:,2)*1000, pos(:,3)*1000, 'b-', 'LineWidth', 1.5);
plot3(path(1,1)*1000, path(1,2)*1000, path(1,3)*1000, ...
      'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(path(end,1)*1000, path(end,2)*1000, path(end,3)*1000, ...
      'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('笛卡尔空间路径'); view(45, 30);
legend('实际轨迹', '起点', '终点', 'Location', 'best');

% —— 子图 2：关节角曲线 ——
subplot(1, 2, 2); hold on; grid on;
colors = lines(7);
for j = 1:7
    plot(1:N, rad2deg(traj(:, j)), 'Color', colors(j,:), ...
         'LineWidth', 1.2, 'DisplayName', sprintf('J%d', j));
end
xlabel('路径点序号'); ylabel('关节角 (deg)');
title('关节空间轨迹'); legend('Location', 'best');
