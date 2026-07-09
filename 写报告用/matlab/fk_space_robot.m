function [pos, euler_angles] = fk_space_robot(theta)
% FK_SPACE_ROBOT  7-DOF空间机械臂正运动学
%   输入: theta - 7个关节角 (rad), 列向量 [theta1;...;theta7]
%   输出: pos          - [x, y, z] 末端位置 (m)
%         euler_angles - [alpha, beta, gamma] XY''Z''欧拉角 (rad)
%
%   基于 Modified DH（非标准D-H），DH参数来自正逆运动学和轨迹规划.docx
%
%   关节排列: Roll-Pitch-Pitch-ElbowPitch-Pitch-Pitch-Roll
%   Modified DH: T_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)

    % === DH 参数表 (8行, 前7行为关节段, 第8行为末端固定变换) ===
    % [alpha(rad), a(m), d(m), theta_offset(rad)]
    DH = [
        0,          0,      0.120,  0;             % i=1
        pi/2,       0,      0.100,  -pi/2;          % i=2
        pi/2,       0,      0.100,  pi;             % i=3
        0,          0.400,  0.100,  pi;             % i=4
        0,          0.400,  0.100,  0;              % i=5
        pi/2,       0,      0.100,  pi/2;           % i=6
        pi/2,       0,      0,      0;              % i=7
        pi,         0,      -0.120, pi;             % i=8
    ];

    T = eye(4);
    for i = 1:8
        alpha = DH(i,1); a = DH(i,2); d = DH(i,3); off = DH(i,4);
        if i <= 7
            th = off + theta(i);
        else
            th = off;  % 第8行固定
        end
        T = T * Rx(alpha) * Tx(a) * Rz(th) * Tz(d);
    end

    % 提取位置
    pos = T(1:3, 4)';

    % 提取 XY'Z' 欧拉角 (内旋: Z-Y-X)
    % R = Rz(gamma) * Ry(beta) * Rx(alpha)
    R = T(1:3, 1:3);
    beta = asin(-R(3,1));
    
    if abs(cos(beta)) > 1e-10
        alpha = atan2(R(3,2), R(3,3));
        gamma = atan2(R(2,1), R(1,1));
    else
        % 奇异情况 (cos(beta)=0, 万向锁)
        alpha = 0;
        gamma = atan2(-R(1,2), R(2,2));
    end
    euler_angles = [alpha, beta, gamma];
end

% ===== 内部辅助函数 =====
function T = Rx(a)
    T = eye(4);
    T(2,2)=cos(a); T(2,3)=-sin(a);
    T(3,2)=sin(a); T(3,3)=cos(a);
end

function T = Ry(b)
    T = eye(4);
    T(1,1)=cos(b); T(1,3)=sin(b);
    T(3,1)=-sin(b); T(3,3)=cos(b);
end

function T = Rz(t)
    T = eye(4);
    T(1,1)=cos(t); T(1,2)=-sin(t);
    T(2,1)=sin(t); T(2,2)=cos(t);
end

function T = Tx(d)
    T = eye(4); T(1,4)=d;
end

function T = Tz(d)
    T = eye(4); T(3,4)=d;
end
