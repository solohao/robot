function traj = path_ik(path_poses)
% PATH_IK  7-DOF空间机械臂路径逆运动学（热启动跟踪）
% 输入: path_poses - Nx6矩阵, 每行 [x y z alpha beta gamma] (m, rad)
% 输出: traj       - Nx7矩阵, 每行对应7个关节角 (rad)
%
% 首帧使用原 ik_space_robot 多随机搜索，后续帧以上一帧解为初值快速迭代。
% 若热启动收敛失败，自动回退到 ik_space_robot。

    DH = [0,0,0.12,0; pi/2,0,0.1,-pi/2; pi/2,0,0.1,pi;
          0,0.4,0.1,pi; 0,0.4,0.1,0; pi/2,0,0.1,pi/2;
          pi/2,0,0,0; pi,0,-0.12,pi];

    N = size(path_poses, 1);
    traj = zeros(N, 7);

    for i = 1:N
        pose = path_poses(i, :);
        T_des = euler_XYpZ_to_T(pose(1), pose(2), pose(3), ...
                                 pose(4), pose(5), pose(6));

        if i == 1
            % 第一帧：使用原多随机搜索
            sols = ik_space_robot(pose(1), pose(2), pose(3), ...
                                  pose(4), pose(5), pose(6));
            theta = sols(1).theta;
        else
            % 热启动：以上一帧解为初值
            theta = traj(i-1, :)';
            lam = 0.3;
            for iter = 1:30
                Tc = fk_mat(theta, DH);
                e = pose_err(Tc, T_des);
                if norm(e) < 1e-6, break; end
                J = jac_num(theta, DH);
                dq = J' * ((J*J' + lam^2*eye(6)) \ e);
                dq = max(min(dq, 0.3), -0.3);
                theta = theta + dq;
                theta = atan2(sin(theta), cos(theta));
            end
            % 校验，失败则回退原函数
            err = max(abs(fk_mat(theta, DH) - T_des), [], "all");
            if err > 1e-3
                sols = ik_space_robot(pose(1), pose(2), pose(3), ...
                                      pose(4), pose(5), pose(6));
                theta = sols(1).theta;
            end
        end
        traj(i, :) = theta';
    end
end
