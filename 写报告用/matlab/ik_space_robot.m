function solutions = ik_space_robot(x, y, z, alpha, beta, gamma)
% IK_SPACE_ROBOT  7-DOF绌洪棿鏈烘鑷傞€嗚繍鍔ㄥ
% 杈撳嚭: solutions.theta (7x1), .error, .is_valid
    DH = [0,0,0.12,0; pi/2,0,0.1,-pi/2; pi/2,0,0.1,pi;
          0,0.4,0.1,pi; 0,0.4,0.1,0; pi/2,0,0.1,pi/2;
          pi/2,0,0,0; pi,0,-0.12,pi];
    T_des = euler_XYpZ_to_T(x,y,z,alpha,beta,gamma);
    cand = {}; rng("shuffle");

    % 绛栫暐1: 澶氶殢鏈哄垵鍊?DLS (500娆?
    for trial = 1:500
        if trial == 1
            theta = zeros(7,1);
        else
            theta = randn(7,1);
        end
        lam = 0.5;
        for iter = 1:500
            Tc = fk_mat(theta,DH);
            e = pose_err(Tc,T_des);
            if norm(e) < 1e-6, break; end
            J = jac_num(theta,DH);
            dq = J' * ((J*J' + lam^2*eye(6)) \ e);
            dq = max(min(dq,0.5),-0.5);
            theta = theta + dq;
            theta = atan2(sin(theta),cos(theta));
        end
        if max(abs(fk_mat(theta,DH)-T_des),[],"all") < 1e-4
            cand{end+1} = theta;
        end
    end

    % 鍘婚噸鎺掑簭
    if isempty(cand)
        solutions = struct("theta",{[]},"error",inf,"is_valid",false);
        return;
    end
    uq = {};
    for i = 1:length(cand)
        dup = false;
        for j = 1:length(uq)
            if mean(abs(wrapToPi(cand{i}-uq{j}))) < 0.15, dup=true; break; end
        end
        if ~dup, uq{end+1}=cand{i}; end
    end
    errs = zeros(length(uq),1);
    for i = 1:length(uq)
        errs(i) = max(abs(fk_mat(uq{i},DH)-T_des),[],"all");
    end
    [~,id] = sort(errs);
    n = min(length(uq),10);
    S(n) = struct("theta",[],"error",[],"is_valid",[]);
    for i = 1:n
        S(i).theta = uq{id(i)}; S(i).error = errs(id(i));
        S(i).is_valid = errs(id(i)) < 0.001;
    end
    solutions = S;
end
