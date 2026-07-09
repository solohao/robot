function T = euler_XYpZ_to_T(x, y, z, alpha, beta, gamma)
% EULER_XYPZ_TO_T  XY''Z''欧拉角 → 齐次变换矩阵
    Rz3 = Rz(gamma); Rz3 = Rz3(1:3,1:3);
    Ry3 = Ry(beta);  Ry3 = Ry3(1:3,1:3);
    Rx3 = Rx(alpha); Rx3 = Rx3(1:3,1:3);
    T = eye(4); T(1:3,1:3) = Rz3*Ry3*Rx3;
    T(1,4)=x; T(2,4)=y; T(3,4)=z;
end
