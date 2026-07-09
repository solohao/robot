function T = fk_mat(theta, DH)
% FK_MAT  正运动学 (Modified DH)
    T = eye(4);
    for i = 1:8
        th = DH(i,4); if i <= 7, th = th + theta(i); end
        T = T * Rx(DH(i,1)) * Tx(DH(i,2)) * Rz(th) * Tz(DH(i,3));
    end
end
