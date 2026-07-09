function J = jac_num(theta, DH)
% JAC_NUM  数值雅可比矩阵 (6x7, 前向差分)
    del = 1e-6;
    T0 = fk_mat(theta,DH); p0=T0(1:3,4); R0=T0(1:3,1:3);
    J = zeros(6,7);
    for i = 1:7
        th=theta; th(i)=th(i)+del;
        Tp=fk_mat(th,DH);
        J(1:3,i) = (Tp(1:3,4)-p0)/del;
        Rr = Tp(1:3,1:3)*R0';
        tr = max(min((trace(Rr)-1)/2,1),-1);
        ang = acos(tr);
        if ang>1e-9
            J(4:6,i)=ang/(2*sin(ang))*[Rr(3,2)-Rr(2,3);Rr(1,3)-Rr(3,1);Rr(2,1)-Rr(1,2)]/del;
        end
    end
end
