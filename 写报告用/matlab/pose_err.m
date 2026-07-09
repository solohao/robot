function e = pose_err(Tc, Td)
% POSE_ERR  位姿误差向量 [Δp; Δφ] (6x1)
    ep = Td(1:3,4)-Tc(1:3,4);
    Re = Td(1:3,1:3)*Tc(1:3,1:3)';
    tr = max(min((trace(Re)-1)/2,1),-1);
    ang = acos(tr);
    if ang<1e-9, eo=zeros(3,1);
    else eo=ang/(2*sin(ang))*[Re(3,2)-Re(2,3);Re(1,3)-Re(3,1);Re(2,1)-Re(1,2)]; end
    e = [ep;eo];
end
