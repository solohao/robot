function T = Rz(t)
    T = eye(4);
    T(1,1)=cos(t); T(1,2)=-sin(t);
    T(2,1)=sin(t); T(2,2)=cos(t);
end
