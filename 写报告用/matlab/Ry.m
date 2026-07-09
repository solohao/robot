function T = Ry(b)
    T = eye(4);
    T(1,1)=cos(b); T(1,3)=sin(b);
    T(3,1)=-sin(b); T(3,3)=cos(b);
end
