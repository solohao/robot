function T = Rx(a)
    T = eye(4);
    T(2,2)=cos(a); T(2,3)=-sin(a);
    T(3,2)=sin(a); T(3,3)=cos(a);
end
