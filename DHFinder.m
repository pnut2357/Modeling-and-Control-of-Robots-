function [DH, J,Xi1] = DHFinder(Z,Zi1,Xi1,U,L,J)
%{
Inputs:
Z: Z axis of joint i (1x3)
Zi1: Z axis of joint i-1 (1x3)
U: direction of link i-1 w.r.t base frame (1x3)
L: sizes of each link (1x1)
J: Joint type (Revolute or Prismatic)
Outputs:
DH: a collection of dh parameters [a alpha d theta]
J: joint types
Xi1: the previous X 
%}

%Normalize Z and U
Z=Z/norm(Z);
Zi1=Zi1/norm(Zi1);
U=U/norm(U);

%Find X
X=cross(Zi1,Z);
if norm(X)==0
    X=Xi1;
end

%Find O for q=[0]
O=U*L;

%Find a
Pb=((X'*X)/(X*X'))*O;
a=norm(Pb);

%Find d
Pb=(Zi1'*Zi1)/(Zi1*Zi1')*O;
d=norm(Pb);

%Find alpha and theta
alpha=acos(dot(Zi1,Z));
theta=acos(dot(Xi1,X));

DH=[a alpha d theta];
Xi1=X;

for i=1:size(DH,1)
    if isnan(DH(i))
      DH(i)=0;
    end
end
end

