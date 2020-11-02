function [DH, J] = DHFinder(Z,U,L,J)
%{
Inputs:
Z: Z axis of joint i
U: direction of link i-1 w.r.t base frame
L: sizes of each link
J: Joint type (Revolute or Prismatic)
Outputs:
DH: a collection of dh parameters [a alpha d theta]
J: joint types
%}

%Normalize Z and U
for i=1:size(Z,1)
    Z(i,:)=Z(i,:)/norm(Z(i,:));
    U(i,:)=U(i,:)/norm(U(i,:));
end

%Find X
X=zeros(size(Z,1),3);
for i=1:size(Z,1)
    if i==1
        X(i,:)=cross([0,0,1],Z(i,:));
    else
        X(i,:)=cross(Z(i-1,:),Z(i,:));
    end
    if X(i,:)==[0,0,0]
        if i==1
            X(i,:)=[0,0,1];
        else
            X(i,:)=X(i-1,:);
        end
    end
end

%Find O for q=[0]
O=zeros(size(Z,1),3);
for i=1:size(Z,1)
    if i==1
        O(i,:)=U(i,:)*L(i);
    else
        O(i,:)=O(i-1,:)+U(i,:)*L(i);
    end
end

%Find a
a=zeros(size(Z,1),1);
for i=1:size(Z,1)
    P=(X(i,:)'*X(i,:))/(X(i,:)*X(i,:)');
    if i==1
        Pb=P*O(i,:)';
    else
        Pb=P*(O(i,:)-O(i-1,:))';
    end
    a(i)=norm(Pb);
end

%Find d
d=zeros(size(Z,1),1);
for i=1:size(Z,1)
    if i==1
        Zi1=[0 0 1];
        P=(Zi1'*Zi1)/(Zi1*Zi1');
        Pb=P*O(i,:)';
    else
        P=(Z(i-1,:)'*Z(i-1,:))/(Z(i-1,:)*Z(i-1,:)');
        Pb=P*(O(i,:)-O(i-1,:))';
    end
    d(i)=norm(Pb);
end

%Find alpha and theta
alpha=zeros(size(Z,1),1);
theta=zeros(size(Z,1),1);
for i=1:size(Z,1)
    if i==1
        alpha(i)=acos(dot([0 0 1],Z(i,:)));
        theta(i)=acos(dot([1 0 0],X(i,:)));
    else
        alpha(i)=acos(dot(Z(i-1,:),Z(i,:)));
        theta(i)=acos(dot(X(i-1,:),X(i,:)));
    end
end
DH=[a alpha d theta];
for i=1:size(DH,1)
    for j=1:size(DH,2)
        if isnan(DH(i,j))
          DH(i,j)=0;  
        end
    end
end
end

