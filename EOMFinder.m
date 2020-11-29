function [B,C,G,Je, Arm] = EOMFinder(Arm,Im,Il,Mm,Ml,J,k,g0,dh_syms)
%{
Inputs:
Arm: the Serial Link form of the robotic arm in question
Im: the N x 1 vector of motor inertias
Il: the 1 x N cell arry of link inertias (Il{i} is a 3 x 3 matrix)
Mm: the N x 1 vector of motor masses
Ml: the N x 1 vector of link masses
J: the 1 x N vector of joint types (0 is rev 1 is pris)
k: the N x 1 vector of gear ratios
g0: the 3 x 1 vector of gravity with magnitude equal to gravitational acceleration (norm(g0)=9.81 on earth)
dh_syms: the N x 4 matrix of symbolic dh parameters (numbers)
Outputs:
B: the n x n matrix of inertial effects
C: the n x n matrix of centrifugal and coriolis effects
G: the n x 1 matrix of gravity effects
Fpos: the n x 1 matrix of friction coefficients when qdot is positive
Fneg: the n x 1 matrix of friction coefficients when qdot is negative
Je: The 6 x n geometric jacobian
%}
%% Setup
format long
[q,qd,qdd]=Arm.gencoords;
assume(q,'real');
assume(qd,'real');
assume(qdd,'real');
n=Arm.n;
%Get DH parameters and offset
a=(dh_syms(:,3))';
alpha=(dh_syms(:,4))';
%theta = Arm.theta;
%d = Arm.d;
offset = sym(Arm.offset);
for i=1:n
    if J(i)==0
        theta(i)=q(i)+offset(i);
    else
        d(i)=q(i)+offset(i);
    end
end
for i=1:n
    if J(i)==0
        d(i)=Arm.d(i);
    else
        theta(i)=Arm.theta(i);
    end
end
%Get T symbolically
for i=1:n
    T{i}=Tfinder(a(i),alpha(i),d(i),theta(i));
    T0i{i}=eye(4);
    for j=1:i
        T0i{i}=T0i{i}*T{j};
    end
end
%% Find JPL, JOL
for i=1:n
    Jpl{i}=sym(zeros(3,n));
    Jol{i}=sym(zeros(3,n));
    for j=1:i
        if j==1
            zj1=[0 0 1];
            pj1=[0 0 0];
            pl=T0i{i}(1:3,4)/2;
        else
            zj1=T0i{j-1}(1:3,3);
            pj1=T0i{j-1}(1:3,4)';
            pl=(T0i{i}(1:3,4)-T0i{j-1}(1:3,4))/2;
        end
        if J(j)==0
            Jpl{i}(:,j)=cross(zj1,(pl-pj1'))';
            Jol{i}(:,j)=zj1';
        else
            Jpl{i}(:,j)=zj1;
            Jol{i}(:,j)=[0;0;0];
        end
    end
end

%% Find JPM
for i=1:n
    Jpm{i}=sym(zeros(3,n));
    for j=1:i-1
        if j==1
            zj1=[0 0 1];
            pj1=[0 0 0]';
        else
            zj1=T0i{j-1}(1:3,3);
            pj1=T0i{j-1}(1:3,4);
        end
        if i==1
            pm=[0 0 0];
        else
            pm=T0i{i-1}(1:3,4);
        end
        if J(j)==0
            Jpm{i}(:,j)=cross(zj1,(pm-pj1));
        else
            Jpm{i}(:,j)=zj1;
        end
    end
end
%% Find JOM
for i=1:n
    Jom{i}=sym(zeros(3,n));
    for j=1:i
        if j < i
            Jom{i}(:,j)=Jol{i}(:,j);
        else
            if i==1
                zm=[0 0 1]';
            else
                zm=T0i{i-1}(1:3,3);
            end
            Jom{i}(:,j)=k(i)*zm;
        end
    end
end
%% Calculate B
B=zeros(n,n);
for i=1:n
    B=B+Ml(i)*Jpl{i}'*Jpl{i};
    B=B+Jol{i}'*T0i{i}(1:3,1:3)*Il(i)*T0i{i}(1:3,1:3)'*Jol{i};
    B=B+Mm(i)*Jpm{i}'*Jpm{i};
    if i==1
        B=B+Jom{i}'*Im(i)*Jom{i};
    else
        B=B+Jom{i}'*T0i{i-1}(1:3,1:3)*Im(i)*T0i{i-1}(1:3,1:3)'*Jom{i};
    end
end
B=simplify(B);
%% Calculate C
C=sym(zeros(n,n));
for i=1:n
    for j=1:n
        for k=1:n
            bijqk=diff(B(i,j),q(k));
            bikqj=diff(B(i,k),q(j));
            bjkqi=diff(B(j,k),q(i));
            cijk=.5*(bijqk+bikqj-bjkqi);
            C(i,j)=C(i,j)+cijk*qd(k);
        end
    end
end
C=simplify(C);
%% Calculate G
G=sym(zeros(n,1));
for i=1:n
    for j=1:n
        G(i)=G(i)-Ml(j)*g0'*Jpl{j}(:,i);
        G(i)=G(i)-Mm(j)*g0'*Jpm{j}(:,i);
    end
end
G=simplify(G);
%%Functions
%Find Ti-1,i
function [T] = Tfinder(a,alpha,d,theta)
T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
    sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
    0,sin(alpha),cos(alpha),d;
    0,0,0,1];
end
%% Calculate F: NEEDS TO BE DOUBLE CHECKED BASED ON HOW INPUTS LOOK
%{
Fc=Arm.Tc;
Fb=Arm.B.*abs(qd);
Fpos=Fc(:,1)+Fb;
Fneg=Fc(:,2)-Fb;
%}
%% Calculate Je
Je=sym(zeros(6,n));
for i=1:n
    if i==1
        zi1=[0 0 1]';
        Pi1=[0 0 0]';
    else
        zi1=T0i{i-1}(1:3,3);
        Pi1=T0i{i-1}(1:3,4);
    end
    pe=T0i{n}(1:3,4);
    if J(i)==0
        Je(1:3,i)=cross(zi1,(pe-Pi1));
        Je(4:6,i)=zi1;
    else
        Je(1:3,i)=zi1;
    end
end
Je=simplify(Je);
%% Full EOM
end
