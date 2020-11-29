function [xe] = compliance_control(he,xd,Arm,Im,Il,Mm,Ml,J,k,g0eom,dh_syms,t0, tf)
%{
Inputs:
Special inputs:
he: function of t (time) given by user
xd: 6x1 vector given by user

Common inputs:
Arm: the Serial Link form of the robotic arm in question
Im: the N x 1 vector of motor inertias
Il: the 1 x N cell arry of link inertias (Il{i} is a 3 x 3 matrix)
Mm: the N x 1 vector of motor masses
Ml: the N x 1 vector of link masses
J: the 1 x N vector of joint types (0 is rev 1 is pris)
k: the N x 1 vector of gear ratios
g0: the 3 x 1 vector of gravity with magnitude equal to gravitational acceleration (norm(g0)=9.81 on earth)
Outputs:
B: the n x n matrix of inertial effects
C: the n x n matrix of centrifugal and coriolis effects
G: the n x 1 matrix of gravity effects
%}

[B,C,G,Je] = EOMFinder(Arm,Im,Il,Mm,Ml,J,k,g0eom,dh_syms);
timespan=linspace(t0, tf, (tf-t0)*100);
xe=sim('Force_control_compliance',timespan) %Simulate the model
%plot xe, xd and he in AppDesigner
end

