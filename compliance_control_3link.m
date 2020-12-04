function [xe] = compliance_control(he,xd,Arm,Im,Il,Mm,Ml,J,k,g0eom,dh_syms, tf, FB, Fpos, Fneg)
%{
Inputs:
Special inputs:
he: function of t (time) given by user
t0: initial time, constant (0 usually)
tf: final time, constant 
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
dh_syms: the N x 4 matrix of symbolic dh parameters (numbers)
Outputs:
B: the n x n matrix of inertial effects
C: the n x n matrix of centrifugal and coriolis effects
G: the n x 1 matrix of gravity effects
%}
global J;
t0=0;

global B;
global C;
global G;
[B,C,G] = EOMFinder(Arm,Im,Il,Mm,Ml,J,k,g0eom,dh_syms);

global n;
n=3;
global q;
global qd;
%global q0;
q=zeros(n,1);
qd=zeros(n,1);
% q0=zeros(n,1);



%initialize q and dq
% q = Simulink.Signal;
% q.CoderInfo.StorageClass = 'ExportedGlobal';
% q.InitialValue = 'q0';
% dq = Simulink.Signal;
% dq.CoderInfo.StorageClass = 'ExportedGlobal';
% dq.InitialValue = 'q0';

% q=zeros(n,1);
% dq=zeros(n,1);

%dh_calc: the N x 1 matrix of dh parameters (numbers)
dh_calc=double(dh_syms);

%Save all variables to file
%save ('globals.mat','J','q','qd','dh_calc') 
save('C:\Users\Anna Rothweil\Documents\MAE547\Final Report\3-Link\globals.mat')

%timespan=linspace(t0, tf, (tf-t0)*100);
[xe]=sim('Force_control_compliance_3link'); 
%Simulate the model
%plot xe, xd and he in AppDesigner
end

