function [t_Data,xe_Data,he_Data,error_Data,xd_Data] = impedanceControl(numLinks,xdFunc,xd_dotFunc,xd_ddotFunc,kp,kd,Md,q0,heFunc,dt,Arm,Bq,Cq,Gq,Fqpos,Fqneg)
% Inputs:
% numLinks
% xdFunc: 6 x 1 vector of syms that define% [xdx,xdy,xdz,phidx,phidy,phidz]
% xd_dotFunc, xd_ddotFunc, same as above but for vel, acc
% kp, kd, Md: Gains (scalar, scalar, 6 x 6)
% heFunc: 6 x 1 vector of syms that define he as a function of time
% Bq: symbolic version of B
% dt: time step, sec
% q0: initial joint variables

% Ouputs:
% t_Data, xd_Data: m x 1 vectors, where m is number of time points
counter = 0;
syms t
% Initialize time
time = 0;
% Initialize xd, xddot, xdddot, he
xd = eval(subs(xdFunc,t,time));
xd_dot = eval(subs(xd_dotFunc,t,time));
xd_ddot = eval(subs(xd_ddotFunc,t,time));
he = eval(subs(heFunc,t,time));

% Initialize q, qdot:
q = q0;
qdot = zeros(numLinks,1);
qddot = zeros(numLinks,1);
T=Arm.fkine(q);
rpy=tr2rpy(T); % roll pitch yaw
p=transl(T);
%xe: 6 x 1 vector of actual 6-DOF arm position
xe=[p(1); p(2); p(3); rpy(1); rpy(2); rpy(3)];
xe_dot = zeros(6,1);
xe_ddot = zeros(6,1);

% Initialize plotting data
xe_Data = xe;
t_Data = time;
he_Data = he;
xd_Data = xd;
% Initialize matrices:
% Initialize Ja
Ja = Arm.jacob0(q,'rpy');
% Initialize B(q)
B = BFinder(Bq,q);
% Initialize G(q)
G = GFinder(Gq,q);
% Initialize C(q,qdot)
C = CFinder(Cq,q,qdot');
% Initialize F
F = FFinder(Fqpos, Fqneg, qdot')';
% Initialize n(q,qdot)
n = C*qdot + F + G;

% Initialize errors
xtilda = xd - xe;
x_dtilda = xd_dot - xe_dot;
x_ddtilda = xd_ddot - xe_ddot;
error_Data = [norm(xtilda)];
% Perform loop
while norm(xtilda) > 0.1
    % Go through the loop up to the manipulator:
    s = kp*xtilda + kd*x_dtilda + Md*x_ddtilda;
    s = inv(Md)*s;
    s = pinv(Ja)*s;
    s = B*s;
    u = s+n;
    
    % Put u into the manipulator and get q, qdot:
    s = u - (Arm.jacob0(q))'*he;
    qddot = inv(B)*(-G-C*qdot-F+s);
    qdot = qdot + dt*qddot;
    q = q + dt*qdot;
    
    % Update all the matrices with q, qdot:
    % Update Ja
    Ja = Arm.jacob0(q,'rpy');
    % Update Ja_dot*qdot
    Jadot_qdot = Arm.jacob_dot(q,qdot);
    % Evaluate B(q)
    B = BFinder(Bq,q);
    % Evaluate G(q)
    G = GFinder(Gq,q);
    % Evaluate C(q,qdot)
    C = CFinder(Cq,q,qdot');
    % Evaluate F
    F = FFinder(Fqpos, Fqneg, qdot')';
    % Evaluate n(q,qdot)
    n = C*qdot + F + G;
    
    % Update end effector position, velocity, acceleration:
    T=Arm.fkine(q);
    rpy=tr2rpy(T); % roll pitch yaw
    p=transl(T);
    %xe: 6 x 1 vector of actual 6-DOF arm position
    xe = [p(1); p(2); p(3); rpy(1); rpy(2); rpy(3)]
    x_dot_e = Ja*qdot;
    x_ddot_e = Jadot_qdot;
    
    % Increase time step
    time = time + dt;
    
    % Evaluate xd, xddot, xdddot, he at new time step
    xd = eval(subs(xdFunc,t,time));
    xd_dot = eval(subs(xd_dotFunc,t,time));
    xd_ddot = eval(subs(xd_ddotFunc,t,time));
    he = eval(subs(heFunc,t,time));
    
    % Update errors:
    xtilda = xd-xe
    x_dtilda = xd_dot-x_dot_e;
    x_ddtilda = xd_ddot - x_ddot_e;
    
    % Store data:
    xe_Data = [xe_Data,xe];
    t_Data = [t_Data;time];
    error_Data = [error_Data,norm(xtilda)];
    he_Data = [he_Data,he];
    xd_Data = [xd_Data,xd];
%     disp('Current Error')
%     disp(norm(xtilda))
    
    % Update counter:
    counter = counter + 1;
    if counter > 200
        disp('Maximum Iterations Exceeded - Change Parameters or Increase Limit in impedanceControl.m')
        break
    end
end