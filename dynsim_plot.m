%Dynamics function
%Author: Jae Choi
nlinks = length(robot.links);
for i=1:nlinks
    robot.links(i).m = 50; %kg
    robot.links(i).Jm = 0.01; %kgm^2
    robot.links(i).I = [10*ones(3,1); zeros(3,1)];
    robot.links(i).G = 90;
end

q0=[-50;90;30]*pi/180; q_dot0=[0;0;0];

%t = [0:.056:2];
% [q,qd,qdd] = jtraj(qz, qr, time)
%[q,qd,qdd] = jtraj(q0, pi/180*[90 80 90], time);

%time
final_time = 3;

%Once the checks are completed, forward dynamics is applied
[time,q,q_dot] = robot.nofriction.fdyn(final_time,@torqfun,q0,q_dot0);
for i=1:length(q)
    q_ddot(i,:) = robot.accel(q(i,:), q_dot(i,:), [0 0 0]);
end

figure(1)
set(gcf, 'Position',  [100, 100, 700, 600])
hold on
grid on
ax1=subplot(4,1,1); 
for i = 1:nlinks
    plot(time,q(:,i))
end
legend('$q_1$','$q_2$','$q_3$','Interpreter','latex')
ax1=xlabel('time (s)');
ax1=ylabel('$q(t)$ (rad)','Interpreter','latex');
title('$q(t)$ vs time','Interpreter','latex')

ax2=subplot(4,1,2); 
hold on
grid on
for i = 1:nlinks
    plot(time,q_dot(:,i))
end
legend('$\dot{q_1}$','$\dot{q_2}$','$\dot{q_3}$','Interpreter','latex');
ax2=xlabel('time (s)');
ax2=ylabel('$\dot{q(t)}$ (rad/s)','Interpreter','latex');
title('$\dot{q(t)}$ vs time','Interpreter','latex')

ax3=subplot(4,1,3); 
hold on
grid on
for i = 1:nlinks
    plot(time,q_ddot(:,i))
end
legend('$\ddot{q_1}$','$\ddot{q_2}$','$\ddot{q_3}$','Interpreter','latex');
ax3=xlabel('time (s)');
ax3=ylabel('$\ddot{q(t)}$ (rad/s^2)','Interpreter','latex');
title('$\ddot{q(t)}$ vs time','Interpreter','latex')

T=robot.fkine(q);
p=T.transl;
[q_,qd_,qdd_] = jtraj(q0, p(end,:), time);

ax4=subplot(4,1,4);
robot.plot(q_)

% figure(2)
% set(gcf, 'Position',  [100, 100, 700, 600])
% hold on
% grid on
% ax1=subplot(4,1,1); 
% for i = 1:nlinks
%     plot(time,q_(:,i))
% end
% 
% ax2=subplot(4,1,2); 
% hold on
% grid on
% for i = 1:nlinks
%     plot(time,qd_(:,i))
% end
% 
% ax3=subplot(4,1,3); 
% hold on
% grid on
% for i = 1:nlinks
%     plot(time,qdd_(:,i))
% end


%GENERATES torque function
function tau = torqfun(robot, t, q, qd, varargin) %#ok<INUSD>
    tau = [10,5,1];
end