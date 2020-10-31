%% Robotic Plot
%{
MAE 547 Fall 2020
HW2
Author: Jae Choi
Date: 10/01/2020
Contact: jchoi154@asu.edu
%}
clear all; clc;
ask_done=false;
i=0;
dh_matrix=[];
Links=[];
while ask_done==false
    dh_prompt = strcat('What is your link ',num2str(i), ' DH parameters ([a alpha d theta]) \n')
    dh = input(dh_prompt);
    dh_matrix = cat(1, dh_matrix, dh);
    link_prompt = strcat('What is your link ',num2str(i), ' Type? (R: revolute/P: prismatic) \n')
    type = input(link_prompt, 's');
    if type=='R'
        %ind_Link=Link(type, 'a', dh(1), 'alpha', dh(2), 'd', dh(3), 'modified');
        ind_Link=Link('a', dh(1), 'alpha', dh(2), 'd', dh(3));
    elseif type=='P'
        %type = 'prismatic';
        %ind_Link=Link(type, 'a', dh(1), 'alpha', dh(2), 'standard');
        ind_Link=Link('a', dh(1), 'alpha', dh(2), 'theta', dh(4));
        
        link_lim_prompt = strcat('What is the limit of your link ',num2str(i), ' ([0 2]) \n')
        link_lim = input(link_lim_prompt);
        ind_Link.qlim = link_lim;
    end
    Links = cat(1, Links, ind_Link);
    i = i+1;
    prompt2 = 'are you done? (Y/N) '
    YN = input(prompt2, 's');
    
    if YN == 'Y' || YN =='y'
        ask_done=true;
    end
    Links
end
dh_matrix;
dim_size=sum(dh_matrix(:,1))+sum(dh_matrix(:,1))/3;
dim_size = [-dim_size, dim_size, -dim_size, dim_size, -dim_size, dim_size];
Links

robot = SerialLink(Links, 'name', 'Your Robot')

T03_Links = robot.fkine(dh_matrix(:,4)')
figure();
origin=[zeros(1,i)];
robot.plot(origin, 'workspace', dim_size)
%robot.plot(dh_matrix(:,4)')
teach(robot)
% modified                       standard
%  a   alpha   d   theta         a     alpha   d   theta 
% [0,   0,     0,  pi/18]   R    [2,   0,     0,  pi/18]   R
% [2,   0,     0,  pi/9]    R    [3,   0,     0,  pi/9]    R
% [3,   0,     0,  pi/6]    R    [2,   0,     0,  pi/6]    R
% [2,   0,     0,    0]     R    

% theta  d    a    alpha SIGMA     a   alpha   d   theta
% [0, 0.363, 0.300, 0,    0]     [0.300, 0, 0.363, 0]    R
% [0,    0,  0.260, pi,   0]     [0.260, pi, 0, 0]       R
% [0,    0,   0,    0,    1]     [0, 0, 0, 0]            P
% L(3).qlim = [0,0.3];           [0,0.3]
% [0,    0,   0,    0,    0]     [0, 0, 0, 0]            R

% L(1) = Link([0,0.363,0.300,0,0],'standard');
% L(2) = Link([0,0,0.260,pi,0],'standard'); 
% L(3) = Link([0,0,0,0,1],'standard');
% L(3).qlim = [0,0.3];
% L(4) = Link([0,0,0,0,0],'standard');
% RobotArm = SerialLink(L,'name','SCARA') 
% RobotArm.plot([0 0 0 0],'workspace',[-1 1 -1 1 -1 1])

% L(1 )= Link('a', 0.300, 'alpha', 0, 'd', 0.363);
% L(2) = Link('a', 0.260, 'alpha', pi, 'd', 0);
% L(3) = Link('a', 0, 'alpha', 0, 'theta', 0);
% L(3).qlim = [0,0.3];
% L(4) = Link('a', 0, 'alpha', 0, 'd', 0);
% RobotArm = SerialLink(L) 
% RobotArm.plot([0 0 0 0],'workspace',[-1 1 -1 1 -1 1])  %'name','SCARA'

