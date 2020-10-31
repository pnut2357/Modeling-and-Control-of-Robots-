function [R,Tp,q,qd] = Dynamics(Robot)
%Dynamics function
%Author: Jae Choi
%This is a function file for creating the dynamic portion of the robot.
%This will essentially generate the dynamics plots for the specified user
%time interval. any changes to the robot object will be the output of this
%function file.

%Modified: initial code
%Modified: added functionality to qd0 and q0

% BUGS -------------------------------------------------------------------
% For prismatic joints in a robot, the forward dynamics of the system
% breaks. this will need to be fixed

%makes sure the rvc tools are enabled.
%startup_rvc;

%{ 
Link
1      Y  20  Y  95  
2      Y  20  Y  95
3      Y  20  Y  95
Torgue
1      2
2      0
3      0
Initial condition: position and velocity
Link
1      20  0
2      10  0
3      90  30

How long (s): 20
%}

%obtains number of links
nlinks = length(Robot.links);

%Checks if given robot object contains any mass value
for i = 1:nlinks
    
    %Checks if mass is empty
    if isempty(Robot.links(i).m) == 1
        
        fprintf('\nLink %d does not have any mass, would you like to give it mass? (Y/N): ',i);
        userinput = input('','s');
        
        if userinput == 'Y'
            massinput = input('Please enter a mass (in kg): ','s');
            Robot.links(i).m = str2double(massinput);
        else
            %NOTE: WAS GOING TO DO WARNING, BUT THIS MESSAGE DOES NOT DISPLAY
            %OUTSIDE OF FUNCTION AREA, WILL NEED TO FIX THIS LATER SOMEHOW
            fprintf('\nLink %d does not have mass value, giving mass of 50kg\n',i);
            Robot.links(i).m = 50;   %kg
        end
        
    end
    
    %Checks if motor inertia is empty
    if isempty(Robot.links(i).Jm) == 1
        
        fprintf('\nLink %d does not have any motor inertia, would you like to give it inertia? (Y/N): ',i);
        userinput = input('','s');
        
        if userinput == 'Y'
            motorinput = input('Please enter a motor inertia (in kgm^2): ','s');
            Robot.links(i).Jm = str2double(motorinput);
        else
            %NOTE: WAS GOING TO DO WARNING, BUT THIS MESSAGE DOES NOT DISPLAY
            %OUTSIDE OF FUNCTION AREA, WILL NEED TO FIX THIS LATER SOMEHOW
            fprintf('\nMotor %d does not have motor value, giving motor inertia of 0.01 kgm^2\n',i);
            Robot.links(i).Jm = 0.01;   %kgm^2
        end
        
    end
    
    %Checks if Link inertia is empty (GOING TO LEAVE AS CONSTANT FOR NOW)
    if (isempty(Robot.links(i).I) == 1)
        
        fprintf('\nLink %d does not have any link inertia, would you like to give it inertia? (Y/N): ',i);
        userinput = input('','s');
        
        if userinput == 'Y'
            Inertinput = input('Please enter a link inertia (in kgm^2): ','s');
            Robot.links(i).I = [str2double(Inertinput)*ones(3,1); zeros(3,1)];
        else
            %NOTE: WAS GOING TO DO WARNING, BUT THIS MESSAGE DOES NOT DISPLAY
            %OUTSIDE OF FUNCTION AREA, WILL NEED TO FIX THIS LATER SOMEHOW
            fprintf('\nLink %d does not have inertia value, giving link inertia of 10 kgm^2\n',i);
            Robot.links(i).I= [10*ones(3,1); zeros(3,1)];  %kgm^2
        end
    elseif Robot.links(i).I == zeros(3)
        fprintf('\nLink %d does not have any link inertia, would you like to give it inertia? (Y/N): ',i);
        userinput = input('','s');
        
        if userinput == 'Y'
            Inertinput = input('Please enter a link inertia (in kgm^2): ','s');
            Robot.links(i).I = [str2double(Inertinput)*ones(3,1); zeros(3,1)];
        else
            %NOTE: WAS GOING TO DO WARNING, BUT THIS MESSAGE DOES NOT DISPLAY
            %OUTSIDE OF FUNCTION AREA, WILL NEED TO FIX THIS LATER SOMEHOW
            fprintf('\nLink %d does not have inertia value, giving link inertia of 10 kgm^2\n',i);
            Robot.links(i).I= [10*ones(3,1); zeros(3,1)];  %kgm^2
        end        
        
    end
    
    %Checks if motor gear ratios is empty
    if Robot.links(i).G == 0
        
        fprintf('\nMotor %d does not have any motor gear ratio, would you like to give it a ratio? (Y/N): ',i);
        userinput = input('','s');
        
        if userinput == 'Y'
            gearinput = input('Please enter a motor gear ratio (out of 100): ','s');
            Robot.links(i).G = str2double(gearinput);
        else
            %NOTE: WAS GOING TO DO WARNING, BUT THIS MESSAGE DOES NOT DISPLAY
            %OUTSIDE OF FUNCTION AREA, WILL NEED TO FIX THIS LATER SOMEHOW
            fprintf('\nMotor %d does not have motor ratio value, giving motor ratio of 90 \n',i);
            Robot.links(i).G = 90;
        end
        
    end
    
    %FOR NOW I AM GOING TO LEAVE CENTER OF GRAVITY ALONE SINCE IT DEFAULTS
    %TO THE CENTER OF THE PART. 
    
    %Checks if user wants to remove any friction
%     fprintf('\nWould you like to remove friction on Link %d? (Y/N): ',i);
%     frictioninput = input('','s');
%     
%     if frictioninput == 'Y'
%         fprintf('\nFriction is now removed\n')
%         Robot.links(i).nofriction();
%     end
%     
    fprintf('\n ======================================================================= \n')
end

%This section creates a torque function
Torque = [];
for i = 1:nlinks
    fprintf('\nWhat torque function would you like to have for link %d?\n',i);
    torqueinput = input('','s');
    
    Torque = [Torque, str2double(torqueinput)]; %#ok<AGROW>
end

%Initial conditions for position and velocity 
fprintf('\nWould you like to have initial conditions for position and velocity? (Y/N): ')
userinput = input('','s');
D0 = zeros(nlinks,1); DD0 = zeros(nlinks,1);
if userinput == 'Y'
    %WILL  NEED TO ADD THIS LATER
    for i = 1:nlinks
        %if Robot link is revolute
        if Robot.links(i).isprismatic == 0 
            fprintf('\nInitial position of link %d (revolute) (deg): ',i)
            qinput = input('','s');
            D0(i) = str2double(qinput)*pi/180;
            
            fprintf('\nInitial velocity of link %d (revolute) (deg/s): ',i)
            qdinput = input('','s');
            DD0(i) = str2double(qdinput)*pi/180;
        end
        %If Robot link is prismatic
        if Robot.links(i).isprismatic == 1
            fprintf('\nInitial position of link %d (prismatic) (m): ',i)
            qinput = input('','s');
            D0(i) = str2double(qinput);
            
            fprintf('\nInitial velocity of link %d (prismatic) (m/s): ',i)
            qdinput = input('','s');
            DD0(i) = str2double(qdinput);
        end
        
    end
else
    fprintf('\nAssuming zero position and velocity\n')
    %ASSUMES ZERO VELOCITY AND POSITION
    for i = 1:nlinks
        D0(i) = 0; 
        DD0(i) = 0; 
    end
    
end

%GENERATES torque function
    function TAU = torqfun(R, t, q, qd, varargin) %#ok<INUSD>
        TAU = Torque;
    end

%Finally asks user for how much time do they want the simulation to run for
fprintf('\nHow long do you want the simulation to run for? (in seconds): ');
timeinput = input('','s');
time = str2double(timeinput);

%Once the checks are completed, forward dynamics is applied
[Tp,q,qd] = Robot.nofriction.fdyn(time,@torqfun,D0,DD0);

figure(1)
hold on
grid on
for i = 1:nlinks
    plot(Tp,q(:,i))
end
%plot(Tp,q(:,1),'r',Tp,q(:,2),'b',Tp,q(:,3),'g')
title('q(t) vs time')
xlabel('time (s)')
ylabel('q(t)')
%legend('q(1)','q(2)','q(3)','location','best')
hold off

figure(2)
hold on
grid on
for i = 1:nlinks
    plot(Tp,qd(:,i))
end
%plot(Tp,qd(:,1),'r',Tp,qd(:,2),'b',Tp,qd(:,3),'g')
title('qd(t) vs time')
xlabel('time (s)')
ylabel('qd(t)')
%legend('qd(1)','qd(2)','qd(3)','location','best')
hold off

figure(3)
Robot.plot(q)
    
end
