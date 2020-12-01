function xe = ArmFinder(dh_calc, q, J)
%evaluates symbolic Arm for a certain q
 % Create a Link object for this link
 i=size(dh_calc,1);
a=dh_calc(:,1)';
alpha=dh_calc(:,2)';
d=dh_calc(:,3)';
theta=dh_calc(:,4)';
 for n=1:i
     if J(n)==0
        L(n)=Link('a', a(n), 'alpha', alpha(n), 'theta', theta(n));
     else
        L(n)=Link('a', a(n), 'alpha', alpha(n), 'd', d(n));       
     end        
 end
Arm=SerialLink(L);

T=Arm.fkine(q);
%ZYZ Euler angles
rpy=tr2rpy(T);

%xe: 6 x 1 vector of actual 6-DOF arm position
xe=zeros(6, 1);
xe=[T(13); T(14); T(15); rpy(1); rpy(2); rpy(3)];
xe=eval(xe);
end

