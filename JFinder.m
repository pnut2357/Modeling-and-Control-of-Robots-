function [J, he_calc] = JFinder(dh_calc, q, J, he, t)
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

%calculate geometric Jacobian
J=Arm.jacob0(q');

%evaluate he for certain t
he_calc=eval(subs(he, t, t));
end