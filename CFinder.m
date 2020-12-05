function [C] = CFinder(Cq, z, dp)
% Getting C
for j=1:length(z)
    q(j) = sym( sprintf('q%d', j), 'real' );
    qd(j) = sym( sprintf('qd%d', j), 'real' );
end
Cq=subs(Cq,q,z');
Cq=subs(Cq,qd,dp);
C=eval(Cq);
end