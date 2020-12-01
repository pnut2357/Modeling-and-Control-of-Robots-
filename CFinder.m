function [C] = CFinder(Cq, p, dp)
% Getting C
for j=1:length(p)
    q(j) = sym( sprintf('q%d', j), 'real' );
end
Cq=subs(Cq,q,p);
C=eval(Cq);
end

