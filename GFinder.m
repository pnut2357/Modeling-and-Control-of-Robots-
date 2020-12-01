function [G] = GFinder(Gq, p)
% Getting G
for j=1:length(p)
    q(j) = sym( sprintf('q%d', j), 'real' );
end
Gq=subs(Gq,q,p);
G=eval(Gq);
end