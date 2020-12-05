function [G] = GFinder(Gq, z)
% Getting G
for j=1:length(z)
    q(j) = sym( sprintf('q%d', j), 'real' );
end
Gq=subs(Gq,q,z');
G=eval(Gq);
end
