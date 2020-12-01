function [B] = BFinder(Bq,p)
% Getting B
for j=1:length(p)
    q(j) = sym( sprintf('q%d', j), 'real' );
end
Bq=subs(Bq,q,p);
B=eval(Bq);
end