function [B] = BFinder(Bq,z)
% Bq = symbolic B
% p = joint variables
% Getting B
for j=1:length(z)
    q(j) = sym( sprintf('q%d', j), 'real' );
end
Bq=subs(Bq,q,z');
B=eval(Bq);
end