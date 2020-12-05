function [F] = FFinder(Fqpos,Fqneg,pd)
for j=1:length(pd)
    qd(j) = sym( sprintf('qd%d', j), 'real' );
end
for j=1:length(pd)
    if pd(j)<0
        Fq(j)=subs(Fqneg(j),qd(j),pd(j));
    elseif pd(j)>0
        Fq(j)=subs(Fqpos(j),qd(j),pd(j));
    else
        Fq(j)=sym(0);
    end
end
F=eval(Fq);
end