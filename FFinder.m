function [F] = FFinder(Fqpos,Fqneg,pd)
for j=1:length(p)
    qd(j) = sym( sprintf('qd%d', j), 'real' );
end
for j=1:length(p)
    if pd(j)<0
        Fq(j)=subs(Fqneg(j),qd(j),p(j));
    elseif pd(j)>0
        Fq(j)=subs(Fqpos(j),qd(j),p(j));
    else
        Fq(j)=0;
    end
end
F=eval(Fq);
end