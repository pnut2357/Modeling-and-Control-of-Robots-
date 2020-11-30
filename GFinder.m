function [G] = GFinder(Gq, q)
% Getting G
s=size(q, 1);
if s=1
    GQ=subs(Gq, q1, q(1));
end
else if s=2
    GQ=subs(Gq, [q1, q2], [q(1), q(2)]);
end
else if s=3
    GQ=subs(Gq, [q1, q2, q3], [q(1), q(2), q(3)]);
end
else if s=4
    GQ=subs(Gq, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]);
end
else if s=5
    GQ=subs(Gq, [q1, q2, q3, q4, q5], [q(1), q(2), q(3), q(4), q(5)]);
end
else if s=6
    GQ=subs(Gq, [q1, q2, q3, q4, q5, q6], [q(1), q(2), q(3), q(4), q(5), q(6)]);
end
else if s=7
    GQ=subs(Gq, [q1, q2, q3, q4, q5, q6, q7], [q(1), q(2), q(3),  q(4), q(5), q(6), q(7)]);
end

G=eval(GQ);
end