function [C] = CFinder(Cq, q, dq)
% Getting C
s=size(q, 2);
if s=1
    CQ=subs(Cq, q1, q(1));
end
else if s=2
    CQ=subs(Cq, [q1, q2], [q(1), q(2)]);
end
else if s=3
    CQ=subs(Cq, [q1, q2, q3], [q(1), q(2), q(3)]);
end
else if s=4
    CQ=subs(Cq, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]);
end
else if s=5
    CQ=subs(Cq, [q1, q2, q3, q4, q5], [q(1), q(2), q(3), q(4), q(5)]);
end
else if s=6
    CQ=subs(Cq, [q1, q2, q3, q4, q5, q6], [q(1), q(2), q(3), q(4), q(5), q(6)]);
end
else if s=7
    CQ=subs(Cq, [q1, q2, q3, q4, q5, q6, q7], [q(1), q(2), q(3),  q(4), q(5), q(6), q(7)]);
end
C=eval(CQ);
end

