function [B] = BFinder(Bq, q)
% Getting B
s=size(q, 2);
if s=1
    BQ=subs(Bq, q1, q(1));
end
else if s=2
    BQ=subs(Bq, [q1, q2], [q(1), q(2)]);
end
else if s=3
    BQ=subs(Bq, [q1, q2, q3], [q(1), q(2), q(3)]);
end
else if s=4
    BQ=subs(Bq, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]);
end
else if s=5
    BQ=subs(Bq, [q1, q2, q3, q4, q5], [q(1), q(2), q(3), q(4), q(5)]);
end
else if s=6
    BQ=subs(Bq, [q1, q2, q3, q4, q5, q6], [q(1), q(2), q(3), q(4), q(5), q(6)]);
end
else if s=7
    BQ=subs(Bq, [q1, q2, q3, q4, q5, q6, q7], [q(1), q(2), q(3),  q(4), q(5), q(6), q(7)]);
end
B=eval(BQ);
end