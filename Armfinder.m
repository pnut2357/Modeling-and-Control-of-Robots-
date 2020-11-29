function Arm_calc = Armfinder(Arm, q)
%evaluates symbolic Arm for a certain q
s=size(q, 2);
if s=1
    Arm=subs(Arm, q1, q(1));
end
else if s=2
    Arm=subs(Arm, [q1, q2], [q(1), q(2)]);
end
else if s=3
    Arm=subs(Arm, [q1, q2, q3], [q(1), q(2), q(3)]);
end
else if s=4
    Arm=subs(Arm, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]);
end
else if s=5
    Arm=subs(Arm, [q1, q2, q3, q4, q5], [q(1), q(2), q(3), q(4), q(5)]);
end
else if s=6
    Arm=subs(Arm, [q1, q2, q3, q4, q5, q6], [q(1), q(2), q(3), q(4), q(5), q(6)]);
end
else if s=7
    Arm=subs(Arm, [q1, q2, q3, q4, q5, q6, q7], [q(1), q(2), q(3),  q(4), q(5), q(6), q(7)]);
end
Arm_calc=eval(Arm);

T=zeros(4);
%T=vpa(Arm.fkine(q));
T=Arm_calc.fkine(q);

%ZYZ Euler angles
eul=tr2eul(T);

%xe: 6 x 1 vector of actual 6-DOF arm position
xe=zeros(6, 1)
xe=[T(13); T(14); T(15); eul(1); eul(2); eul(3)];
%what is the type of xe???? what is its size?
end

