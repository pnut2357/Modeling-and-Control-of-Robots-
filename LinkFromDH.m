function [L] = LinkFromDH(DH, J)
%{
Inputs:
DH: DH parameters [a alpha d theta]
J: Joints
Outputs:
L: collection of links
%}

for i=1:length(J,1)
   if J(i)=="r"
       L{i}=Link('revolute','a',DH(i,1),'alpha',DH(i,2),'d',DH(i,3),'theta',DH(i,4));
   else
       L{i}=Link('prismatic','a',DH(i,1),'alpha',DH(i,2),'d',DH(i,3),'theta',DH(i,4),);
   end
end
end

