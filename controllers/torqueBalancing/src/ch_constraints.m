function [A,B] = ch_constraints(hull_points)
%%N.B.: THE POINTS OF THE CONVEX HULL SHOULD BE IN A COUNTER CLOCKWISE
%%ORDER!
%%if Ax<=B then x is in the convex hull

[npoints,direction] = max([size(hull_points,1),size(hull_points,2)]);
assert(npoints > 2);

hp = zeros(npoints,2);
if (direction == 1)
    hp = hull_points;
else hp = hull_points';
end

if(sum(hp(1,:)==hp(npoints,:))==2)
    npoints = npoints-1;
end

A = zeros(npoints,2);
B = zeros(npoints,1);
for i=1:npoints
    P1 = hp(i,:);
    P2 = hp(rem(i, npoints) + 1,:);
    
    A(i,:) = [P2(2)-P1(2),P1(1)-P2(1)];
    B(i) = P1(1)*P2(2) - P2(1)*P1(2);
    
end

