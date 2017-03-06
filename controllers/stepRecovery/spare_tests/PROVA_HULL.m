wHl = [eye(3), [0;0;0];
       zeros(1,3),1];
   
theta = -30/180*pi;

wHr = [cos(theta), -sin(theta), 0,  0.05;
       sin(theta),  cos(theta), 0,   0.3;
       0,                    0, 1,     0;
       0,                    0, 0,     1];
   
foot_size =  [ -0.05   0.10 ;    
               -0.02   0.02];
       
p = [0.12;0]; 
           
[OUT, xch, ych, K_in] = in_convex_hull(p,wHl, wHr, foot_size);
K = K_in(1:sum(K_in>0));
figure
hold on
plot([xch(1:4),xch(1)],[ych(1:4),ych(1)])
plot([xch(5:8),xch(5)],[ych(5:8),ych(5)],'b')
plot(xch(K),ych(K),'r')
ch_points = [xch(K)',ych(K)'];
if(sum(ch_points(1,:)==ch_points(end,:))==2)
    pm = mean(ch_points(1:(end-1),:))';disp('Extra point');
else pm = mean(ch_points)'; disp('No extra point');
end
plot(pm(1),pm(2),'+');
plot(p(1),p(2),'*');
axis equal

OUT

[A,B] = ch_constraints([xch(K)',ych(K)']);
% hull_points = [xch(K)',ych(K)'];
% 
% [npoints,direction] = max([size(hull_points,1),size(hull_points,2)]);
% assert(npoints > 2);
% 
% hp = zeros(npoints,2);
% if (direction == 1)
%     hp = hull_points;
% else hp = hull_points';
% end
% 
% if(sum(hp(1,:)==hp(npoints,:))==2)
%     npoints = npoints-1;
% end
% 
% A = zeros(npoints,2);
% B = zeros(npoints,1);
% for i=1:npoints
%     P1 = hp(i,:);
%     P2 = hp(rem(i, npoints) + 1,:);
%     
%     A(i,:) = [P2(2)-P1(2),P1(1)-P2(1)];
%     B(i) = P1(1)*P2(2) - P2(1)*P1(2);
% end
test = gt(A*p,B);% A and B are defined such that if A*p<=B than the point is in the convex hull
if sum(test) == 0
    OUTc = 0
else OUTc = 1
end
    
pm = mean([xch(K)',ych(K)'])';



   