function [OUT,K] = in_ch(x,xch,ych)
% DT = delaunayTriangulation(xch',ych');
% [~,Points] = freeBoundary(DT);
% in = inpolygon(x(1),x(2),Points(:,1),Points(:,2));

%shp = alphaShape(xch',ych',Inf);
%in = inShape(shp,x(1),x(2));

K = convhull(xch,ych);
in = inpolygon(x(1),x(2),xch(K),ych(K));
OUT = 1 - in;