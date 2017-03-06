function [OUT,K] = in_ch(x,xch,ych)
% DT = delaunayTriangulation(xch',ych');
% [~,Points] = freeBoundary(DT);
% in = inpolygon(x(1),x(2),Points(:,1),Points(:,2));

%shp = alphaShape(xch',ych',Inf);
%in = inShape(shp,x(1),x(2));
K = zeros(length(xch),1);
K_out = convhull(xch,ych);
K(1:length(K_out)) = K_out;
in = inpolygon(x(1),x(2),xch(K_out),ych(K_out));
OUT = 1 - in;