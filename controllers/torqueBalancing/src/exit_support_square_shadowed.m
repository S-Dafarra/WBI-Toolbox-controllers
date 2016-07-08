function OUT = exit_support_square_shadowed(x, supportp, shadow)
%computes whether x is outside the support polygon defined in supportp. x
%should be a two dimensional vector, while supportp a 2x2 matrix, where
%the first coloumn is the lower edge, while the first row corresponds to
%the first dimension of x. Notice that this check is done only if x is
%inside the shadowed region.

if((x(1)>shadow(1,1))||(x(1)<shadow(1,2))||(x(2)>shadow(2,1))||(x(2)<shadow(2,2)))
    if((x(1)<supportp(1,1))||(x(1)>supportp(1,2))||(x(2)<supportp(2,1))||(x(2)>supportp(2,2)))
         OUT = 1;
    else OUT = 0;
    end
else OUT=0;
end
end