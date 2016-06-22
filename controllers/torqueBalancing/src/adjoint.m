function s_A_b = adjoint (R,p)

p_hat = [       0, -p(3),  p(2);
          p(3),        0, -p(1);
         -p(2),  p(1),        0];
     
s_A_b = [R,             p_hat*R;
         zeros(3,3),       R];
end
