function [Jc, dJcNu] = compSmoothJacobians(constraints,JL,JR, dJLNu,dJRNu)
%codegen
Jc     = [ JL*constraints(1) ;
           JR*constraints(2)];
       
dJcNu = [ dJLNu*constraints(1) ;
           dJRNu*constraints(2)];
       
end