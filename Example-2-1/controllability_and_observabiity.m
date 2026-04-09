% Control system controllability and observability examination
n=2;
disp(['System order n = ', num2str(n)])
% "num2str" is used to display numerics with the text
A=[-1 1; 
   0 -2];
B=[0;1];
C=[1 0];
% calling live function that examines controllability
controllability(A,B);
% calling live function that examines observability
observability(A,C);