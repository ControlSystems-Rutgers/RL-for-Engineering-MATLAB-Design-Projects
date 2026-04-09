clear all
% LQ Problem for a Simple Pendulum
% Input Data
A=[0 1;-64*cos(pi/6)  -5];
B=[0;1];
Q=eye(2);
R=1;
x0=[0.1; -0.1];
% Testing Assumption 2.1
[rankCM,n]=controllability(A,B)
C=chol(Q);
[rankOM,n]=observability(A,C)
% Finding the optimal feedback gain and the optimal crtiterion
if(rankOM==n & rankCM==n)
   [Fopt,Jopt]=optimal_LQ(A,B,Q,R,x0);
else
   disp(['Optimal control does not exist'])
end


