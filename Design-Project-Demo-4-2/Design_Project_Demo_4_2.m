% Optimal LQ RL Inverted Pendulum on a Cart Demo Live Script
% (pendulum_continuous.mlx)
% Optimal LQ RL Inverted Pendulum on a Cart Demo
[A,B,x0] = environment_data;
[R1,R2,Fopt] = optimal_control(A,B,x0);
% RL Mechanism
Ji=reinforcer(A,B,R2,R1,x0);
% Presentation of the obtained results
system_response(A,B,Fopt,x0);
% Approximate optimal performance presentation
rewards(Ji);
% Input Data Live  Function (environment_data.mlx)
function [A,B,x0] = environment_data
% Input Data: State Space Matrices
A=[0 0 1 0; 0 0 0 1; 0 40.2 -14.53 0; 0 82.56 -15.77 0];
B=[0;0;27.28;26.74];
C=[1 0 0 0;0 1 0 0]; D=[0;0];
x0=[1;1;1;1];% assumed initial conditions
end
% Optimal Control (optimal_control.mlx)
function [R1,R2,Fopt] = optimal_control(A,B,x0)
% LQ Optimal Control
R1=[10 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0];
R2=20;
% Controllability and Observability Tests
CO=ctrb(A,B); rankCO=rank(CO) 
% finds the rank of the controllability matrix
% the rank is equal to n (system order)implies 
% controllability
OM=obsv(A,R1); rankOM=rank(OM) 
% rank equal to n implies observability
% Note that the same rank is obtained by using obsv(A,Chol(R1))
% Direct Solution of the Optimal LQ Problem
[Fopt,P]=lqr(A,B,R1,R2);
Jopt=0.5*x0'*P*x0
end
% Reinforcer Live Function (reinforcer.mlx)
function Ji = rl_environment_dynamics(A,B,R2,R1,x0)
% Reinforcement Learning Solution of the Optimal LQ Problem
ev=eig(A);
% Since one eigenvalue is in the right half plane, need to 
% find F0 that stabilizes the  system. To find such a F0, 
% we can  use the MATLAB function place that finds the 
% feedback matrix that places the closed-loop matrix A-B*F0
% eigenvalues in the desired locations; 
% can have complex conjugate eigenvalues
lambda_desired=[-5; -10; -20+j*15; -20-j*15]; 
F0=place(A,B,lambda_desired);
% check eigenvalues of (A-B*F0)
evF0=eig(A-B*F0);
%
% Vaisbod-Milstein-Kleinman Reinforcer
R2I=inv(R2);
for i=1:10
    Pi=lyap((A-B*F0)',R1+F0'*R2*F0);
    i
    Ji(i)=0.5*x0'*Pi*x0
    F0=R2I*B'*Pi;
end
end
% System Responses Live Function (system_responses.mlx)
function system_response(A,B,Fopt,x0)
% Plotting state responses over 5 [s] 
time=0:0.01:5;
for j=1:1:501;
    x(:,j)=expm((A-B*Fopt).*time(j))*x0;
end
figure (1)
plot(time,x(1,:))
grid; xlabel('Time [s]');ylabel('State variable x1')
figure (2)
plot(time,x(2,:))
grid; xlabel('Time [s]');ylabel('State variable x2')
figure (3)
plot(time,x(3,:))
grid; xlabel('Time [s]');ylabel('State variable x3')
figure (4)
plot(time,x(4,:))
grid; xlabel('Time [s]');ylabel('State variable x4')
end
% Rewards (rewards.mlx)
% function rewards(Ji)
figure (5)
i=1:1:10
plot(i,Ji(i),'*')
grid; xlabel('Iteration number'); ylabel('Approximate optimal performance')
% It is nicer to present the changes of Ji in a table
% i=1  Ji=886.9 % i=2  Ji=520.0 % i=3  Ji=348.9
% i=4  Ji=265.6 % i=5  Ji= 226.4 % i=6  Ji= 211.8
% i=7  Ji= 209.2 % i=8  Ji= 209.1 = Jopt
% i=9  Ji= 209.1 % i=10 Ji= 209.1

