% Optimal LQ RL Inverted Pendulum on a Cart Demo
clear all
% Data Input: State Space Matrices
A=[0 0 1 0; 0 0 0 1; 0 40.2 -14.53 0; 0 82.56 -15.77 0];
B=[0;0;27.28;26.74];
C=[1 0 0 0;0 1 0 0];
D=[0;0];
x0=[1;1;1;1];% assumed initial condition
% LQ Optimal Control
R1=[10 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0];
R2=20;
% Controllability and Observability Tests
CO=ctrb(A,B); rankCO=rank(CO) 
% Forms the controllability matrix and finds the rank of   
% the controllability matrix.
% The rank is equal to n (system order)implies 
% the system is controllable.
OM=obsv(A,R1); rOM=rank(OM) 
% Rank equal to n implies observability.
% Note that the same rank is obtained by using             % obsv(A,Chol(R1)).
% Direct Solution of the Optimal LQ Problem
[Fopt,P]=lqr(A,B,R1,R2);
Jopt=0.5*x0'*P*x0
% Reinforcement Learning Solution of the Optimal LQ Problem
ev=eig(A);
% Since one eigenvalue is in the right half plane, need to 
% find F0 that stabilizes the  system. To find such a F0, 
% we can  use the MATLAB function place that finds the 
% feedback matrix that places the closed-loop matrix A-B*F0
% eigenvalues in the desired locations; 
% We can have complex conjugate eigenvalues
lambda_desired=[-5; -10; -20+j*15; -20-j*15]; 
F0=place(A,B,lambda_desired);
% Check eigenvalues of (A-B*F0)
evF0=eig(A-B*F0);
% Vaisbod-Milstein-Kleinman Reinforcer
R2I=inv(R2);
Fi=F0
time=0:0.01:4;
for i=1:9
    Pi=lyap((A-B*Fi)',R1+Fi'*R2*Fi);
    Ji(i)=0.5*x0'*Pi*x0;
    Fi=R2I*B'*Pi;
    for j=1:1:401;
    x(:,j)=expm((A-B*Fi)*time(j))*x0;
    u(j)=-Fi*x(:,j);
    end
    % Presentation of the obtained results
    figure (1)
    plot(time,x(1,:)); hold on
    xlabel('Time [s]');ylabel('State variable x1'); grid
    figure (2)
    plot(time,x(2,:)); hold on
    xlabel('Time [s]');ylabel('State variable x2'); grid 
    figure (3)
    plot(time,x(3,:)); hold on
    xlabel('Time [s]');ylabel('State variable x3'); grid
    figure (4)
    plot(time,x(4,:)); hold on
    xlabel('Time [s]');ylabel('State variable x4'); grid
    figure (5)
    plot(time,u); hold on
    xlabel('Time [s]');ylabel('Control input u'); grid
end
% Approximate optimal performance presentation
figure (6)
i=1:1:9
plot(i,Ji,'*')
grid; xlabel('Iteration number'); ylabel('Approximate optimal performance')
% It is nicer to present the changes of Ji in a table
% i=1, Ji=886.8558; i=2, Ji=520.0401; i=3, Ji=348.9477; i=4, Ji = 265.5725; 
% i=5, Ji=226.3736; i=6, Ji=211.8111; i=7, Ji=209.2012; i=8, Ji = 209.1143;
% i=9, Ji=209.1142=Jopt; i=10, Ji = 209.1142
