% RL for an Inverted Pendulum on a Cart - DEMO
clear all
% Continuous-time inverted pendulum on a cart model
Ac=[0 0 1 0; 
    0 0 0 1; 
   0 40.2 -14.53 0; 
   0 82.56 -15.77 0];
Bc=[0;0;27.28;26.74];
Cc=[1 0 0 0; 0 1 0 0];
Dc=[0;0];
sysc=ss(Ac,Bc,Cc,Dc); % continuous-time state space model
% this model can be discretized using MATLAB for the given 
Ts=0.1 % selected the sampling period Ts
sysd=c2d(sysc,Ts); % discrete-time state space model
% extraction of discrete-time state space matrices
[Ad,Bd,Cd,Dd]=ssdata(sysd); 
x0=[1;1;1;1]; % assumed initial conditions
% LQ Optimal Control
R1=0.01*eye(4); R2=1; 
% assumed performance criterion penalty matrices
% Test for controllability and observability
CO=ctrb(Ad,Bd);
rankCO=rank(CO) % rank of the controllability matrix
% rank equal to n, system is controllable
OM=obsv(Ad,R1); % rank of the observability matrix
rankOM=rank(OM) 
% rank equal to n, system is  observable
% the same rank is obtained by using obsv(Ad,Chol(R1))
% direct solution of the discrete optimal LQ problem
[Fopt,P]=dlqr(Ad,Bd,R1,R2);Jopt=0.5*x0'*P*x0
% Reinforcement learning solution
ev=eig(Ad);
% To find F0 that stabilize the system, we can use the 
% MATLAB function "place" that finds the feedback gain
% matrix that places the closed-loop eigenvalues of 
% Ad-Bd*F0 in the desired locations
lambda_des=[-0.5;0.2;-0.1+j*0.5;-0.1-j*0.5];
F0=place(Ad,Bd,lambda_des);
Fi=F0
% check eigenvalues Ad-Bd*F0; evF0=eig(Ad-Bd*F0);
% Hewer's Discrete-Time LQ Optimal Reinforcer
for i=1:15
    Pi=dlyap((Ad-Bd*Fi)',R1+Fi'*R2*Fi);
    i
    Ji(i)=0.5*x0'*Pi*x0
    Fi=inv(R2+Bd'*Pi*Bd)*Bd'*Pi*Ad;
    k=1:1:100;
    % evaluating approximate states and control
for j=1:100
     x(:,j)=((Ad-Bd*Fi)^k(j))*x0;
     u(j)=-Fi*x(:,j);
end
% plotting approximate states, control, performance
figure (1)
stairs(k,x(1,:));
hold on
xlabel('Time [s]');ylabel('State variable x1'); grid
figure (2)
stairs(k,u(k));
hold on
xlabel('Time [s]');ylabel('Approximate control'); grid
axis([0 20 -8 10])
end
figure (3)
i=1:1:15
plot(i,Ji,'*')
grid; xlabel('iteration number'); 
ylabel('approximate optimal performance')
% it is nicer to present the changes of Ji in a table which 
% will also provide information about the decimal digits
% i=1  Ji=4269.3540
% i=2  Ji=1080.0163
% i=3  Ji= 442.5366
% i=4  Ji= 237.4153
% i=5  Ji= 158.0270
% i=6  Ji= 123.0368
% i=7  Ji= 106.8144
% i=8  Ji=  99.3348
% i=9  Ji=  96.1003 
% i=10 Ji=  95.0129
% i=11 Ji=  94.8454
% i=12 Ji=  94.8414
% i=13 Ji=  94.8411 = Jopt
% i=14 Ji=  94.8411
% i=15 Ji=  94.8411
