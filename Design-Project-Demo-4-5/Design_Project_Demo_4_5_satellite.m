% Optimal LQ RL for a Satellite Demo
% Calling Simulink from MATLAB script loops
clear; close all; clc
omega=7.222*10^(-5);
% Input Data: State Space Matrices
A=[0 0 0 1 0 0;
   0 0 0 0 1 0; 
   0 0 0 0 0 1;
   3*omega^2 0 0 0 2*omega 0
   0 0 0 -2*omega 0 0;
   0 0 (-1)*omega^2 0 0 0]
B=[zeros(3,3);eye(3)]
C=[eye(3) zeros(3,3)]
D=zeros(3,3)
x0=[10000; 10000; -10000; 2; 1; 3];% assumed initial condition in [18]
% LQ Optimal Control
R1=eye(6); R2=eye(3);
% Controllability and Observability Tests
CO=ctrb(A,B); rankCO=rank(CO)
% finds the rank of the controllability matrix
% the rank is equal to n (system order)implies controllability
OM=obsv(A,R1); rankOM=rank(OM)
% rank equal to n implies observability
% Note that the same rank is obtained by using 
% obsv(A,Chol(R1))
% Direct Solution of the Optimal LQ Problem
[Fopt,P]=lqr(A,B,R1,R2)
Jopt=0.5*x0'*P*x0
% Reinforcement Learning Solution of the Optimal LQ Problem
ev=eig(A);
% Since one eigenvalue is in the right half plane, need to 
% find F0 that stabilizes the  system. To find such a F0, 
% we can  use the MATLAB function place that finds the 
% feedback matrix that places the closed-loop matrix A-B*F0
% eigenvalues in the desired locations; 
% can have complex conjugate eigenvalues
lambda_desired=[-1; -3; -5; -7+i*8; -7-i*8; -10]; 
F0=place(A,B,lambda_desired);
% check eigenvalues of (A-B*F0)
evF0=eig(A-B*F0);
% Vaisbod-Milstein-Kleinman Reinforcer
R2I=inv(R2);
iter = 10;
for i=1:iter
    Pi=lyap((A-B*F0)',R1+F0'*R2*F0);
    i
    Ji(i)=0.5*x0'*Pi*x0
    F0=R2I*B'*Pi;
    % Calling Simulink
    model=sim(satellite)
    time_itr(1:length(model.t),i) = model.t;
    X_1(1:length(model.t),i) = model.x(:,1);
    X_2(1:length(model.t),i) = model.x(:,2);
    X_3(1:length(model.t),i) = model.x(:,3);
    X_4(1:length(model.t),i) = model.x(:,4);
    X_5(1:length(model.t),i) = model.x(:,5);
    X_6(1:length(model.t),i) = model.x(:,6);
   % U_1(1:length(model.t),i,1:size(model.u1(:,:),2))=model.u1(:,:);
    U_1(1:length(model.t),i) = model.u(:,1);
    U_2(1:length(model.t),i) = model.u(:,2);
    U_3(1:length(model.t),i) = model.u(:,3);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
markers = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};
lineStyles = {'-', '--', ':', '-.'};
figure (1)
 for i=1:iter
     hold on
     Iname_1 = sprintf('Iteration %d', i);
     plot(time_itr(:,i),X_1(:,i),'Marker',markers{mod(i,iter+1)},...
         'linestyle',lineStyles{max(mod(i,4),1)},'DisplayName',Iname_1)     
 end
grid on
set(gca,'GridLineStyle','--')
xlabel('Time [s]');title('State variable x_1')
legend;
figure (2)
 for i=1:iter
     hold on
     Iname_2 = sprintf('Iteration %d', i);
     plot(time_itr(:,i),X_2(:,i),'Marker',markers{mod(i,iter+1)},...
         'linestyle',lineStyles{max(mod(i,4),1)},'DisplayName',Iname_2)
 end
 grid on
 set(gca,'GridLineStyle','--')
 legend; xlabel('Time [s]');title('State variable x_2')
 figure (3)
  for i=1:iter
     Iname_3 = sprintf('Iteration %d', i);
     plot(time_itr(:,i),X_3(:,i),'Marker',markers{mod(i,iter+1)},...
         'linestyle',lineStyles{max(mod(i,4),1)},'DisplayName',Iname_3)
     hold on
 end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('State variable x_3')
figure (4)
 for i=1:iter
     Iname_4 = sprintf('Iteration %d', i);
     plot(time_itr(:,i),X_4(:,i),'Marker',markers{mod(i,iter+1)},...
         'linestyle',lineStyles{max(mod(i,4),1)},'DisplayName',Iname_4)
     hold on
 end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('State variable x_4')
figure (5)
 for i=1:iter
     Iname_5 = sprintf('Iteration %d', i);
     plot(time_itr(:,i),X_5(:,i),'Marker',markers{mod(i,iter+1)},...
         'linestyle',lineStyles{max(mod(i,4),1)},'DisplayName',Iname_5)
     hold on
 end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('State variable x_5')
figure (6)
 for i=1:iter
     Iname_6 = sprintf('Iteration %d', i);
     plot(time_itr(:,i),X_6(:,i),'Marker',markers{mod(i,iter+1)},...
         'linestyle',lineStyles{max(mod(i,4),1)},'DisplayName',Iname_6)
     hold on
 end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('State variable x_6')
% Plotting approximate optimal controls (actions)
figure (7)
   for i=1:iter
Iname_u1 = sprintf('Iteration %d', i);
plot(time_itr(:,i),U_1(:,i),'Marker',markers{mod(i,iter+1)},...          
    'linestyle',lineStyles{max(mod(i,2),1)},'DisplayName',Iname_u1)
hold on
   end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('control u_1(t)')
% axis([0 2 -50 200])
figure (8)
   for i=1:iter
Iname_u2 = sprintf('Iteration %d', i);
plot(time_itr(:,i),U_2(:,i),'Marker',markers{mod(i,iter+1)},...
    'linestyle',lineStyles{max(mod(i,2),1)},'DisplayName',Iname_u2)
hold on
   end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('disturbance u_2(t)')
% axis([0 2 -50 15])
figure (9)
   for i=1:iter
Iname_u3 = sprintf('Iteration %d', i);
plot(time_itr(:,i),U_3(:,i),'Marker',markers{mod(i,iter+1)},...
    'linestyle',lineStyles{max(mod(i,2),1)},'DisplayName',Iname_u3)
hold on
   end
grid on
set(gca,'GridLineStyle','--')
legend; xlabel('Time [s]');title('disturbance u_3(t)')
% axis([0 2 -50 15])



