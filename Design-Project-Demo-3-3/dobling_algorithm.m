% Doubling Algorithm
%% Example 4 -PEM Fuel Cell
Ac = [-6.30908 0 -10.9544 0 83.74458 0 0 24.05866;
0 -161.083 0 0 51.52923 0 -18.0261 0;
-18.7858 0 -46.3136 0 275.6592 0 0 158.3741;
0 0 0 -17.3506 193.9373 0 0 0;
1.299576 0 2.969317 0.3977 -38.7024 0.105748 0 0;
16.64244 0 38.02522 5.066579 -479.384 0 0 0;
0 -450.386 0 0 142.2084 0 -80.9472 0;
2.02257 0 4.621237 0 0 0 0 -51.2108];
Bc = [0; 0; 0; 3.946683; 0; 0; 0; 0];
Cc = [0 0 0 5.066579 -116.446 0 0 0;
0 0 0 0 1 0 0 0;
12.96989 10.32532 -0.56926 0 0 0 0 0];
% Create state space model
sys = ss(Ac,Bc,Cc,zeros(3,1));
% Convert to discrete, where dt is your discrete time-step (in seconds)
d_sys = c2d(sys,0.025);
A = d_sys.A; B = d_sys.B; C = d_sys.C;
Q = eye(8); R=100; x0 = ones(8,1);
% Check ctrb, obsv,
eig(A)
rank(ctrb(A,B))
rank(obsv(A,C))
%% P by Riccati --DARE
[P,L,Fopt] = dare(A,B,Q,R) % L: closed loop eigenvalues
% % [F,P]=dlqr(A,B,Q,R)
Jopt=0.5*x0'*P*x0
%
% Doubling Algorithm
%
M0=A;
N0=B*inv(R)*B';
V0=Q;
for i=1:10
    W0=inv(eye(8)+N0*V0);
    Mi=M0*W0*M0;
    Ni=N0+M0*W0*N0*M0';
    Vi=V0+M0'*V0*W0*M0;
    M0=Mi;
    N0=Ni;
    V0=Vi;
    i
    error=norm(Vi-A'*Vi*A+A'*Vi*B*inv(R+B'*Vi*B)*B'*Vi*A,2);
    Error=norm(P-Vi,2)
end
Vi;

