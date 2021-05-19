%% LQR Gain calculation
% Sample time
ts = Ts;
% System matrices
A = linsys.a;
B = linsys.b;
C = linsys.c;
D = linsys.d;

% create state space system
sys = ss(A,B,C,D);

% Setup Q and R tuning parameter matrices
Q = eye(12)*0.01
R = eye(14)*0.1
% Compute lqr gain [K]
[K, S, CLP] = lqr(sys, Q, R);