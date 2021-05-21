%% Calculation of the LQR gain matrix [K] for the Altitude
% First define gravity and mass
g = 9.81;
m = 0.063; %grams
% Simplified system model
% m*zDotDot = -m*g + F

% State space representation
% xDot = A*x+B*u
% y = C*x
% xDot = [zDot zDotDot]^T
% x = [z zDot]^T
% A = [0 1; 0 0];
% B = [-g+1/m -g+1/m]^T
% C = [1 0];
A = [0 1; 0 0];
%B = [-g+(1/m) -g+(1/m)]';
B = [(1/m)+(g*m) (1/m)+(g*m)]';
C = [1 0];
D = 0;

% Create system from matrices
sys = ss(A,B,C,D);

% Define Q and R tuning matrices
Q = [   1/8 0;
        0 1/12    ];
    % Q_ii = maximum acceptable value of x_i^2
% For this size system R can only be of dimensions 1x1 -scalar
R = 1/(4*Controller.totalThrustMaxRelative*Controller.motorsThrustPerMotorMax)^2;
    % R_ii = maximum acceptable value of u_i^2

% Calculate the LQR gain K
[KLQR S CLP] = lqr(sys, Q, R, 0);
KLQR

% Feed forward control
N = inv([A B; C 0]);
%Nx = N(1:2,2:3);
%Nu = N(3,2:3);
Nx = N(1:2,3);
Nu = N(3,3);
NBar = Nu+KLQR*Nx;


%KLQR = [0.8 0.3];
% u = -Kx
% delta_u(t) = -K*delta_x(t)
% u(t) = -K*delta_x(t) + u^*
% u^* = I think is when u = g -> that is F = g - ie. the thrust should
% equal the gravity for equibrilibrium 