%% Calculation of the LQR gain matrix [K] for the Altitude
% First define gravity and mass
g = 9.82;
m = 0.063; %grams

% Create system state space model
A = [0 1; 0 0];
B = [(1/m) (1/m)]';
C = [1 0];
D = 0;

% Create system from state space matrices
sys = ss(A,B,C,D);

% Define Q and R tuning matrices
Q = [   1/9 0 0;
        0 1/9 0;
        0 0 1/12];
    % Q_ii = maximum acceptable value of x_i^2
% For this size system R can only be of dimensions 1x1 -scalar
R = 1/(4*Controller.totalThrustMaxRelative*Controller.motorsThrustPerMotorMax)^2;
    % R_ii = maximum acceptable value of u_i^2

% Calculate LQI gain
[KLQI, S, CLP] = lqi(sys, Q, R, 0);
KLQI