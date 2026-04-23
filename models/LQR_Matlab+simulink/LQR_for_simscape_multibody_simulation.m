%% ============================================================
%  LQR — Linear Quadratic Regulator
%% ============================================================

clear all; close all; clc;

%% --- Parameters (same as always) ---
M = 0.5;
m = 0.2;
l = 0.1;
lc = l/2;
J = (1/12)*m*l^2;
g = 9.81;
Delta = (m + M)*(m*lc^2 + J) - (m*lc)^2;

A = [0, 1,                    0, 0;
     0, 0, -(m^2*g*lc^2)/Delta, 0;
     0, 0,                    0, 1;
     0, 0, (m+M)*m*g*lc/Delta,  0];

B = [0; (m*lc^2+J)/Delta; 0; -m*lc/Delta];

C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = [0; 0];
%% ============================================================

% LQR minimizes the cost function:
%   J = integral( X'*Q*X + u'*R*u ) dt
%
% Q penalizes STATE errors (big Q = "I really don't want this state to deviate")
% R penalizes CONTROL EFFORT (big R = "don't use too much force")
%
% The ratio Q/R is what matters, not the absolute values.

% Q is 4x4 diagonal: [x penalty, x_dot penalty, theta penalty, theta_dot penalty]
% R is 1x1: force penalty

% Start with identity and tune from there:
Q = diag([1, 1, 10, 1]);
% Explanation:
%   Q(1,1) = 1  → moderate penalty on cart position deviation
%   Q(2,2) = 1  → moderate penalty on cart velocity
%   Q(3,3) = 10 → HIGH penalty on angle deviation (most important!)
%   Q(4,4) = 1  → moderate penalty on angular velocity

R = 1;   % penalty on force — increase this to use less force

% MATLAB solves the Riccati equation and returns optimal K
K_lqr = lqr(A, B, Q, R);

fprintf('LQR gain vector:\n');
fprintf('  K = [%.4f, %.4f, %.4f, %.4f]\n', K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4));

% Simulate
t=0:0.01:5;
X0=[0;0;0.1;1];
A_lqr = A - B*K_lqr;
sys_lqr = ss(A_lqr, B, C, D);
[y_lqr, t_lqr] = initial(sys_lqr, X0, t);

% The poles it chose:
fprintf('\nResulting closed-loop poles:\n');
disp(eig(A_lqr));
