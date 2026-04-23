%% =========================================================================
%INVERTED PENDULUM ON A CART : Open_Loop Simulation
% State Verctor: X=[x ,x_dot, theta, theta_dot]
% Input: F(force on the cart)
%% =========================================================================

clear all; %clear all variables from workspace 
close all; %close all figure windows
clc; %clear command window

%% ========= System Parameters =====================

%these parameters are used in many textbooks 
M=0.5; %masse of cart (Kg)
m=0.2; %mass of pendulum bob (Kg)
l=0.1; %length of pendulum bob (m)
lc=l/2;%distance from pivot to center of mass = l/2
J = (1/12)*m*l^2;  % moment of inertia of uniform rod about its center
g = 9.81;    % gravitational acceleration (m/s²)

%% ========= Determinant (delta)======================
Delta = (m + M)*(m*lc^2 + J) - (m*lc)^2;
fprintf("Delta =%.4f\n", Delta); %this number should be positive , if Delta=0 , the system is singular

%% ========= State Matrix A (4*4) ==================
A = [0,    1,                    0,   0;
     0,    0,   -(m^2*g*lc^2)/Delta,  0;
     0,    0,                    0,   1;
     0,    0,  (m+M)*m*g*lc/Delta,    0];
disp('Matrix A:');
disp(A);
% KEY INSIGHT: Look at A(4,3) — it is POSITIVE.
% This means: if theta increases slightly → theta_ddot increases too
% → theta increases more → runaway. That is why the system is unstable.

%% ======== INPUT MATRIX B (4x1) =================
% B tells us how the force F enters each state equation.
% x and theta positions are not directly affected by F (rows 1 and 3 = 0)
% Only the accelerations (rows 2 and 4) are affected.

B = [0;
     (m*lc^2 + J)/Delta;
     0;
     -m*lc/Delta];

disp('Matrix B:');
disp(B);

%% ========== OUTPUT MATRIX C and D ===========
% C selects which states we "measure" (observe as outputs).
% We want to observe: x (cart position) and theta (pendulum angle)
% x is state 1, theta is state 3
% C is 2x4: two outputs, four states

C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = [0; 0];  % no direct feedthrough (F doesn't instantly appear in output)

%% ======== CREATE THE STATE-SPACE SYSTEM OBJECT =============

sys = ss(A, B, C, D);

disp('State-space system created.');
disp('Outputs: [x (cart position), theta (pendulum angle)]');
disp('Input:   [F (force on cart)]');


%% ============ CHECK STABILITY — EIGENVALUES ==========

poles = eig(A);
disp('Open-loop poles (eigenvalues of A):');
disp(poles);
%% ========== OPEN-LOOP SIMULATION (initial condition response) =======
% We give the system a tiny initial tilt (theta = 0.1 rad ≈ 5.7°)
% and zero everything else. No force F is applied (u = 0).
% We watch what happens over 5 seconds.

t = 0:0.01:5;          % time vector: 0 to 5 seconds, step 0.01s
X0 = [0; 0; 0.1; 0];  % initial state: [x=0, x_dot=0, theta=0.1rad, theta_dot=0]

% 'initial' simulates the response to initial conditions with u=0
[y, t_out, x_out] = initial(sys, X0, t);

% y has 2 columns: column 1 = x(t), column 2 = theta(t)
% x_out has 4 columns: all 4 states over time


%% ========= PLOT THE RESULTS ==============

figure; % Optional: explicitly opens a new window

subplot(2,1,1);   % first plot in a 2-row layout
plot(t_out, y(:,2), 'b', 'LineWidth', 1.5);
% y(:,2) is theta (all time steps, column 2)
xlabel('Time (s)');
ylabel('Theta (rad)');
title('Open-loop response — Pendulum angle (unstable)');
grid on;

subplot(2,1,2);   % second plot
plot(t_out, y(:,1), 'r', 'LineWidth', 1.5);
% y(:,1) is x (cart position)
xlabel('Time (s)');
ylabel('Cart position x (m)');
title('Open-loop response — Cart position');
grid on;

sgtitle('Inverted Pendulum — Open-loop (no controller)');

%% ======== CONTROLLABILITY CHECK ==============
% Before designing a controller, we MUST verify the system is controllable.
% Controllability means: can we drive the system from any state to any other state using the input F?
% Method: build the controllability matrix Co = [B, AB, A²B, A³B]
% If rank(Co) = n (number of states = 4), the system IS controllable.

Co = ctrb(A, B);        % MATLAB builds the controllability matrix
rank_Co = rank(Co);      % compute its rank

fprintf('Rank of controllability matrix: %d (should be 4)\n', rank_Co);

if rank_Co == 4
    disp('System is CONTROLLABLE — we can design a controller!');
else
    disp('WARNING: System is NOT fully controllable.');
end