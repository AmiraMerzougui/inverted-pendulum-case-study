%% ============================================================
%  INVERTED PENDULUM — Step 3: PID Controller Design
%% ============================================================

clear all; close all; clc;

%% --- PARAMETERS ---
M = 0.5;
m = 0.2;
l = 0.1;
lc = l/2;
J = (1/12)*m*l^2;
g = 9.81;
Delta = (m + M)*(m*lc^2 + J) - (m*lc)^2;

A = [0, 1,                   0, 0;
     0, 0, -(m^2*g*lc^2)/Delta, 0;
     0, 0,                   0, 1;
     0, 0, (m+M)*m*g*lc/Delta,  0];

B = [0; (m*lc^2+J)/Delta; 0; -m*lc/Delta];

C = [1, 0, 0, 0;   % output 1: x
     0, 0, 1, 0];  % output 2: theta

D = [0; 0];

sys = ss(A, B, C, D);

%% --- EXTRACT THE THETA SUBSYSTEM ---
% For PID design, we focus on the transfer function from F to theta.
% This is the "plant" our PID will control.
%
% sys has 2 outputs. We want output 2 (theta) only.
% sys(2,1) means: output 2, input 1

sys_theta = sys(2, 1);   % transfer function: F → theta

% Convert to transfer function form to see poles and zeros clearly
G_theta = tf(sys_theta);
disp('Transfer function F → theta:');
disp(G_theta);

% You'll see an unstable pole (positive real part) confirming instability.
%% --- PID CONTROLLER STRUCTURE ---
% A PID controller computes: F = Kp*e + Ki*integral(e) + Kd*derivative(e)
% where e = theta_reference - theta_measured
%       theta_reference = 0 (we want pendulum upright)
%
% In transfer function form:
%   C(s) = Kp + Ki/s + Kd*s = (Kd*s² + Kp*s + Ki) / s
%
% We start with manual tuning using known rules, then refine.

%% --- INITIAL TUNING (manual starting point) ---
% Rule of thumb for unstable plants:
% Start with Kp large enough to overcome the instability,
% Kd to add damping, Ki small (to avoid integrator wind-up).

Kp = 100;   % proportional gain — main stabilizing force
Ki = 1;     % integral gain — removes steady-state error (small for now)
Kd = 20;    % derivative gain — adds damping, reduces oscillation

% Build the PID controller as a transfer function
% pid() creates a continuous-time PID object
C_pid = pid(Kp, Ki, Kd);

disp('PID controller:');
disp(C_pid);

%% --- CLOSE THE LOOP ---
% The feedback loop is:
%   reference (theta_ref=0) → [+] → PID → F → Plant → theta → [-] → back
% 'feedback(C*G, 1)' creates the closed-loop system
% The '1' means unity feedback (we feed back theta directly)

open_loop = C_pid * sys_theta;        % PID × Plant (open-loop)
closed_loop = feedback(open_loop, 1); % close the loop with negative feedback

disp('Closed-loop poles:');
disp(pole(closed_loop));
% ALL poles must have NEGATIVE real parts for stability.
% If any pole has positive real part → unstable → increase Kp or Kd.

%% --- SIMULATE STEP RESPONSE ---
% A "step response" means: at t=0, we command theta_ref to jump from 0
% to a small value (0.1 rad). We watch how fast the system responds.
% For pendulum stabilization, we actually want disturbance rejection:
% start at theta=0.1 rad, try to return to 0.

t = 0:0.001:5;  % fine time step for smooth curves

% Initial condition response: theta starts at 0.1 rad, try to stabilize
X0 = [0; 0; 0.1; 0];

% For closed-loop with initial conditions, we use lsim
% First build the full closed-loop state-space
sys_cl_full = ss(A - B*[0,0,Kp,Kd], B, C, D);
% Explanation: the PID feedback modifies the A matrix.
% With proportional + derivative only (Kp on theta, Kd on theta_dot):
% F = -Kp*theta - Kd*theta_dot
% This changes: A_cl = A - B*K where K = [0, 0, Kp, Kd]
% (zeros on x and x_dot because this PID only sees theta)

%% --- PROPER CLOSED-LOOP SIMULATION WITH lsim ---
% State feedback: F = -K * X
% where K = [Kx, Kx_dot, Kp, Kd]
% For PID on theta only: K = [0, 0, Kp, Kd]  (no cart feedback yet)
% For full control later: K = [K1, K2, K3, K4]

K_pid = [0, 0, Kp, Kd];  % gain vector (ignoring x for now)

A_cl = A - B*K_pid;        % closed-loop A matrix
% Explanation: the control law u = -K*X substituted into X_dot = AX + Bu
% gives X_dot = AX + B*(-K*X) = (A - BK)*X → new system matrix is A-BK

sys_cl = ss(A_cl, B, C, D);

% Simulate initial condition response
[y_cl, t_cl, x_cl] = initial(sys_cl, X0, t);

%% --- PLOT CLOSED-LOOP RESULTS ---

figure;

subplot(3,1,1);
plot(t_cl, y_cl(:,2), 'b', 'LineWidth', 2);
% y_cl(:,2) = theta over time
yline(0, 'r--', 'Reference (0 rad)');  % draw a red dashed line at 0
xlabel('Time (s)');
ylabel('Theta (rad)');
title(sprintf('Pendulum angle — PID (Kp=%.0f, Ki=%.0f, Kd=%.0f)', Kp, Ki, Kd));
grid on;

subplot(3,1,2);
plot(t_cl, y_cl(:,1), 'g', 'LineWidth', 2);
% y_cl(:,1) = cart position x
xlabel('Time (s)');
ylabel('Cart position x (m)');
title('Cart position (drifts without x-control)');
grid on;

subplot(3,1,3);
% Reconstruct the control force F = -K*X
F_control = -(K_pid * x_cl')';  % matrix multiply K by each state vector
% x_cl is (n_timesteps × 4), K_pid is (1×4)
% K_pid * x_cl' gives (1×n_timesteps), we transpose to get column
plot(t_cl, F_control, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Force F (N)');
title('Control signal (force applied to cart)');
grid on;

sgtitle('Inverted Pendulum — PID Closed-loop Response');

%% --- PERFORMANCE ANALYSIS ---
% Let's measure how good our controller is

% Find settling time (when theta stays within 2% of 0)
theta = y_cl(:,2);
settled_idx = find(abs(theta) < 0.02 * abs(theta(1)), 1, 'last');
% This finds the last time theta is still outside 2% band
settling_time = t_cl(end);  % conservative estimate

fprintf('\n--- PID Performance ---\n');
fprintf('Initial angle: %.2f rad (%.1f degrees)\n', X0(3), X0(3)*180/pi);
fprintf('Max theta: %.4f rad\n', max(abs(theta)));

% Check all closed-loop poles
cl_poles = eig(A_cl);
fprintf('\nClosed-loop poles:\n');
disp(cl_poles);
all_stable = all(real(cl_poles) < 0);
if all_stable
    fprintf('System is STABLE with this PID!\n');
else
    fprintf('WARNING: System still UNSTABLE — increase Kp or Kd\n');
end


%% --- TUNING GUIDANCE (run this section multiple times) ---
% Effect of each gain:
%
% Increase Kp → faster response, but too much → oscillation or instability
% Increase Kd → more damping, less overshoot, but too much → noisy
% Increase Ki → removes steady-state error, but too much → slow/unstable
%
% Try these sets and compare:
% Set 1: Kp=100, Ki=1,  Kd=20  (starting point)
% Set 2: Kp=150, Ki=1,  Kd=25  (more aggressive)
% Set 3: Kp=100, Ki=5,  Kd=20  (more integral action)
% Set 4: Kp=200, Ki=0,  Kd=30  (PD only — often works well for unstable)
