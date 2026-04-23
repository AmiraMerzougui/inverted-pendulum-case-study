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

%% ============================================================
%  GENERATE ALL REQUIRED FIGURES FROM SIMULINK OUTPUT
%  Run this after simulation completes(simulink_linear_model)
%% ============================================================

% After Simulink runs, these variables exist in workspace:
% tout        — time vector
% states_out  — Nx4 matrix: [x, x_dot, theta, theta_dot]
% F_out       — Nx1 force vector

% Extract individual signals
x_cart   = states_out(:, 1);   % cart position
x_dot    = states_out(:, 2);   % cart velocity
theta    = states_out(:, 3);   % pendulum angle (radians)
theta_dot = states_out(:, 4);  % angular velocity

%% ============================================================
%  FIGURE 1 — Open-loop response (run this separately FIRST
%  using your open-loop script, save the data, then overlay)
%  Here we just show closed-loop and label it correctly
%% ============================================================

figure('Name', 'Fig 1 — Pendulum Angle Response','Position', [100, 100, 800, 500]);

plot(tout, theta * 180/pi, 'b-', 'LineWidth', 2);
hold on;
yline(0, 'r--', 'LineWidth', 1.5, 'Label', 'Reference \theta = 0°');
% Mark settling time
threshold = 0.02 * 0.1 * 180/pi;  % 2% of initial angle in degrees
settled_idx = find(abs(theta * 180/pi) > threshold, 1, 'last');
if ~isempty(settled_idx) && settled_idx < length(tout)
    xline(tout(settled_idx), 'g--', 'LineWidth', 1.2, ...
        'Label', sprintf('Settling: %.2fs', tout(settled_idx)));
end

xlabel('Time (s)', 'FontSize', 13);
ylabel('\theta (degrees)', 'FontSize', 13);
title('Pendulum angle \theta(t) — LQR closed-loop', 'FontSize', 14);
legend('\theta(t)', 'Reference', 'FontSize', 11);
grid on;
% Add text box with gain values
text(0.02, 0.95, sprintf('K = [%.1f, %.1f, %.1f, %.1f]', ...
    K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4)), ...
    'Units', 'normalized', 'FontSize', 10, ...
    'VerticalAlignment', 'top', 'BackgroundColor', 'white');

%% ============================================================
%  FIGURE 2 — Cart position x(t)
%% ============================================================

figure('Name', 'Fig 2 — Cart Position','Position', [150, 150, 800, 500]);

plot(tout, x_cart, 'g-', 'LineWidth', 2);
hold on;
yline(0, 'r--', 'LineWidth', 1.5);

xlabel('Time (s)', 'FontSize', 13);
ylabel('x (m)', 'FontSize', 13);
title('Cart position x(t) — LQR closed-loop', 'FontSize', 14);
legend('x(t)', 'Reference x = 0', 'FontSize', 11);
grid on;

%% ============================================================
%  FIGURE 3 — Control force F(t)
%% ============================================================

figure('Name', 'Fig 3 — Control Force', ...
       'Position', [200, 200, 800, 500]);

plot(tout, F_out, 'm-', 'LineWidth', 2);
hold on;
yline(0, 'k--', 'LineWidth', 1);

xlabel('Time (s)', 'FontSize', 13);
ylabel('Force F (N)', 'FontSize', 13);
title('Control force F(t) applied to cart', 'FontSize', 14);
grid on;
text(0.02, 0.95, sprintf('Max |F| = %.1f N', max(abs(F_out))), ...
    'Units', 'normalized', 'FontSize', 11, 'BackgroundColor', 'white');

%% ============================================================
%  FIGURE 4 — All states on one figure (the summary figure)
%% ============================================================

figure('Name', 'Fig 4 — Full State Response', ...
       'Position', [250, 50, 1000, 700]);

subplot(2, 2, 1);
plot(tout, theta * 180/pi, 'b', 'LineWidth', 2);
yline(0, 'r--'); grid on;
xlabel('Time (s)'); ylabel('\theta (degrees)');
title('Pendulum angle \theta(t)');

subplot(2, 2, 2);
plot(tout, x_cart, 'g', 'LineWidth', 2);
yline(0, 'r--'); grid on;
xlabel('Time (s)'); ylabel('x (m)');
title('Cart position x(t)');

subplot(2, 2, 3);
plot(tout, theta_dot, 'c', 'LineWidth', 2);
yline(0, 'r--'); grid on;
xlabel('Time (s)'); ylabel('\dot{\theta} (rad/s)');
title('Angular velocity theta_dot');

subplot(2, 2, 4);
plot(tout, F_out, 'm', 'LineWidth', 2);
yline(0, 'k--'); grid on;
xlabel('Time (s)'); ylabel('F (N)');
title('Control force F(t)');

sgtitle('LQR Full State Response — Inverted Pendulum', 'FontSize', 14);

%% ============================================================
%  FIGURE 5 — Pole map (eigenvalues)
%% ============================================================

figure('Name', 'Fig 5 — Pole Map', ...
       'Position', [300, 300, 700, 500]);

% Open-loop poles
ol_poles = eig(A);
% Closed-loop poles
cl_poles = eig(A - B*K_lqr);

hold on;
% Shade unstable region
x_max = max(abs(real([ol_poles; cl_poles]))) * 2;
y_max = max(abs(imag([ol_poles; cl_poles]))) * 2 + 5;
patch([0, x_max, x_max, 0], [-y_max, -y_max, y_max, y_max], ...
    [1, 0.85, 0.85], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Open-loop
plot(real(ol_poles), imag(ol_poles), 'rx', ...
    'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', 'Open-loop poles');
% Closed-loop
plot(real(cl_poles), imag(cl_poles), 'bx', ...
    'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', 'Closed-loop poles (LQR)');

xline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Stability boundary');

xlabel('Real part \sigma', 'FontSize', 13);
ylabel('Imaginary part j\omega', 'FontSize', 13);
title('Pole map: open-loop vs LQR closed-loop', 'FontSize', 14);
legend('Location', 'northwest', 'FontSize', 11);
grid on;
text(x_max*0.5, y_max*0.8, 'UNSTABLE', 'Color', [0.7 0.1 0.1], ...
    'FontSize', 12, 'FontWeight', 'bold');
text(-x_max*0.9, y_max*0.8, 'STABLE', 'Color', [0.1 0.5 0.2], ...
    'FontSize', 12, 'FontWeight', 'bold');

%% ============================================================
%  PRINT PERFORMANCE TABLE TO CONSOLE
%% ============================================================

fprintf('\n========= PERFORMANCE METRICS =========\n');

% Settling time (2% criterion)
thresh = 0.02 * abs(theta(1));
last_outside = find(abs(theta) > thresh, 1, 'last');
if isempty(last_outside)
    t_settle = 0;
elseif last_outside == length(tout)
    t_settle = Inf;
else
    t_settle = tout(last_outside);
end
fprintf('Settling time (2%%):    %.3f s\n', t_settle);

% Overshoot
max_theta = max(abs(theta));
fprintf('Max |theta|:           %.4f rad = %.2f deg\n', ...
    max_theta, max_theta*180/pi);

% Steady state error
fprintf('Final theta:           %.6f rad (%.4f deg)\n', ...
    theta(end), theta(end)*180/pi);
fprintf('Final x:               %.6f m\n', x_cart(end));

% Control effort
fprintf('Max force:             %.2f N\n', max(abs(F_out)));
fprintf('Control energy ∫F²dt:  %.2f N²s\n', trapz(tout, F_out.^2));
