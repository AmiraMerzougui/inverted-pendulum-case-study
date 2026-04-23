%% ============================================================
%  INVERTED PENDULUM — Interactive PID Tuner with Live Plots
%  Run this script in MATLAB. A GUI window opens with sliders.
%  Move any slider → all plots update instantly.
%% ============================================================

function inverted_pendulum_pid_tuner()
% We wrap everything in a function so variables stay local.
% This is standard MATLAB GUI practice.

clear; close all; clc;

%% ---- FIXED SYSTEM PARAMETERS ----
% These are defined once. You can expose them as sliders too if you want.
g  = 9.81;   % gravity (m/s²)
M  = 0.5;    % cart mass (kg)
m  = 0.2;    % pendulum mass (kg)
l  = 0.1;    % pendulum length (m)
lc = l / 2;  % pivot to center of mass
J  = (1/12) * m * l^2;  % moment of inertia of uniform rod

% Determinant (denominator that appears in A and B)
Delta = (m + M) * (m*lc^2 + J) - (m*lc)^2;

% State matrix A (linearized around upright equilibrium)
% State vector: [x, x_dot, theta, theta_dot]
A = [0,  1,                      0,  0;
     0,  0,  -(m^2*g*lc^2)/Delta,   0;
     0,  0,                      0,  1;
     0,  0,   (m+M)*m*g*lc/Delta,   0];

% Input matrix B
B = [0;
     (m*lc^2 + J) / Delta;
     0;
     -m*lc / Delta];

% Output matrix C: we observe x (state 1) and theta (state 3)
C = [1, 0, 0, 0;
     0, 0, 1, 0];
D = [0; 0];

%% ---- INITIAL PID VALUES ----
Kp0 = 100;
Ki0 = 1;
Kd0 = 20;

%% ---- CREATE THE FIGURE WINDOW ----
% 'uifigure' creates a modern MATLAB app-style window
fig = uifigure('Name', 'PID Tuner — Inverted Pendulum', ...
               'Position', [50, 50, 1200, 750]);
fig.Color = [0.97 0.97 0.97];

%% ---- LAYOUT: divide the window into panels ----

% Left panel: sliders (controls)
pLeft = uipanel(fig, 'Position', [10, 10, 260, 730], ...
    'Title', 'PID Gains', 'FontSize', 13, 'FontWeight', 'bold');

% Right side: 3 rows of plots
% We'll use tiledlayout for clean subplots
ax_parent = uipanel(fig, 'Position', [280, 10, 910, 730], ...
    'BorderType', 'none');

%% ---- CREATE SLIDERS ----
% Each slider has: a label, the slider itself, and a value display
% We return handles so we can read them in the callback

% --- Kp sliderz ---
uilabel(pLeft, 'Position', [10, 680, 120, 22], ...
    'Text', 'Kp  (Proportional)', 'FontSize', 11);
sld_Kp = uislider(pLeft, ...
    'Position', [10, 665, 220, 3], ...  % [x, y, width, height]
    'Limits', [0, 500], ...             % range
    'Value', Kp0, ...
    'MajorTicks', [0, 100, 200, 300, 400, 500], ...
    'ValueChangedFcn', @(~,~) updatePlots());
lbl_Kp = uilabel(pLeft, 'Position', [10, 648, 220, 18], ...
    'Text', sprintf('Kp = %.1f', Kp0), ...
    'FontSize', 11, 'FontColor', [0.2 0.4 0.8]);

% --- Ki slider ---
uilabel(pLeft, 'Position', [10, 615, 120, 22], ...
    'Text', 'Ki  (Integral)', 'FontSize', 11);
sld_Ki = uislider(pLeft, ...
    'Position', [10, 600, 220, 3], ...
    'Limits', [0, 50], ...
    'Value', Ki0, ...
    'MajorTicks', [0, 10, 20, 30, 40, 50], ...
    'ValueChangedFcn', @(~,~) updatePlots());
lbl_Ki = uilabel(pLeft, 'Position', [10, 583, 220, 18], ...
    'Text', sprintf('Ki = %.1f', Ki0), ...
    'FontSize', 11, 'FontColor', [0.2 0.6 0.3]);

% --- Kd slider ---
uilabel(pLeft, 'Position', [10, 550, 120, 22], ...
    'Text', 'Kd  (Derivative)', 'FontSize', 11);
sld_Kd = uislider(pLeft, ...
    'Position', [10, 535, 220, 3], ...
    'Limits', [0, 100], ...
    'Value', Kd0, ...
    'MajorTicks', [0, 25, 50, 75, 100], ...
    'ValueChangedFcn', @(~,~) updatePlots());
lbl_Kd = uilabel(pLeft, 'Position', [10, 518, 220, 18], ...
    'Text', sprintf('Kd = %.1f', Kd0), ...
    'FontSize', 11, 'FontColor', [0.8 0.3 0.2]);

%% ---- STABILITY STATUS DISPLAY ----
lbl_status = uilabel(pLeft, ...
    'Position', [10, 470, 230, 40], ...
    'Text', 'Status: checking...', ...
    'FontSize', 13, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.9 0.9 0.9], ...
    'HorizontalAlignment', 'center');

%% ---- EIGENVALUE DISPLAY ----
uilabel(pLeft, 'Position', [10, 440, 230, 22], ...
    'Text', 'Closed-loop eigenvalues:', 'FontSize', 11, 'FontWeight', 'bold');
lbl_eig = uilabel(pLeft, ...
    'Position', [10, 300, 240, 135], ...
    'Text', '...', ...
    'FontSize', 10, 'FontName', 'Courier', ...
    'VerticalAlignment', 'top');

%% ---- PERFORMANCE METRICS ----
uilabel(pLeft, 'Position', [10, 270, 230, 22], ...
    'Text', 'Performance metrics:', 'FontSize', 11, 'FontWeight', 'bold');
lbl_perf = uilabel(pLeft, ...
    'Position', [10, 120, 240, 145], ...
    'Text', '...', ...
    'FontSize', 10, 'FontName', 'Courier', ...
    'VerticalAlignment', 'top');

%% ---- WHAT TO TUNE HINT ----
uilabel(pLeft, 'Position', [10, 10, 240, 105], ...
    'Text', sprintf(['TUNING GUIDE:\n' ...
        'Kp: main stabilizer\n' ...
        '  too low = unstable\n' ...
        '  too high = oscillates\n' ...
        'Kd: adds damping\n' ...
        '  increase to reduce overshoot\n' ...
        'Ki: removes steady error\n' ...
        '  keep small for unstable plants']), ...
    'FontSize', 10, 'FontName', 'Courier', ...
    'VerticalAlignment', 'top', ...
    'FontColor', [0.4 0.4 0.4]);

%% ---- CREATE AXES FOR PLOTS ----
% We create 4 axes manually inside ax_parent

% Axis 1: Theta response (top left, large)
ax1 = axes(ax_parent, 'Position', [0.04, 0.55, 0.58, 0.38]);
title(ax1, 'Pendulum angle \theta(t)  [initial tilt = 0.1 rad]');
xlabel(ax1, 'Time (s)'); ylabel(ax1, '\theta (rad)');
grid(ax1, 'on'); hold(ax1, 'on');

% Axis 2: Cart position (bottom left)
ax2 = axes(ax_parent, 'Position', [0.04, 0.07, 0.58, 0.38]);
title(ax2, 'Cart position x(t)');
xlabel(ax2, 'Time (s)'); ylabel(ax2, 'x (m)');
grid(ax2, 'on'); hold(ax2, 'on');

% Axis 3: Pole map (top right)
ax3 = axes(ax_parent, 'Position', [0.67, 0.55, 0.30, 0.38]);
title(ax3, 'Pole map (complex plane)');
xlabel(ax3, 'Real part'); ylabel(ax3, 'Imaginary part');
grid(ax3, 'on'); hold(ax3, 'on');
xline(ax3, 0, 'k--', 'LineWidth', 1.5);  % imaginary axis (stability boundary)

% Axis 4: Control force (bottom right)
ax4 = axes(ax_parent, 'Position', [0.67, 0.07, 0.30, 0.38]);
title(ax4, 'Control force F(t)');
xlabel(ax4, 'Time (s)'); ylabel(ax4, 'F (N)');
grid(ax4, 'on'); hold(ax4, 'on');

%% ---- SIMULATION SETTINGS ----
t_end = 5;                  % simulate for 5 seconds
t = 0 : 0.005 : t_end;     % time vector, 0.005s step
X0 = [0; 0; 0.1; 0];       % initial state: [x=0, xdot=0, theta=0.1rad, thdot=0]

%% ---- THE UPDATE FUNCTION ----
% This runs every time ANY slider moves.
% It recomputes everything and redraws all plots.

    function updatePlots()
        %% Read current slider values
        Kp = sld_Kp.Value;
        Ki = sld_Ki.Value;
        Kd = sld_Kd.Value;

        %% Update value labels next to sliders
        lbl_Kp.Text = sprintf('Kp = %.1f', Kp);
        lbl_Ki.Text = sprintf('Ki = %.1f', Ki);
        lbl_Kd.Text = sprintf('Kd = %.1f', Kd);

        %% ----- BUILD THE CLOSED-LOOP SYSTEM -----
        %
        % Control law: u = -Kp*theta - Kd*theta_dot - Ki*integral(theta)
        %
        % For a pure state-feedback (PD part):
        %   K_state = [0, 0, Kp, Kd]  → affects rows via B*K
        %   A_cl = A - B*K_state
        %
        % The integral term (Ki) needs an AUGMENTED state.
        % We add a 5th state: x5 = integral of theta.
        % Then the state vector becomes [x, x_dot, theta, theta_dot, int_theta]
        %
        % Augmented system:
        %   A_aug = [A,         zeros(4,1);   % original 4 states + no coupling to integral
        %            [0,0,1,0], 0           ]  % x5_dot = theta = C_theta * X
        %   B_aug = [B; 0]
        %   K_aug = [0, 0, Kp, Kd, Ki]        % full gain vector for 5 states

        % --- Augmented matrices (5x5) ---
        A_aug = [A,          zeros(4,1);   % 4 original rows + zero coupling
                 0, 0, 1, 0, 0        ];   % 5th row: x5_dot = theta (state 3)
        % Explanation: C_theta = [0,0,1,0] picks theta from state vector.
        % So the 5th equation is: x5_dot = [0,0,1,0]*X + 0*x5 = theta

        B_aug = [B; 0];
        % The integral state is not directly driven by force, so last element is 0.

        K_aug = [0, 0, Kp, Kd, Ki];
        % Gain vector: ignore x and x_dot for pure pendulum PID.
        % Kp acts on theta (state 3), Kd on theta_dot (state 4), Ki on integral (state 5).

        % --- Closed-loop matrix ---
        A_cl = A_aug - B_aug * K_aug;
        % Derivation: X_dot = A_aug*X + B_aug*u
        %             u = -K_aug * X  (feedback law)
        %             → X_dot = (A_aug - B_aug*K_aug)*X = A_cl * X

        %% ----- COMPUTE EIGENVALUES -----
        poles = eig(A_cl);
        % eig() returns a vector of 5 complex numbers (one per state).
        % For stability: ALL must have negative real part.

        real_parts = real(poles);
        imag_parts = imag(poles);
        all_stable = all(real_parts < 0);

        %% ----- UPDATE STABILITY STATUS LABEL -----
        if all_stable
            lbl_status.Text = 'STABLE';
            lbl_status.BackgroundColor = [0.7, 0.95, 0.75];  % green
            lbl_status.FontColor = [0.05, 0.35, 0.1];
        else
            n_unstable = sum(real_parts >= 0);
            lbl_status.Text = sprintf('UNSTABLE  (%d positive pole(s))', n_unstable);
            lbl_status.BackgroundColor = [0.98, 0.75, 0.75];  % red
            lbl_status.FontColor = [0.5, 0.05, 0.05];
        end

        %% ----- UPDATE EIGENVALUE DISPLAY -----
        eig_text = '';
        for k = 1:length(poles)
            re = real_parts(k);
            im = imag_parts(k);
            if abs(im) < 1e-6
                eig_text = [eig_text, sprintf('λ%d = %+.3f\n', k, re)];
            elseif im > 0
                eig_text = [eig_text, sprintf('λ%d = %+.3f + %.3fj\n', k, re, abs(im))];
            else
                eig_text = [eig_text, sprintf('λ%d = %+.3f - %.3fj\n', k, re, abs(im))];
            end
        end
        lbl_eig.Text = eig_text;

        %% ----- SIMULATE THE CLOSED-LOOP SYSTEM -----
        % We use lsim (linear simulation) with initial conditions.
        % The augmented system has 5 states, so X0_aug needs 5 elements.

        X0_aug = [X0; 0];  % add initial integral = 0

        C_aug = [1, 0, 0, 0, 0;   % output 1: x (cart position)
                 0, 0, 1, 0, 0];  % output 2: theta (pendulum angle)
        D_aug = [0; 0];

        sys_cl = ss(A_cl, B_aug, C_aug, D_aug);
        % ss() packages the state-space matrices into a MATLAB system object.

        try
            [y, t_out, x_out] = initial(sys_cl, X0_aug, t);
            % initial() simulates response to initial conditions with u=0.
            % y: outputs over time (Nx2 matrix: column1=x, column2=theta)
            % x_out: all states over time (Nx5 matrix)

            % Reconstruct control force at each timestep
            % F = -K_aug * X  for each time step
            F = -(K_aug * x_out')';
            % x_out' is 5×N, K_aug is 1×5, product is 1×N, then transpose to Nx1

            %% ----- COMPUTE PERFORMANCE METRICS -----
            theta = y(:, 2);  % pendulum angle over time
            x_pos = y(:, 1);  % cart position over time

            % Settling time: find when |theta| stays below 2% of initial value
            threshold = 0.02 * abs(X0(3));  % 2% of 0.1 rad
            settled = abs(theta) < threshold;
            % Find first time it stays settled (look from the end backward)
            settle_idx = find(~settled, 1, 'last');
            if isempty(settle_idx)
                t_settle = 0;
            elseif settle_idx == length(t_out)
                t_settle = Inf;  % never settled
            else
                t_settle = t_out(settle_idx);
            end

            % Max overshoot
            max_theta = max(abs(theta));
            overshoot_pct = (max_theta - abs(X0(3))) / abs(X0(3)) * 100;
            overshoot_pct = max(0, overshoot_pct);

            % Max force required
            max_F = max(abs(F));

            perf_text = sprintf(['Settling time: %.2f s\n', ...
                                 'Max |theta|:   %.3f rad\n', ...
                                 '               (%.1f deg)\n', ...
                                 'Overshoot:     %.1f%%\n', ...
                                 'Max |F|:       %.1f N\n', ...
                                 'Final x:       %.3f m'], ...
                t_settle, max_theta, max_theta*180/pi, ...
                overshoot_pct, max_F, x_pos(end));
            lbl_perf.Text = perf_text;

        catch ME
            % If the system is badly unstable, initial() might overflow
            y = zeros(length(t), 2);
            x_out = zeros(length(t), 5);
            F = zeros(length(t), 1);
            lbl_perf.Text = 'Simulation diverged.';
        end

        %% ----- UPDATE ALL PLOTS -----

        % --- Plot 1: Theta ---
        cla(ax1);  % clear previous lines
        hold(ax1, 'on');
        plot(ax1, t_out, y(:,2), 'b-', 'LineWidth', 2, 'DisplayName', '\theta(t)');
        yline(ax1, 0, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Reference (0)');
        yline(ax1, 0.02 * X0(3), 'g:', 'LineWidth', 1, 'DisplayName', '±2% band');
        yline(ax1, -0.02 * X0(3), 'g:', 'LineWidth', 1, 'HandleVisibility', 'off');
        legend(ax1, 'Location', 'northeast');
        title(ax1, sprintf('\\theta(t)   [Kp=%.0f, Ki=%.1f, Kd=%.0f]', Kp, Ki, Kd));
        xlabel(ax1, 'Time (s)'); ylabel(ax1, '\theta (rad)');
        grid(ax1, 'on');
        if all_stable
            ylim(ax1, [-0.15, 0.15]);
        else
            ylim(ax1, 'auto');
        end

        % --- Plot 2: Cart position ---
        cla(ax2);
        hold(ax2, 'on');
        plot(ax2, t_out, y(:,1), 'g-', 'LineWidth', 2);
        yline(ax2, 0, 'k--', 'LineWidth', 1);
        xlabel(ax2, 'Time (s)'); ylabel(ax2, 'x (m)');
        title(ax2, 'Cart position x(t)  [note: no cart position feedback]');
        grid(ax2, 'on');

        % --- Plot 3: Pole map ---
        cla(ax3);
        hold(ax3, 'on');
        xline(ax3, 0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Stability boundary');
        % Shade the unstable region (right half-plane)
        xlim_val = max(10, max(abs(real_parts)) * 1.5);
        ylim_val = max(10, max(abs(imag_parts)) * 1.5);

        patch(ax3, [0, xlim_val, xlim_val, 0], ...
                   [-ylim_val, -ylim_val, ylim_val, ylim_val], ...
                   [1, 0.85, 0.85], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
                   'DisplayName', 'Unstable region');

        % Plot stable poles (green X) and unstable poles (red X)
        for k = 1:length(poles)
            re = real_parts(k);
            im = imag_parts(k);
            if re < 0
                plot(ax3, re, im, 'gx', 'MarkerSize', 14, 'LineWidth', 2.5);
            else
                plot(ax3, re, im, 'rx', 'MarkerSize', 14, 'LineWidth', 2.5);
            end
        end
        xlabel(ax3, 'Real part \sigma');
        ylabel(ax3, 'Imaginary part j\omega');
        title(ax3, 'Pole map (× = green: stable, red: unstable)');
        xlim(ax3, [-xlim_val, xlim_val]);
        ylim(ax3, [-ylim_val, ylim_val]);
        grid(ax3, 'on');

        % --- Plot 4: Control force ---
        cla(ax4);
        hold(ax4, 'on');
        plot(ax4, t_out, F, 'm-', 'LineWidth', 1.5);
        yline(ax4, 0, 'k--', 'LineWidth', 1);
        xlabel(ax4, 'Time (s)'); ylabel(ax4, 'F (N)');
        title(ax4, 'Control force F(t)');
        grid(ax4, 'on');

        drawnow;  % force MATLAB to render immediately (makes it feel real-time)
    end

%% ---- RUN INITIAL PLOT ----
updatePlots();  % call once on startup so plots aren't empty

end  % end of main function