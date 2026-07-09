%% Standalone Wheelbase Isolation Tool (2-DOF Steady-State)
%% Simplified Wheelbase Performance & Points Simulator
clear; clc;

%% 1. Vehicle and Environment Constraints
mass_car = 162.8;               % kg (from your carConfig)
mass_driver = 61;               % kg (from your carConfig)
M = mass_car + mass_driver;     % Total mass (kg)
g = 9.81;                       % m/s^2
h_cg = 11.06 * 0.0254;          % CG height converted to meters (~0.281m)
weight_dist_r = 0.512;          % Rear weight bias percentage

% Tire grip coefficient baseline (tuned to match your tire data scaling)
mu_baseline = 1.55;             

% Track event constants
accel_dist = 75;                % meters
skidpad_radius = 8.5;           % meters (inner radius = 7.625m, + theoretical dist from cone)

% 2026 Michigan Comp Times
accel_winning_time = 4.126;     
skidpad_winning_time = 4.878;   

%% 2. Define Wheelbase Sweep Array
% Sweeping a realistic design window 
wheelbase_sweep_in = linspace(60, 63, 30);

% Pre-allocate output arrays for plotting
sim_accel_time   = zeros(size(wheelbase_sweep_in));
sim_skidpad_time = zeros(size(wheelbase_sweep_in));
sim_accel_points = zeros(size(wheelbase_sweep_in));
sim_skid_points  = zeros(size(wheelbase_sweep_in));
sim_total_points = zeros(size(wheelbase_sweep_in));

%% 3. Closed-Form Physics Simulation Loop
for i = 1:length(wheelbase_sweep_in)
    L = wheelbase_sweep_in(i) * 0.0254; % Convert current wheelbase to meters
    
    %% --- PIPELINE 1: ACCELERATION RUN ---
    % Analytical max acceleration limited by rear traction and load transfer:
    % ax = (mu * g * wd_r) / (1 - (mu * h_cg / L))
    ax_max = (mu_baseline * g * weight_dist_r) / (1 - (mu_baseline * h_cg / L));
    
    % Kinematic time to cover 75m from a standstill (d = 0.5 * a * t^2)
    % Plus a constant 0.35s launch/shifting latency to match real-world data
    sim_accel_time(i) = sqrt((2 * accel_dist) / ax_max) + 0.35;
    
    % Rulebook scoring formula for Acceleration
    t_max_accel = accel_winning_time * 1.5;
    if sim_accel_time(i) > t_max_accel
        sim_accel_points(i) = 4.5;
    else
        sim_accel_points(i) = 95.5 * ((t_max_accel / sim_accel_time(i)) - 1.0) / ...
                                     ((t_max_accel / accel_winning_time) - 1.0) + 4.5;
    end
    
    %% --- PIPELINE 2: SKIDPAD RUN ---
    % Kinematic steer angle required to hold the circle: delta = L / R
    delta_rad = L / skidpad_radius;
    
    % Because the front wheels must turn inward to make the tight turn, 
    % their lateral force vector tilts backward, reducing pure lateral efficiency:
    ay_max = (mu_baseline * g) * cos(delta_rad);
    
    % Calculate the maximum stable velocity on the skidpad arc (v = sqrt(ay * R))
    v_max_skid = sqrt(ay_max * skidpad_radius);
    
    % Time to complete a single full lap circle (Circumference = 2 * pi * R)
    sim_skidpad_time(i) = (2 * pi * skidpad_radius) / v_max_skid;
    
    % Rulebook scoring formula for Skidpad
    t_max_skid = 1.25 * skidpad_winning_time;
    if sim_skidpad_time(i) > t_max_skid
        sim_skid_points(i) = 3.5;
    else
        sim_skid_points(i) = 71.5 * ((t_max_skid / sim_skidpad_time(i))^2.0 - 1.0) / ...
                                   ((t_max_skid / skidpad_winning_time)^2.0 - 1.0) + 3.5;
    end
    
    %% --- TOTAL PERFORMANCE ---
    sim_total_points(i) = sim_accel_points(i) + sim_skid_points(i);
end

%% 4. Data Visualization Setup
figure('Color', [1 1 1], 'Position', [100, 100, 1000, 450]);

% Left Chart: Simulated Lap Times
subplot(1,2,1);
yyaxis left
plot(wheelbase_sweep_in, sim_accel_time, 'r', 'LineWidth', 2.5);
ylabel('Acceleration Time (seconds)', 'Color', 'r', 'FontWeight', 'bold');
ax = gca; ax.YColor = 'r';

yyaxis right
plot(wheelbase_sweep_in, sim_skidpad_time, 'b', 'LineWidth', 2.5);
ylabel('Skidpad Time (seconds)', 'Color', 'b', 'FontWeight', 'bold');
ax = gca; ax.YColor = 'b';

grid on; box on;
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold');
title('Wheelbase Impact on Simulated Times', 'FontSize', 12);

% Right Chart: Resulting Points Distribution
subplot(1,2,2); hold on;
plot(wheelbase_sweep_in, sim_accel_points, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Accel Points (Max 100)');
plot(wheelbase_sweep_in, sim_skid_points, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Skidpad Points (Max 75)');
plot(wheelbase_sweep_in, sim_total_points, 'k', 'LineWidth', 3, 'DisplayName', 'Total Points Combined');

grid on; box on;
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold');
ylabel('Points Awarded', 'FontWeight', 'bold');
title('The Competing Points Trade-off', 'FontSize', 12);
legend('Location', 'best');

% Print optimal configuration to command window
[max_pts, idx] = max(sim_total_points);
fprintf('\n====================================================\n');
fprintf('  SIMULATION ANALYSIS COMPLETE\n');
fprintf('====================================================\n');
fprintf('Optimal Wheelbase for points: %.2f inches\n', wheelbase_sweep_in(idx));
fprintf('Predicted Accel Time:         %.3f seconds\n', sim_accel_time(idx));
fprintf('Predicted Skidpad Time:       %.3f seconds\n', sim_skidpad_time(idx));
fprintf('Maximum Combined Score:       %.2f points\n', max_pts);
fprintf('====================================================\n');