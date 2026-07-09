% Wheelbase_Accel_Sweep.m
% Isolates the impact of wheelbase length on longitudinal load transfer,
% rear tire grip, and 75m Acceleration Event times.
%
% HOW TO USE:
% 1) Place in your main Lapsim2 directory.
% 2) Run directly. It extracts accelCar from carConfig() and simulates the 75m sprint.

clear; clc; setup_paths;

%% 1. SWEEP CONFIGURATION
fprintf('Step 1: Extracting baseline Accel car configuration...\n');
carCell = carConfig();
% Column 2 in carCell contains the specialized acceleration event setup 
% (e.g., lighter driver weight, low-drag aero trim)
base_accel_car = carCell{1, 2}; 

% Define Wheelbase Sweep Range (inches)
% Sweeping a broader range to make load transfer delta obvious
wheelbases_in = linspace(60, 62.5, 6); 
num_wb = length(wheelbases_in);

% Storage structure for results
results = struct();

%% 2. RUN LONGITUDINAL ACCELERATION SWEEP
fprintf('Step 2: Simulating 75m Acceleration Event across wheelbases...\n');

for w = 1:num_wb
    wb_in = wheelbases_in(w);
    wb_m = wb_in * 0.0254;
    weight_dist = 0.512; % Keep static weight distribution percentage constant
    
    % Clone baseline accel car and update geometry
    car_mod = base_accel_car;
    car_mod.W_b = wb_m;
    car_mod.l_f = wb_m * weight_dist;
    car_mod.l_r = wb_m * (1 - weight_dist);
    
    % 1. Solve for max longitudinal acceleration capability vs. velocity
    % This optimizer finds the tire grip / wheelspin limit at low speeds 
    % and engine power limit at high speeds.
    [~, long_vel_interp, long_accel_interp] = long_accel_sweep(car_mod);
    
    % 2. Simulate the 0.3m (2.19 ft) staging rollout
    % As defined in Events2.Accel(), car starts slightly behind timing beam
    [~, launch_vel, ~, ~] = straight(0, 2.19, long_vel_interp, long_accel_interp, ...
                                     car_mod.max_vel, car_mod);
    
    % 3. Simulate the official 75m sprint from the timing beam
    [time_vec, end_vel, long_accel_vec, long_vel_vec] = straight(launch_vel, 75, ...
                                     long_vel_interp, long_accel_interp, ...
                                     car_mod.max_vel, car_mod);
    
    % 4. Calculate Longitudinal Load Transfer (Delta Fz) in Newtons
    % formula: Delta_Fz = (Mass * ax * h_g) / Wheelbase
    delta_Fz = (car_mod.M * long_accel_vec * car_mod.h_g) / wb_m;
    
    % 5. Approximate Rear Axle Total Normal Load (Fz_rear) during run
    static_Fz_rear = car_mod.M * 9.81 * car_mod.l_f / wb_m;
    total_Fz_rear  = static_Fz_rear + delta_Fz;
    
    % Store data
    results(w).wb_in       = wb_in;
    results(w).time_75m    = time_vec(end);
    results(w).end_vel     = end_vel;
    results(w).time_vec    = time_vec;
    results(w).vel_vec     = long_vel_vec;
    results(w).accel_vec   = long_accel_vec / 9.81; % Convert m/s^2 to Gs
    results(w).delta_Fz    = delta_Fz;
    results(w).total_Fz_r  = total_Fz_rear;
    
    fprintf('  -> Wheelbase: %.1f in | 75m Time: %.3f s | Trap Speed: %.1f m/s\n', ...
            wb_in, time_vec(end), end_vel);
end

fprintf('Sweep Complete! Generating Dark-Mode Visualizations...\n');

%% 3. DARK-MODE FRIENDLY PLOTTING
colors = cool(num_wb); % Bright cyan-to-magenta colormap

% --- Figure 1: Overall Performance Summary ---
fig1 = figure('Name', '75m Accel Performance Summary', 'NumberTitle', 'off', 'Position', [150, 150, 800, 400]);

subplot(1, 2, 1); hold on; grid on;
times = [results.time_75m];
plot(wheelbases_in, times, '-o', 'LineWidth', 2.5, 'Color', [0.0, 0.9, 0.9], ...
     'MarkerFaceColor', [1.0, 0.3, 0.6], 'MarkerSize', 8);
title('75m Sprint Time vs. Wheelbase');
xlabel('Wheelbase (in)'); ylabel('Time (seconds)');
% Annotate delta time
time_diff = (max(times) - min(times)) * 1000;
subtitle(sprintf('Max Delta: %.1f ms across sweep', time_diff));

subplot(1, 2, 2); hold on; grid on;
traps = [results.end_vel] * 2.23694; % Convert m/s to mph for readability
plot(wheelbases_in, traps, '-s', 'LineWidth', 2.5, 'Color', [0.3, 1.0, 0.3], ...
     'MarkerFaceColor', [1.0, 0.85, 0.0], 'MarkerSize', 8);
title('75m Finish Line Trap Speed');
xlabel('Wheelbase (in)'); ylabel('Trap Speed (mph)');

% Apply dark-mode styling
set(findall(fig1, 'type', 'axes'), 'Color', 'none', 'XColor', [0.8 0.8 0.8], ...
    'YColor', [0.8 0.8 0.8], 'GridColor', [0.4 0.4 0.4], 'GridAlpha', 0.5, 'LineWidth', 1.1);


% --- Figure 2: Launch Mechanics & Load Transfer ---
fig2 = figure('Name', 'Launch Dynamics & Load Transfer', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 600]);

% Plot 1: Acceleration Profile vs. Velocity
subplot(2, 2, 1); hold on; grid on;
for w = 1:num_wb
    plot(results(w).vel_vec, results(w).accel_vec, 'LineWidth', 2, ...
         'Color', colors(w, :), 'DisplayName', sprintf('%.1f in', wheelbases_in(w)));
end
title('Longitudinal Gs vs. Velocity');
xlabel('Velocity (m/s)'); ylabel('Acceleration (g)');
legend('Location', 'northeast', 'TextColor', [0.9 0.9 0.9]);
xlim([0, max(results(1).vel_vec)]);

% Plot 2: Velocity Profile vs. Time
subplot(2, 2, 2); hold on; grid on;
for w = 1:num_wb
    plot(results(w).time_vec, results(w).vel_vec, 'LineWidth', 2, ...
         'Color', colors(w, :), 'DisplayName', sprintf('%.1f in', wheelbases_in(w)));
end
title('Velocity vs. Elapsed Time');
xlabel('Time (s)'); ylabel('Velocity (m/s)');

% Plot 3: Dynamic Load Transfer (Delta Fz) vs. Velocity
subplot(2, 2, 3); hold on; grid on;
for w = 1:num_wb
    % Convert Newtons to lbf (divided by 4.448) for intuitive tire scale
    plot(results(w).vel_vec, results(w).delta_Fz / 4.44822, 'LineWidth', 2, ...
         'Color', colors(w, :), 'DisplayName', sprintf('%.1f in', wheelbases_in(w)));
end
title('Dynamic Load Transfer (\DeltaF_z) vs. Velocity');
xlabel('Velocity (m/s)'); ylabel('Weight Shifted Rearward (lbf)');

% Plot 4: Total Rear Axle Normal Load vs. Velocity
subplot(2, 2, 4); hold on; grid on;
for w = 1:num_wb
    plot(results(w).vel_vec, results(w).total_Fz_r / 4.44822, 'LineWidth', 2, ...
         'Color', colors(w, :), 'DisplayName', sprintf('%.1f in', wheelbases_in(w)));
end
title('Total Rear Axle Normal Load (F_{z,rear})');
xlabel('Velocity (m/s)'); ylabel('Total Rear Load (lbf)');

% Apply dark-mode styling
set(findall(fig2, 'type', 'axes'), 'Color', 'none', 'XColor', [0.8 0.8 0.8], ...
    'YColor', [0.8 0.8 0.8], 'GridColor', [0.4 0.4 0.4], 'GridAlpha', 0.5, 'LineWidth', 1.1);





%{
%%%%%%%%%%%%%%%  Understeer gradient, sample track %%%%%%%%%%%%%%%%

%% Track Position Understeer Gradient Comparison
clear; clc;
setup_paths;

%% 1. Load Baseline Configuration Template
disp('Loading vehicle configuration...');
carCell = carConfig(); 
base_car = carCell{1,1}; 

%% 2. Define Track Profile (Radius and Velocity Profiles)
% Let's simulate a track section: Hairpin -> Short Straight -> High-Speed Sweeper
% You can replace this with your actual logged data or lap simulation arrays!
track_distance = 0:10:200; % Distance points along the track (meters)

% Corner Radius Profile at each distance point (in meters)
% (Note: Large number like 1000 represents a straight line)
radius_profile = [25, 25, 25, 40, 1000, 1000, 1000, 60, 60, 60, 60, 60, 45, 45, 45, 45, 1000, 1000, 1000, 1000, 1000];

% Car Velocity Profile at each distance point (in m/s)
vel_profile    = [10, 11, 12, 16,  22,   24,   23,   18, 18, 19, 19, 18, 14, 13, 14, 15,   20,   22,   24,   25,   25];

%% 3. Run Sweep Across Wheelbases and Map to Track
wheelbase_sweep_in = linspace(60, 62.5, 6);
numCars = length(wheelbase_sweep_in);
numPoints = length(track_distance);

% Pre-allocate matrix to store K at each track point for each car
% Rows = Wheelbase setups, Columns = Track positions
K_track_matrix = zeros(numCars, numPoints);

disp('Analyzing understeer gradient throughout track layout...');
for i = 1:numCars
    car = base_car;
    L = wheelbase_sweep_in(i) * 0.0254;
    car.W_b = L;
    car.l_f = L * 0.512;       
    car.l_r = L * (1 - 0.512); 
    
    fprintf('Processing Car %d of %d (Wheelbase: %.1f in)...\n', i, numCars, wheelbase_sweep_in(i));
    
    for pt = 1:numPoints
        R_local = radius_profile(pt);
        V_local = vel_profile(pt);
        
        % If it's a pure straight line, K is theoretically 0 (Neutral)
        if R_local > 500
            K_track_matrix(i, pt) = 0;
            continue;
        end
        
        % 1. Run the team's solver for this specific corner radius
        [~, K_sweep, ~, ~, ~, ~, ~, ~, ~, vel_sweep, ~, ~] = UndersteerGradient(car, R_local);
        
        % 2. Match the K array length to the velocity array length
        % K is calculated via diff(), so it has 1 less element than vel_sweep.
        vel_sweep_trimmed = vel_sweep(2:end);
        
        % 3. Look up what K is at our track's current velocity using interpolation
        % 'linear','extrap' ensures it handles edge boundary conditions safely.
        K_track_matrix(i, pt) = interp1(vel_sweep_trimmed, K_sweep, V_local, 'linear', 'extrap');
    end
end

disp('Mapping complete. Rendering Track Performance Figures...');

%% 4. Plotting Track-Space Metrics
figure('Name', 'Track Understeer Gradient Comparison', 'Position', [150, 150, 1000, 600]);

legend_labels = cellstr(num2str(wheelbase_sweep_in', 'Wheelbase = %.1f in'));

% Plot the K values across the track layout
for i = 1:numCars
    plot(track_distance, K_track_matrix(i, :), 'LineWidth', 2);
    hold on;
end


grid on; box on;
xlim([track_distance(1) track_distance(end)]);
xlabel('Track Position (meters)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Understeer Gradient K (deg/g)', 'FontSize', 12, 'FontWeight', 'bold');
title('Understeer Gradient (K) Profile Throughout Track Lap', 'FontSize', 14, 'FontWeight', 'bold');
legend(legend_labels, 'Location', 'best');

% Annotate Track Conditions on the plot background
ax = gca;
y_lims = ax.YLim;
% Highlight the tight low-speed corner zone
patch([0 30 30 0], [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], 'g', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
text(5, y_lims(2)*0.8, 'Low-Speed Corner', 'FontSize', 9, 'FontAngle', 'italic');

% Highlight high-speed sweeper zone
patch([70 110 110 70], [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
text(75, y_lims(2)*0.8, 'High-Speed Sweeper', 'FontSize', 9, 'FontAngle', 'italic');

%}

