%% Master RCVD Bicycle Model: Multi-Corner & Stability Suite
clear; clc;
setup_paths;

%% 1. Initialize Baseline Vehicle & Pull Dynamic Parameters
disp('Step 1: Running carConfig to pull dynamic parameters...');
carCell = carConfig(); 
car = carCell{1,1};       
accelCar = carCell{1,2};  
numWorkers = 16; 

M = car.M;                
g = car.g;
weight_dist_r = 0.512; 
weight_dist_f = 1 - weight_dist_r;
track_width = car.t_f;    

rho = 1.162;              
ClA = car.aero.cla;       
aero_dist_f = 0.539; 
aero_dist_r = 1 - aero_dist_f;

%% 2. Run Run-Once Track Solver & Extract 3 Low / 3 High Speed Corners
disp('Step 2: Processing track solver to cluster velocity conditions...');
paramsArr = gg2(car, numWorkers); 
car = makeGG(paramsArr, car);

comp = Events2(car, accelCar);
comp.calcTimes(); 

load('michigantrack2024.mat'); 
V_profile  = comp.autocross.long_vel;
ay_profile = comp.autocross.lat_accel;

% --- Extract 3 Low-Speed Hairpins (Highest Curvature Peaks) ---
[~, locs_low] = findpeaks(abs(curvature), 'SortStr', 'descend', 'MinPeakDistance', 50);
idx_low_3 = locs_low(1:3);
R_low = 1 ./ abs(curvature(idx_low_3));
V_low = V_profile(idx_low_3);
ay_low = abs(ay_profile(idx_low_3));

% --- Extract 3 High-Speed Sweepers (Moderate Curvature, Highest Speeds) ---
sweep_candidates = find(abs(curvature) > 0.015 & abs(curvature) < 0.035);
[~, sweep_locs] = findpeaks(V_profile(sweep_candidates), 'SortStr', 'descend', 'MinPeakDistance', 30);
idx_high_3 = sweep_candidates(sweep_locs(1:3));
R_high = 1 ./ abs(curvature(idx_high_3));
V_high = V_profile(idx_high_3);
ay_high = abs(ay_profile(idx_high_3));

fprintf('\n====================================================\n');
fprintf('  MULTI-CORNER TRACK CLUSTERS EXTRACTED\n');
fprintf('====================================================\n');
for c = 1:3
    fprintf('Low  %d: R = %5.1f m | V = %5.1f m/s (%4.1f mph) | Ay = %4.2f G\n', c, R_low(c), V_low(c), V_low(c)*2.237, ay_low(c)/g);
end
fprintf('----------------------------------------------------\n');
for c = 1:3
    fprintf('High %d: R = %5.1f m | V = %5.1f m/s (%4.1f mph) | Ay = %4.2f G\n', c, R_high(c), V_high(c), V_high(c)*2.237, ay_high(c)/g);
end
fprintf('====================================================\n\n');

%% 3. Dynamic Empirical Tire Load Sensitivity (Pacejka Sourced)
disp('Step 3: Calculating empirical Pacejka load sensitivity slopes...');
total_weight = M * g;
Fz_f_stat = (total_weight * weight_dist_f) / 2;
Fz_r_stat = (total_weight * weight_dist_r) / 2;
alpha_deg = 0.1; 
alpha_rad = alpha_deg * (pi / 180);

% Query tire model at static weight and at +500N aero/transfer load
Fy_f1 = car.tire.F_y(alpha_deg, 0, Fz_f_stat, 0);
Fy_f2 = car.tire.F_y(alpha_deg, 0, Fz_f_stat + 500, 0);
Fy_r1 = car.tire.F_y(alpha_deg, 0, Fz_r_stat, 0);
Fy_r2 = car.tire.F_y(alpha_deg, 0, Fz_r_stat + 500, 0);

Cf_base = abs(2 * (Fy_f1 / alpha_rad));
Cr_base = abs(2 * (Fy_r1 / alpha_rad));
Cf_loaded = abs(2 * (Fy_f2 / alpha_rad));
Cr_loaded = abs(2 * (Fy_r2 / alpha_rad));

% True empirical stiffness drop per Newton of added axle load (N/rad per N)
dCf_dFz = (Cf_base - Cf_loaded) / 1000; 
dCr_dFz = (Cr_base - Cr_loaded) / 1000;

fprintf('Empirical Axle Stiffnesses: Cf = %.1f N/rad | Cr = %.1f N/rad\n', Cf_base, Cr_base);
fprintf('Empirical Load Sensitivity: dCf/dFz = %.4f | dCr/dFz = %.4f\n\n', dCf_dFz, dCr_dFz);

%% 4. Wheelbase Parameter Sweep Loop across Multi-Corner Cluster
disp('Step 4: Sweeping geometric parameters across corner clusters...');
wheelbase_sweep_in = linspace(60, 62.5, 6); 
numSteps = length(wheelbase_sweep_in);

% Pre-allocate storage arrays [Steps x 3 Corners]
steer_low  = zeros(numSteps, 3);
steer_high = zeros(numSteps, 3);
SI_low     = zeros(numSteps, 3);
SI_high    = zeros(numSteps, 3);

% Pre-allocate 10-point Metric Velocity Sweep for RCVD Fig 5.52 [Steps x Velocities]
vel_sweep_mps = linspace(5, 35, 10); % 10 points from 5 m/s (~11 mph) to 35 m/s (~78 mph)
sens_matrix   = zeros(numSteps, length(vel_sweep_mps));

for i = 1:numSteps
    L = wheelbase_sweep_in(i) * 0.0254; % meters
    l_f = L * weight_dist_r;            
    l_r = L * weight_dist_f;            
    
    % --- Process 3 Low-Speed Corners ---
    for c = 1:3
        Fy_f = (M * ay_low(c) * l_r) / L;
        Fy_r = (M * ay_low(c) * l_f) / L;
        Fz_f = ((M * g * l_r) / L) + (0.5 * rho * (V_low(c)^2) * ClA * aero_dist_f);
        Fz_r = ((M * g * l_f) / L) + (0.5 * rho * (V_low(c)^2) * ClA * aero_dist_r);
        
        Cf = Cf_base - (dCf_dFz * (Fz_f - (total_weight*weight_dist_f)));
        Cr = Cr_base - (dCr_dFz * (Fz_r - (total_weight*weight_dist_r)));
        
        balance = (Fy_f / Cf) - (Fy_r / Cr);
        steer_low(i, c) = rad2deg(L / R_low(c)) + rad2deg(balance);
        
        % RCVD Eq 5.68 Stability Index (SI)
        SM = (l_f * Cf - l_r * Cr) / (L * (Cf + Cr));
        C0 = (Cf * Cr * L^2) / (2 * M * (Cf + Cr));
        SI_low(i, c) = -SM + (C0 * (R_low(c) * g) / (V_low(c)^2)) / 1000; 
    end
    
    % --- Process 3 High-Speed Corners ---
    for c = 1:3
        Fy_f = (M * ay_high(c) * l_r) / L;
        Fy_r = (M * ay_high(c) * l_f) / L;
        Fz_f = ((M * g * l_r) / L) + (0.5 * rho * (V_high(c)^2) * ClA * aero_dist_f);
        Fz_r = ((M * g * l_f) / L) + (0.5 * rho * (V_high(c)^2) * ClA * aero_dist_r);
        
        Cf = Cf_base - (dCf_dFz * (Fz_f - (total_weight*weight_dist_f)));
        Cr = Cr_base - (dCr_dFz * (Fz_r - (total_weight*weight_dist_r)));
        
        balance = (Fy_f / Cf) - (Fy_r / Cr);
        steer_high(i, c) = rad2deg(L / R_high(c)) + rad2deg(balance);
        
        SM = (l_f * Cf - l_r * Cr) / (L * (Cf + Cr));
        C0 = (Cf * Cr * L^2) / (2 * M * (Cf + Cr));
        SI_high(i, c) = -SM + (C0 * (R_high(c) * g) / (V_high(c)^2)) / 1000;
    end
    
    % --- Calculate RCVD Fig 5.52 Steering Sensitivity across 10 Speeds in m/s ---
    for v = 1:length(vel_sweep_mps)
        V_val = vel_sweep_mps(v);
        
        % Dynamic normal loads with aero downforce
        Fz_f_val = ((M * g * l_r) / L) + (0.5 * rho * (V_val^2) * ClA * aero_dist_f);
        Fz_r_val = ((M * g * l_f) / L) + (0.5 * rho * (V_val^2) * ClA * aero_dist_r);
        
        % Empirical Pacejka load-sensitive stiffnesses
        Cf_val = Cf_base - (dCf_dFz * (Fz_f_val - (total_weight*weight_dist_f)));
        Cr_val = Cr_base - (dCr_dFz * (Fz_r_val - (total_weight*weight_dist_r)));
        
        % Understeer Gradient & Ackermann Gradient in deg/g (Pure Metric)
        UG_deg_per_g = rad2deg((total_weight * weight_dist_f) / Cf_val - (total_weight * weight_dist_r) / Cr_val);
        ack_grad_deg_per_g = rad2deg(L * g / (V_val^2));
        
        % Steering sensitivity dA_y / dDelta in g per degree
        sens_matrix(i, v) = 1 / (UG_deg_per_g + ack_grad_deg_per_g);
    end
end

%% 5. Plot RCVD Multi-Corner Diagnostics across Separate Tabs
disp('Step 5: Rendering dark-mode diagnostic tabs...');
colors = ['b', 'c', 'm']; % Distinct visibility against dark backgrounds

% =========================================================================
% FIGURE / TAB 1: LOW-SPEED STEERING DEMAND (3 Corners)
% =========================================================================
figure('Name', 'Low-Speed Steering', 'Position', [100, 100, 750, 500]);
hold on;
for c = 1:3
    plot(wheelbase_sweep_in, steer_low(:, c), [colors(c) '-o'], 'LineWidth', 2.5, ...
        'DisplayName', sprintf('Corner %d (R=%.1fm, V=%.1fm/s)', c, R_low(c), V_low(c)));
    yline(rad2deg((62*0.0254)/R_low(c)), [colors(c) '--'], 'LineWidth', 1, 'HandleVisibility', 'off');
end
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Total Steering Angle (degrees)', 'FontWeight', 'bold', 'FontSize', 11);
title('LOW-SPEED CLUSTER: Steering Demand across 3 Hairpins', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% =========================================================================
% FIGURE / TAB 2: HIGH-SPEED STEERING DEMAND (3 Corners)
% =========================================================================
figure('Name', 'High-Speed Steering', 'Position', [150, 150, 750, 500]);
hold on;
for c = 1:3
    plot(wheelbase_sweep_in, steer_high(:, c), [colors(c) '-o'], 'LineWidth', 2.5, ...
        'DisplayName', sprintf('Sweeper %d (R=%.1fm, V=%.1fm/s)', c, R_high(c), V_high(c)));
    yline(rad2deg((62*0.0254)/R_high(c)), [colors(c) '--'], 'LineWidth', 1, 'HandleVisibility', 'off');
end
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Total Steering Angle (degrees)', 'FontWeight', 'bold', 'FontSize', 11);
title('HIGH-SPEED CLUSTER: Steering Demand across 3 Sweepers', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% =========================================================================
% FIGURE / TAB 3: RCVD STABILITY INDEX (SI)
% =========================================================================
figure('Name', 'RCVD Stability Index', 'Position', [200, 200, 750, 500]);
hold on;
for c = 1:3
    plot(wheelbase_sweep_in, SI_low(:, c), [colors(c) '-o'], 'LineWidth', 2, ...
        'DisplayName', sprintf('Low-Speed %d (R=%.1fm)', c, R_low(c)));
    plot(wheelbase_sweep_in, SI_high(:, c), [colors(c) '--s'], 'LineWidth', 2, ...
        'DisplayName', sprintf('High-Speed %d (R=%.1fm)', c, R_high(c)));
end
yline(0, 'g--', 'LineWidth', 2, 'DisplayName', 'Neutral Steer / Critical Speed Line');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Stability Index SI = \partial C_N / \partial A_y', 'FontWeight', 'bold', 'FontSize', 11);
title('RCVD Eq 5.68: Directional Stability Index vs Wheelbase', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

yl3 = ylim; ymid3 = mean(yl3);
text(wheelbase_sweep_in(2), ymid3 + (yl3(2)-ymid3)*0.4, '\leftarrow Unstable Directional Zone (+)', 'FontWeight', 'bold', 'Color', [0.7 0.2 0.2]);
text(wheelbase_sweep_in(2), ymid3 - (ymid3-yl3(1))*0.4, '\leftarrow Stable Weathercock Zone (-)', 'FontWeight', 'bold', 'Color', [0.2 0.5 0.2]);

% =========================================================================
% FIGURE / TAB 4: RCVD FIG 5.52 STEERING SENSITIVITY (All 6 Wheelbases)
% =========================================================================
figure('Name', 'Steering Sensitivity', 'Position', [250, 250, 750, 500]);
hold on;
colors_sens = cool(numSteps); % Cyan-to-Magenta colormap for dark mode visibility

for i = 1:numSteps
    % Highlight current 62in baseline (Index 5) with a distinctive dashed style
    if abs(wheelbase_sweep_in(i) - 62) < 0.1
        plot(vel_sweep_mps, sens_matrix(i, :), '--o', 'Color', colors_sens(i, :), 'LineWidth', 3, ...
            'MarkerSize', 6, 'MarkerFaceColor', colors_sens(i, :), ...
            'DisplayName', sprintf('%.1f in Wheelbase (Current Baseline)', wheelbase_sweep_in(i)));
    else
        plot(vel_sweep_mps, sens_matrix(i, :), '-o', 'Color', colors_sens(i, :), 'LineWidth', 2, ...
            'MarkerSize', 5, 'MarkerFaceColor', colors_sens(i, :), ...
            'DisplayName', sprintf('%.1f in Wheelbase', wheelbase_sweep_in(i)));
    end
end

grid on; box on;
xlim([vel_sweep_mps(1) vel_sweep_mps(end)]);
xlabel('Velocity (m/s)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Steering Sensitivity dA_y / d\delta (g per deg)', 'FontWeight', 'bold', 'FontSize', 11);
title('RCVD Fig 5.52: Steering Sensitivity across 10 Speeds & All 6 Wheelbases', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);

yl4 = ylim;
text(vel_sweep_mps(4), yl4(2)*0.85, 'Shorter wheelbases sit on higher, steeper sensitivity curves \rightarrow', ...
    'FontWeight', 'bold', 'Color', colors_sens(1, :), 'HorizontalAlignment', 'left');

disp('All 4 RCVD diagnostic tabs rendered successfully.');



%{
%% Master Wheelbase Sector & Balance Analysis Tool (Isolated Snapshot)
clear; clc;
%% 1. Initialize Environment & Load Configuration Object
setup_paths; 
disp('Step 1: Running carConfig to pull dynamic parameters...');
carCell = carConfig(); 
car = carCell{1,1};       % Extract baseline car object
accelCar = carCell{1,2};  % Extract baseline acceleration car object
% Set parallel workers for the single baseline optimization run
numWorkers = 16; 
% Dynamic Parameter Extraction from config setup
M = car.M;                % Total mass (car + driver) automatically computed
g = car.g;
weight_dist_r = 0.512; 
weight_dist_f = 1 - weight_dist_r;
track_width = car.t_f;    % Pulls track width straight from car Class parameters
% Aero Parameter Extraction
rho = 1.162;              % Air density (kg/m^3)
ClA = car.aero.cla;       % Pulls total lift area dynamically
aero_dist_f = 0.539; 
aero_dist_r = 1 - aero_dist_f;
%% 2. Run RUN ONCE Baseline Lapsim Track Solver for Sector Extraction
disp('Step 2: Processing baseline track solver to extract velocity conditions...');
% Generate performance lookup tables for the baseline car once
paramsArr = gg2(car, numWorkers); 
car = makeGG(paramsArr, car);
% Run track calculation loop once to get the baseline velocity profile
comp = Events2(car, accelCar);
comp.calcTimes(); 
% Pull geometry data and solver results
load('michigantrack2024.mat'); % Loads 'arclength' and 'curvature'
V_profile  = comp.autocross.long_vel;
ay_profile = comp.autocross.lat_accel;
% Automated Corner Extraction Logic (Frozen for the upcoming sweep)
% Sector 1: Low-Speed Hairpin (Peak Curvature Spike)
[max_curve, idx_low] = max(abs(curvature));
R1 = 1 / max_curve;                  
V1 = V_profile(idx_low);             
ay1 = abs(ay_profile(idx_low));      
% Sector 2: High-Speed Sweeper (Moderate cornering radius, maximum speed profile)
corner_indices = find(abs(curvature) > 0.015 & abs(curvature) < 0.04);
[~, max_v_idx] = max(V_profile(corner_indices));
idx_high = corner_indices(max_v_idx);
R2 = 1 / abs(curvature(idx_high));   
V2 = V_profile(idx_high);            
ay2 = abs(ay_profile(idx_high));     
%% 3. Dynamic Tire Stiffness Calculation (Axle Baselines)
disp('Step 3: Querying Tire2 class model for linear axle stiffness slips...');
total_weight = M * g;
Fz_front_tire = (total_weight * weight_dist_f) / 2;
Fz_rear_tire  = (total_weight * weight_dist_r) / 2;
alpha_deg = 0.1; 
alpha_rad = alpha_deg * (pi / 180);
% Direct lookup using your active tire object method
Fy_front = car.tire.F_y(alpha_deg, 0, Fz_front_tire, 0);
Fy_rear  = car.tire.F_y(alpha_deg, 0, Fz_rear_tire, 0);
Cf_base = abs(2 * (Fy_front / alpha_rad));
Cr_base = abs(2 * (Fy_rear / alpha_rad));
% Display extracted constants to command window
fprintf('\n====================================================\n');
fprintf('  DYNAMIC PARAMETER ANALYSIS COMPLETE (FROZEN TARGETS)\n');
fprintf('====================================================\n');
fprintf('Extracted Baseline Axle Stiffnesses:\n');
fprintf('  Cf_base: %7.2f N/rad | Cr_base: %7.2f N/rad\n\n', Cf_base, Cr_base);
fprintf('Sector 1 Parameters (Low-Speed Hairpin):\n');
fprintf('  Radius: %5.2f m | Frozen Speed: %5.2f m/s | Lat Accel: %4.2f G\n\n', R1, V1, ay1/g);
fprintf('Sector 2 Parameters (High-Speed Sweeper):\n');
fprintf('  Radius: %5.2f m | Frozen Speed: %5.2f m/s | Lat Accel: %4.2f G\n', R2, V2, ay2/g);
fprintf('====================================================\n\n');
%% 4. Wheelbase Parameter Sweep Loop (Fixed Speed and Radius)
disp('Step 4: Sweeping geometric parameters across frozen track benchmarks...');
wheelbase_sweep_in = linspace(60, 63, 14); % 14 steps matching your design grid
balance_sector1 = zeros(size(wheelbase_sweep_in));
balance_sector2 = zeros(size(wheelbase_sweep_in));
steer_total_sector1 = zeros(size(wheelbase_sweep_in));
steer_total_sector2 = zeros(size(wheelbase_sweep_in));
load_sensitivity = 0.005; % % Drop in stiffness per Newton of added load
for i = 1:length(wheelbase_sweep_in)
    L = wheelbase_sweep_in(i) * 0.0254; % Convert target to meters
    l_f = L * weight_dist_r;            
    l_r = L * weight_dist_f;            
    
    %% --- SECTOR 1 BALANCE ---
    % Evaluated using the identical frozen ay1 and V1 values across all wheelbases
    Fy_f1 = (M * ay1 * l_r) / L;
    Fy_r1 = (M * ay1 * l_f) / L;
    
    % Axle loads factoring speed-dependent aero downforce profile
    Fz_f1 = ((M * g * l_r) / L) + (0.5 * rho * (V1^2) * ClA * aero_dist_f);
    Fz_r1 = ((M * g * l_f) / L) + (0.5 * rho * (V1^2) * ClA * aero_dist_r);
    
    Cf1 = Cf_base * (1 - load_sensitivity * (Fz_f1 / 1000));
    Cr1 = Cr_base * (1 - load_sensitivity * (Fz_r1 / 1000));
    balance_sector1(i) = (Fy_f1 / Cf1) - (Fy_r1 / Cr1);
    
    %% --- SECTOR 2 BALANCE ---
    % Evaluated using the identical frozen ay2 and V2 values across all wheelbases
    Fy_f2 = (M * ay2 * l_r) / L;
    Fy_r2 = (M * ay2 * l_f) / L;
    
    % High speed aero load balance coupling
    Fz_f2 = ((M * g * l_r) / L) + (0.5 * rho * (V2^2) * ClA * aero_dist_f);
    Fz_r2 = ((M * g * l_f) / L) + (0.5 * rho * (V2^2) * ClA * aero_dist_r);
    
    Cf2 = Cf_base * (1 - load_sensitivity * (Fz_f2 / 1000));
    Cr2 = Cr_base * (1 - load_sensitivity * (Fz_r2 / 1000));
    balance_sector2(i) = (Fy_f2 / Cf2) - (Fy_r2 / Cr2);
    
    % Add the Kinematic Ackermann component to get Total Steering Wheel Angle (in degrees)
    steer_total_sector1(i) = rad2deg(L / R1) + rad2deg(balance_sector1(i));
    steer_total_sector2(i) = rad2deg(L / R2) + rad2deg(balance_sector2(i));
end

% Compute fixed pure kinematic steering baseline for current 62-inch wheelbase configuration
L_current = 62 * 0.0254;
steer_kinematic_baseline1 = rad2deg(L_current / R1);
steer_kinematic_baseline2 = rad2deg(L_current / R2);

%% 5. Plot Dynamic Handling Results
disp('Step 5: Generating high-resolution balance visualizations...');
figure('Position', [150, 200, 1100, 480]);
% --- LEFT SUBPLOT: LOW-SPEED HAIRPIN ---
subplot(1,2,1);
plot(wheelbase_sweep_in, steer_total_sector1, 'b-o', 'LineWidth', 2.5, ...
    'DisplayName', sprintf('Low-Speed (R=%.1fm, V=%.1fm/s)', R1, V1));
hold on;
yline(steer_kinematic_baseline1, 'w--', 'LineWidth', 1.5, ...
    'DisplayName', 'Current 62in Kinematic Baseline');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Total Steering Angle (degrees)', 'FontWeight', 'bold', 'FontSize', 11);
title('Low-Speed Corner Steering Demand', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');
% --- RIGHT SUBPLOT: HIGH-SPEED SWEEPER ---
subplot(1,2,2);
plot(wheelbase_sweep_in, steer_total_sector2, 'r-o', 'LineWidth', 2.5, ...
    'DisplayName', sprintf('High-Speed (R=%.1fm, V=%.1fm/s)', R2, V2));
hold on;
yline(steer_kinematic_baseline2, 'w--', 'LineWidth', 1.5, ...
    'DisplayName', 'Current 62in Kinematic Baseline');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Total Steering Angle (degrees)', 'FontWeight', 'bold', 'FontSize', 11);
title('High-Speed Corner Steering Demand', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');
%}