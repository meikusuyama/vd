%% Master RCVD Bicycle Model & Lapsim Points Optimization Suite
clear; clc;
setup_paths;

%% 1. Initialize Baseline Vehicle & Lock Static Parameters
disp('Step 1: Loading baseline car and locking static weight distribution...');
carCell = carConfig(); 
car = carCell{1,1};       
accelCar = carCell{1,2};  
numWorkers = 16; 

g = car.g;
track_width = car.t_f;    
rho = 1.162;              
ClA_base = car.aero.cla;       
aero_dist_f = 0.539; 
aero_dist_r = 1 - aero_dist_f;

% --- STATIC WEIGHT DISTRIBUTION SETUP ---
M = car.M;                        
weight_dist_r = 0.512;            
weight_dist_f = 1 - weight_dist_r;
total_weight = M * g;

% --- UNDERTRAY DOWNFORCE SCALING PARAMETERS ---
ut_fraction = 230 / 830;          
ut_sensitivity = 1.0;             
baseline_L_in = 62.0;             

fprintf('----------------------------------------------------\n');
fprintf('  STATIC CONFIGURATION LOCKED\n');
fprintf('----------------------------------------------------\n');
fprintf('Total Vehicle Mass        : %6.2f kg (%5.1f lbs)\n', M, M * 2.20462);
fprintf('Static Rear Distribution  : %6.2f %%\n', weight_dist_r * 100);
fprintf('----------------------------------------------------\n\n');

%% 2. Run Baseline Track Solver & Extract Corner Profiles
disp('Step 2: Processing baseline track solver to cluster velocity conditions...');
paramsArr = gg2(car, numWorkers); 
car_base = makeGG(paramsArr, car);
comp_base = Events2(car_base, accelCar);
comp_base.calcTimes(); 

load('michigantrack2024.mat'); 
V_profile  = comp_base.autocross.long_vel;
ay_profile = comp_base.autocross.lat_accel;

% Extract 3 Low-Speed Hairpins & 3 High-Speed Sweepers
[~, locs_low] = findpeaks(abs(curvature), 'SortStr', 'descend', 'MinPeakDistance', 50);
idx_low_3 = locs_low(1:3);
R_low = 1 ./ abs(curvature(idx_low_3));
V_low = V_profile(idx_low_3);
ay_low = abs(ay_profile(idx_low_3));

sweep_candidates = find(abs(curvature) > 0.015 & abs(curvature) < 0.035);
[~, sweep_locs] = findpeaks(V_profile(sweep_candidates), 'SortStr', 'descend', 'MinPeakDistance', 30);
idx_high_3 = sweep_candidates(sweep_locs(1:3));
R_high = 1 ./ abs(curvature(idx_high_3));
V_high = V_profile(idx_high_3);
ay_high = abs(ay_profile(idx_high_3));

%% 3. Dynamic Empirical Tire Load Sensitivity
disp('Step 3: Calculating empirical Pacejka load sensitivity slopes...');
Fz_f_stat = (total_weight * weight_dist_f) / 2;
Fz_r_stat = (total_weight * weight_dist_r) / 2;
alpha_deg = 0.1; 
alpha_rad = alpha_deg * (pi / 180);

Fy_f1 = car.tire.F_y(alpha_deg, 0, Fz_f_stat, 0);
Fy_f2 = car.tire.F_y(alpha_deg, 0, Fz_f_stat + 500, 0);
Fy_r1 = car.tire.F_y(alpha_deg, 0, Fz_r_stat, 0);
Fy_r2 = car.tire.F_y(alpha_deg, 0, Fz_r_stat + 500, 0);

Cf_base = abs(2 * (Fy_f1 / alpha_rad));
Cr_base = abs(2 * (Fy_r1 / alpha_rad));
Cf_loaded = abs(2 * (Fy_f2 / alpha_rad));
Cr_loaded = abs(2 * (Fy_r2 / alpha_rad));

dCf_dFz = (Cf_base - Cf_loaded) / 1000; 
dCr_dFz = (Cr_base - Cr_loaded) / 1000;

%% 4. Wheelbase Parameter Sweep Loop (RCVD & Lapsim Evaluation)
disp('Step 4: Sweeping wheelbase and running full lapsim event solvers...');
wheelbase_sweep_in = linspace(60, 62.5, 6); 
numSteps = length(wheelbase_sweep_in);

steer_low  = zeros(numSteps, 3);
steer_high = zeros(numSteps, 3);
SI_low     = zeros(numSteps, 3);
SI_high    = zeros(numSteps, 3);
vel_sweep_mps = linspace(5, 35, 10); 
sens_matrix   = zeros(numSteps, length(vel_sweep_mps));

% Pre-allocate Lapsim points array (5 categories x numSteps)
points_array = zeros(5, numSteps);

for i = 1:numSteps
    L = wheelbase_sweep_in(i) * 0.0254; % meters
    l_f = L * weight_dist_r;            
    l_r = L * weight_dist_f;            
    
    % --- 4A: DYNAMIC UNDERTRAY DEGRADATION ---
    length_ratio = wheelbase_sweep_in(i) / baseline_L_in;
    dynamic_ClA = ClA_base * ((1 - ut_fraction) + ut_fraction * (length_ratio ^ ut_sensitivity));
    
    % --- 4B: UPDATE CAR OBJECTS & RUN LAPSIM ---
    fprintf('Running Lapsim for %.1f inch wheelbase (%d/%d)...\n', wheelbase_sweep_in(i), i, numSteps);
    
    % Inject new dimensions and aero into the simulation objects
    car.W_b = L; car.l_f = l_f; car.l_r = l_r; car.aero.cla = dynamic_ClA;
    accelCar.W_b = L; accelCar.l_f = l_f; accelCar.l_r = l_r; accelCar.aero.cla = dynamic_ClA;
    
    % Run specific gg2 and events for this wheelbase step
    paramsArr_sweep = gg2(car, 0); 
    car_sweep = makeGG(paramsArr_sweep, car);
    comp_sweep = Events2(car_sweep, accelCar);
    comp_sweep.calcTimes();
    
    % Extract points directly from Events2 object
    pts = comp_sweep.points;
    points_array(:, i) = [pts.accel; pts.autocross; pts.endurance; pts.skidpad; pts.total];
    
    % --- 4C: RCVD BICYCLE MODEL CALCULATIONS ---
    % Process 3 Low-Speed Corners
    for c = 1:3
        Fy_f = (M * ay_low(c) * l_r) / L;
        Fy_r = (M * ay_low(c) * l_f) / L;
        Fz_f = ((M * g * l_r) / L) + (0.5 * rho * (V_low(c)^2) * dynamic_ClA * aero_dist_f);
        Fz_r = ((M * g * l_f) / L) + (0.5 * rho * (V_low(c)^2) * dynamic_ClA * aero_dist_r);
        Cf = Cf_base - (dCf_dFz * (Fz_f - (total_weight*weight_dist_f)));
        Cr = Cr_base - (dCr_dFz * (Fz_r - (total_weight*weight_dist_r)));
        steer_low(i, c) = rad2deg(L / R_low(c)) + rad2deg((Fy_f / Cf) - (Fy_r / Cr));
        SI_low(i, c) = -((l_f*Cf - l_r*Cr)/(L*(Cf+Cr))) + (((Cf*Cr*L^2)/(2*M*(Cf+Cr))) * (R_low(c)*g)/(V_low(c)^2))/1000; 
    end
    
    % Process 3 High-Speed Corners
    for c = 1:3
        Fy_f = (M * ay_high(c) * l_r) / L;
        Fy_r = (M * ay_high(c) * l_f) / L;
        Fz_f = ((M * g * l_r) / L) + (0.5 * rho * (V_high(c)^2) * dynamic_ClA * aero_dist_f);
        Fz_r = ((M * g * l_f) / L) + (0.5 * rho * (V_high(c)^2) * dynamic_ClA * aero_dist_r);
        Cf = Cf_base - (dCf_dFz * (Fz_f - (total_weight*weight_dist_f)));
        Cr = Cr_base - (dCr_dFz * (Fz_r - (total_weight*weight_dist_r)));
        steer_high(i, c) = rad2deg(L / R_high(c)) + rad2deg((Fy_f / Cf) - (Fy_r / Cr));
        SI_high(i, c) = -((l_f*Cf - l_r*Cr)/(L*(Cf+Cr))) + (((Cf*Cr*L^2)/(2*M*(Cf+Cr))) * (R_high(c)*g)/(V_high(c)^2))/1000;
    end
    
    % Calculate RCVD Fig 5.52 Steering Sensitivity
    for v = 1:length(vel_sweep_mps)
        V_val = vel_sweep_mps(v);
        Fz_f_val = ((M * g * l_r) / L) + (0.5 * rho * (V_val^2) * dynamic_ClA * aero_dist_f);
        Fz_r_val = ((M * g * l_f) / L) + (0.5 * rho * (V_val^2) * dynamic_ClA * aero_dist_r);
        Cf_val = Cf_base - (dCf_dFz * (Fz_f_val - (total_weight*weight_dist_f)));
        Cr_val = Cr_base - (dCr_dFz * (Fz_r_val - (total_weight*weight_dist_r)));
        UG_deg_per_g = rad2deg((total_weight * weight_dist_f) / Cf_val - (total_weight * weight_dist_r) / Cr_val);
        ack_grad_deg_per_g = rad2deg(L * g / (V_val^2));
        sens_matrix(i, v) = 1 / (UG_deg_per_g + ack_grad_deg_per_g);
    end
end

%% 5. Plot RCVD Diagnostics (Cleaned up format)
disp('Step 5: Rendering clean diagnostic tabs...');
colors = ['b', 'c', 'm']; 

% TAB 1: LOW-SPEED STEERING DEMAND
figure('Name', 'Low-Speed Steering', 'Position', [100, 100, 750, 500]);
hold on;
for c = 1:3
    plot(wheelbase_sweep_in, steer_low(:, c), [colors(c) '-o'], 'LineWidth', 2.5, ...
        'DisplayName', sprintf('Corner %d (R=%.1fm, V=%.1fm/s)', c, R_low(c), V_low(c)));
end
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Total Steering Angle (degrees)', 'FontWeight', 'bold', 'FontSize', 11);
title('LOW-SPEED CLUSTER: Steering Demand', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% TAB 2: HIGH-SPEED STEERING DEMAND
figure('Name', 'High-Speed Steering', 'Position', [150, 150, 750, 500]);
hold on;
for c = 1:3
    plot(wheelbase_sweep_in, steer_high(:, c), [colors(c) '-o'], 'LineWidth', 2.5, ...
        'DisplayName', sprintf('Sweeper %d (R=%.1fm, V=%.1fm/s)', c, R_high(c), V_high(c)));
end
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Total Steering Angle (degrees)', 'FontWeight', 'bold', 'FontSize', 11);
title('HIGH-SPEED CLUSTER: Steering Demand', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% TAB 3: RCVD STABILITY INDEX (SI)
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

% TAB 4: RCVD FIG 5.52 STEERING SENSITIVITY
figure('Name', 'Steering Sensitivity', 'Position', [250, 250, 750, 500]);
hold on;
colors_sens = cool(numSteps); 
for i = 1:numSteps
    if abs(wheelbase_sweep_in(i) - 62) < 0.1
        plot(vel_sweep_mps, sens_matrix(i, :), '--o', 'Color', colors_sens(i, :), 'LineWidth', 3, ...
            'MarkerSize', 6, 'MarkerFaceColor', colors_sens(i, :), ...
            'DisplayName', sprintf('%.1f in Wheelbase (Baseline)', wheelbase_sweep_in(i)));
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
title('RCVD Fig 5.52: Steering Sensitivity across 10 Speeds', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);

%% 6. Plot Lapsim Competition Points (Tab 5)
disp('Step 6: Rendering Lapsim Competition Points Bar Chart...');

figure('Name', 'FSAE Competition Points', 'Position', [300, 300, 900, 500]);
points_categories = {'Accel', 'Autocross', 'Endurance', 'Skidpad', 'Total'};

% Format wheelbase labels automatically for the legend
car_labels = cell(1, numSteps);
for i = 1:numSteps
    car_labels{i} = sprintf('%.1f in', wheelbase_sweep_in(i));
end

% Ensure MATLAB does not randomly re-order the categorical x-axis
pre_ordered_categorical = @(x) reordercats(categorical(x), x);

% Plot grouped bar chart
b = bar(pre_ordered_categorical(points_categories), points_array);

legendObj = legend(car_labels, 'location', 'northwest');
title(legendObj, 'Wheelbase');
title('Simulated FSAE Competition Points vs. Wheelbase');
ylabel('Points');
grid on;

% Label bar heights above each bar (matches your plot_lapsim_points format)
for i = 1:numel(b)
    xtips = b(i).XEndPoints;
    ytips = b(i).YEndPoints;
    labels = string(round(b(i).YData, 1));
    text(xtips, ytips, labels, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 8, 'FontWeight', 'bold')
end

disp('All simulation runs and clean plotting complete.');