
%% Master High-Fidelity Wheelbase & Multi-Event Analysis Suite
clear; clc;
setup_paths;

%% 1. Initialize Baseline Vehicle Objects
disp('Step 1: Loading vehicle configurations...');
carCell = carConfig(); 
car = carCell{1,1};       
accelCar = carCell{1,2};  
numWorkers = 16; 

%% 2. Run Run-Once Track Solver to Extract Corner Targets
disp('Step 2: Processing track map to isolate sector velocities...');
paramsArr = gg2(car, numWorkers); 
car = makeGG(paramsArr, car);

comp = Events2(car, accelCar);
comp.calcTimes(); 

load('michigantrack2024.mat'); % Loads 'arclength' and 'curvature'
V_profile  = comp.autocross.long_vel;
ay_profile = comp.autocross.lat_accel;

% Extract Target Sector 1 (Low-Speed Hairpin)
[max_curve, idx_low] = max(abs(curvature));
R1 = 1 / max_curve;                  
V1 = V_profile(idx_low);             

% Extract Target Sector 2 (High-Speed Sweeper)
corner_indices = find(abs(curvature) > 0.015 & abs(curvature) < 0.04);
[~, max_v_idx] = max(V_profile(corner_indices));
idx_high = corner_indices(max_v_idx);
R2 = 1 / abs(curvature(idx_high));   
V2 = V_profile(idx_high);            

fprintf('\n====================================================\n');
fprintf('  FROZEN TRACK BENCHMARKS EXTRACTED\n');
fprintf('====================================================\n');
fprintf('Sector 1 Hairpin: R = %5.2f m | V = %5.2f m/s\n', R1, V1);
fprintf('Sector 2 Sweeper: R = %5.2f m | V = %5.2f m/s\n', R2, V2);
fprintf('====================================================\n\n');

%% 3. Execute High-Fidelity Wheelbase Parameter Sweep
disp('Step 3: Beginning 4-wheel non-linear optimization & event sweep...');
wheelbase_sweep_in = linspace(60, 62.5, 6); 
numSteps = length(wheelbase_sweep_in);

% Pre-allocate arrays for true solved steering and slip metrics
steer_true_sector1  = zeros(size(wheelbase_sweep_in));
steer_true_sector2  = zeros(size(wheelbase_sweep_in));
alpha_split_sector1 = zeros(size(wheelbase_sweep_in));
alpha_split_sector2 = zeros(size(wheelbase_sweep_in));
sim_accel_time      = zeros(size(wheelbase_sweep_in));

for i = 1:numSteps
    L = wheelbase_sweep_in(i) * 0.0254; % Convert target step to meters
    
    % Mutate geometric parameters inside BOTH active vehicle objects
    car.W_b = L;
    car.l_f = L * 0.512;       % Lock front distribution leverage ratio
    car.l_r = L * (1 - 0.512); % Lock rear distribution leverage ratio
    
    accelCar.W_b = L;
    accelCar.l_f = L * 0.512;
    accelCar.l_r = L * (1 - 0.512);
    
    fprintf('Optimizing state vectors for Wheelbase: %.2f in (%d of %d)...\n', ...
            wheelbase_sweep_in(i), i, numSteps);
        
    %% --- SOLVE SECTOR 1 (LOW SPEED HANDLING) ---
    [~, x_guess1] = constant_radius(R1, 3, car); 
    x0_1 = x_guess1;
    x0_1(3) = V1;       
    x0_1(5) = V1 / R1;  
    
    [x_table1, ~] = constant_radius(R1, V1, car, x0_1);
    steer_true_sector1(i) = x_table1{1, 'steer_angle'};
    
    % Extract true 4-wheel slip angles to characterize understeer
    % Note: Tire 1 & 2 are Front; Tire 3 & 4 are Rear
    alpha_f1 = (abs(x_table1{1, 'alpha_1'}) + abs(x_table1{1, 'alpha_2'})) / 2;
    alpha_r1 = (abs(x_table1{1, 'alpha_3'}) + abs(x_table1{1, 'alpha_4'})) / 2;
    alpha_split_sector1(i) = alpha_f1 - alpha_r1; % (+) is Understeer, (-) is Oversteer
    
    %% --- SOLVE SECTOR 2 (HIGH SPEED HANDLING) ---
    [~, x_guess2] = constant_radius(R2, 3, car);
    x0_2 = x_guess2;
    x0_2(3) = V2;       
    x0_2(5) = V2 / R2;  
    
    [x_table2, ~] = constant_radius(R2, V2, car, x0_2);
    steer_true_sector2(i) = x_table2{1, 'steer_angle'};
    
    alpha_f2 = (abs(x_table2{1, 'alpha_1'}) + abs(x_table2{1, 'alpha_2'})) / 2;
    alpha_r2 = (abs(x_table2{1, 'alpha_3'}) + abs(x_table2{1, 'alpha_4'})) / 2;
    alpha_split_sector2(i) = alpha_f2 - alpha_r2;
    
    %% --- SOLVE FULL LAPSIM EVENTS (Lapsim2 Aligned) ---
    % Re-run gg2 on the regular 'car' object matching standard Lapsim2 architecture
    paramsArr_car = gg2(car, 0);
    car = makeGG(paramsArr_car, car);
    
    comp_step = Events2(car, accelCar);
    comp_step.calcTimes();
    
    % Extract official 75m Acceleration Time
    try
        sim_accel_time(i) = comp_step.accel.time;
    catch
        try
            sim_accel_time(i) = comp_step.times.accel;
        catch
            sim_accel_time(i) = comp_step.accel_time;
        end
    end
end

% Compute fixed pure kinematic baseline angle using current 62-inch chassis length
L_current = 62 * 0.0254;
steer_kinematic_baseline1 = rad2deg(L_current / R1);
steer_kinematic_baseline2 = rad2deg(L_current / R2);

%% 4. Plot High-Fidelity Results across Separate Tabs/Figures
disp('Step 4: Rendering independent figure tabs...');

% =========================================================================
% FIGURE / TAB 1: LOW-SPEED SECTOR ANALYSIS
% =========================================================================
figure('Name', 'Low-Speed Analysis', 'Position', [100, 100, 750, 650]);

% Top Subplot: Steering Demand
subplot(2,1,1);
plot(wheelbase_sweep_in, steer_true_sector1, 'b-o', 'LineWidth', 2.5, 'DisplayName', sprintf('Solved 4-Wheel Model (R=%.1fm)', R1));
hold on;
yline(steer_kinematic_baseline1, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Current 62in Kinematic Base');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
ylabel('Steering Angle (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title(sprintf('LOW-SPEED HAIRPIN: Steering Demand (V = %.1f m/s)', V1), 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Bottom Subplot: True Understeer Characterization
subplot(2,1,2);
plot(wheelbase_sweep_in, alpha_split_sector1, 'b-o', 'LineWidth', 2.5, 'DisplayName', '\alpha_f - \alpha_r Balance');
hold on;
yline(0, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Neutral Steer Line (0 deg)');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('\alpha_f - \alpha_r Split (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title('LOW-SPEED HAIRPIN: True Understeer Balance', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% FIXED YLIM BUG HERE: Store limits in 'yl1' first!
yl1 = ylim; 
ymid1 = mean(yl1);
text(wheelbase_sweep_in(2), ymid1 + (yl1(2)-ymid1)*0.3, '\leftarrow Understeer Zone (+)', 'FontWeight', 'bold', 'Color', [0.2 0.5 0.2]);
text(wheelbase_sweep_in(2), ymid1 - (ymid1-yl1(1))*0.3, '\leftarrow Oversteer Zone (-)', 'FontWeight', 'bold', 'Color', [0.7 0.2 0.2]);

% =========================================================================
% FIGURE / TAB 2: HIGH-SPEED SECTOR ANALYSIS
% =========================================================================
figure('Name', 'High-Speed Analysis', 'Position', [150, 150, 750, 650]);

% Top Subplot: Steering Demand
subplot(2,1,1);
plot(wheelbase_sweep_in, steer_true_sector2, 'r-o', 'LineWidth', 2.5, 'DisplayName', sprintf('Solved 4-Wheel Model (R=%.1fm)', R2));
hold on;
yline(steer_kinematic_baseline2, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Current 62in Kinematic Base');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
ylabel('Steering Angle (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title(sprintf('HIGH-SPEED SWEEPER: Steering Demand (V = %.1f m/s)', V2), 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Bottom Subplot: True Understeer Characterization
subplot(2,1,2);
plot(wheelbase_sweep_in, alpha_split_sector2, 'r-o', 'LineWidth', 2.5, 'DisplayName', '\alpha_f - \alpha_r Balance');
hold on;
yline(0, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Neutral Steer Line (0 deg)');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('\alpha_f - \alpha_r Split (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title('HIGH-SPEED SWEEPER: True Understeer Balance (Aero Coupled)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% FIXED YLIM BUG HERE: Store limits in 'yl2' first!
yl2 = ylim; 
ymid2 = mean(yl2);
text(wheelbase_sweep_in(2), ymid2 + (yl2(2)-ymid2)*0.3, '\leftarrow Understeer Zone (+)', 'FontWeight', 'bold', 'Color', [0.2 0.5 0.2]);
text(wheelbase_sweep_in(2), ymid2 - (ymid2-yl2(1))*0.3, '\leftarrow Oversteer Zone (-)', 'FontWeight', 'bold', 'Color', [0.7 0.2 0.2]);

% =========================================================================
% FIGURE / TAB 3: ACCELERATION EVENT PERFORMANCE
% =========================================================================
figure('Name', 'Acceleration Event', 'Position', [200, 200, 750, 450]);

plot(wheelbase_sweep_in, sim_accel_time, 'g-o', 'LineWidth', 3, 'MarkerFaceColor', 'g', 'DisplayName', 'Simulated 75m Sprint');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('75m Sprint Time (seconds)', 'FontWeight', 'bold', 'FontSize', 11);
title('ACCELERATION EVENT: Longitudinal Load Transfer Impact', 'FontSize', 12, 'FontWeight', 'bold');

% Highlight current 62-inch baseline on acceleration graph
idx_62 = find(abs(wheelbase_sweep_in - 62) < 0.2, 1);
if ~isempty(idx_62)
    hold on;
    plot(wheelbase_sweep_in(idx_62), sim_accel_time(idx_62), 'ro', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Current 62in Baseline');
    legend('Location', 'best', 'FontSize', 10);
end

disp('All diagnostic tabs rendered successfully.');

%{
%% Master High-Fidelity Wheelbase & Multi-Event Analysis Suite
clear; clc;
setup_paths;

%% 1. Initialize Baseline Vehicle Objects
disp('Step 1: Loading vehicle configurations...');
carCell = carConfig(); 
car = carCell{1,1};       
accelCar = carCell{1,2};  
numWorkers = 16; 

%% 2. Run Run-Once Track Solver to Extract Corner Targets
disp('Step 2: Processing track map to isolate sector velocities...');
paramsArr = gg2(car, numWorkers); 
car = makeGG(paramsArr, car);

comp = Events2(car, accelCar);
comp.calcTimes(); 

load('michigantrack2024.mat'); % Loads 'arclength' and 'curvature'
V_profile  = comp.autocross.long_vel;
ay_profile = comp.autocross.lat_accel;

% Extract Target Sector 1 (Low-Speed Hairpin)
[max_curve, idx_low] = max(abs(curvature));
R1 = 1 / max_curve;                  
V1 = V_profile(idx_low);             

% Extract Target Sector 2 (High-Speed Sweeper)
corner_indices = find(abs(curvature) > 0.015 & abs(curvature) < 0.04);
[~, max_v_idx] = max(V_profile(corner_indices));
idx_high = corner_indices(max_v_idx);
R2 = 1 / abs(curvature(idx_high));   
V2 = V_profile(idx_high);            

fprintf('\n====================================================\n');
fprintf('  FROZEN TRACK BENCHMARKS EXTRACTED\n');
fprintf('====================================================\n');
fprintf('Sector 1 Hairpin: R = %5.2f m | V = %5.2f m/s\n', R1, V1);
fprintf('Sector 2 Sweeper: R = %5.2f m | V = %5.2f m/s\n', R2, V2);
fprintf('====================================================\n\n');

%% 3. Execute High-Fidelity Wheelbase Parameter Sweep
disp('Step 3: Beginning 4-wheel non-linear optimization & event sweep...');
wheelbase_sweep_in = linspace(60, 62.5, 6); 
numSteps = length(wheelbase_sweep_in);

% Pre-allocate arrays for true solved steering and slip metrics
steer_true_sector1  = zeros(size(wheelbase_sweep_in));
steer_true_sector2  = zeros(size(wheelbase_sweep_in));
alpha_split_sector1 = zeros(size(wheelbase_sweep_in));
alpha_split_sector2 = zeros(size(wheelbase_sweep_in));
sim_accel_time      = zeros(size(wheelbase_sweep_in));

for i = 1:numSteps
    L = wheelbase_sweep_in(i) * 0.0254; % Convert target step to meters
    
    % Mutate geometric parameters inside BOTH active vehicle objects
    car.W_b = L;
    car.l_f = L * 0.512;       % Lock front distribution leverage ratio
    car.l_r = L * (1 - 0.512); % Lock rear distribution leverage ratio
    
    accelCar.W_b = L;
    accelCar.l_f = L * 0.512;
    accelCar.l_r = L * (1 - 0.512);
    
    fprintf('Optimizing state vectors for Wheelbase: %.2f in (%d of %d)...\n', ...
            wheelbase_sweep_in(i), i, numSteps);
        
    %% --- SOLVE SECTOR 1 (LOW SPEED HANDLING) ---
    [~, x_guess1] = constant_radius(R1, 3, car); 
    x0_1 = x_guess1;
    x0_1(3) = V1;       
    x0_1(5) = V1 / R1;  
    
    [x_table1, ~] = constant_radius(R1, V1, car, x0_1);
    steer_true_sector1(i) = x_table1{1, 'steer_angle'};
    
    % Extract true 4-wheel slip angles to characterize understeer
    % Note: Tire 1 & 2 are Front; Tire 3 & 4 are Rear
    alpha_f1 = (abs(x_table1{1, 'alpha_1'}) + abs(x_table1{1, 'alpha_2'})) / 2;
    alpha_r1 = (abs(x_table1{1, 'alpha_3'}) + abs(x_table1{1, 'alpha_4'})) / 2;
    alpha_split_sector1(i) = alpha_f1 - alpha_r1; % (+) is Understeer, (-) is Oversteer
    
    %% --- SOLVE SECTOR 2 (HIGH SPEED HANDLING) ---
    [~, x_guess2] = constant_radius(R2, 3, car);
    x0_2 = x_guess2;
    x0_2(3) = V2;       
    x0_2(5) = V2 / R2;  
    
    [x_table2, ~] = constant_radius(R2, V2, car, x0_2);
    steer_true_sector2(i) = x_table2{1, 'steer_angle'};
    
    alpha_f2 = (abs(x_table2{1, 'alpha_1'}) + abs(x_table2{1, 'alpha_2'})) / 2;
    alpha_r2 = (abs(x_table2{1, 'alpha_3'}) + abs(x_table2{1, 'alpha_4'})) / 2;
    alpha_split_sector2(i) = alpha_f2 - alpha_r2;
    
    %% --- SOLVE ACCELERATION EVENT ---
    % Re-run gg2 for accelCar so longitudinal weight transfer limits update
    paramsArr_accel = gg2(accelCar, 0); % 0 workers for quick single-run speed
    accelCar = makeGG(paramsArr_accel, accelCar);
    
    comp_step = Events2(car, accelCar);
    comp_step.calcTimes();
    
    % Extract official 75m Acceleration Time
    try
        sim_accel_time(i) = comp_step.accel.time;
    catch
        try
            sim_accel_time(i) = comp_step.times.accel;
        catch
            sim_accel_time(i) = comp_step.accel_time;
        end
    end
end

% Compute fixed pure kinematic baseline angle using current 62-inch chassis length
L_current = 62 * 0.0254;
steer_kinematic_baseline1 = rad2deg(L_current / R1);
steer_kinematic_baseline2 = rad2deg(L_current / R2);

%% 4. Plot High-Fidelity Results across Separate Tabs/Figures
disp('Step 4: Rendering independent figure tabs...');

% =========================================================================
% FIGURE / TAB 1: LOW-SPEED SECTOR ANALYSIS
% =========================================================================
figure('Name', 'Low-Speed Analysis', 'Position', [100, 100, 750, 650]);

% Top Subplot: Steering Demand
subplot(2,1,1);
plot(wheelbase_sweep_in, steer_true_sector1, 'b-o', 'LineWidth', 2.5, 'DisplayName', sprintf('Solved 4-Wheel Model (R=%.1fm)', R1));
hold on;
yline(steer_kinematic_baseline1, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Current 62in Kinematic Base');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
ylabel('Steering Angle (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title(sprintf('LOW-SPEED HAIRPIN: Steering Demand (V = %.1f m/s)', V1), 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Bottom Subplot: True Understeer Characterization
subplot(2,1,2);
plot(wheelbase_sweep_in, alpha_split_sector1, 'b-o', 'LineWidth', 2.5, 'DisplayName', '\alpha_f - \alpha_r Balance');
hold on;
yline(0, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Neutral Steer Line (0 deg)');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('\alpha_f - \alpha_r Split (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title('LOW-SPEED HAIRPIN: True Understeer Balance', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% FIXED YLIM BUG HERE: Store limits in 'yl1' first!
yl1 = ylim; 
ymid1 = mean(yl1);
text(wheelbase_sweep_in(2), ymid1 + (yl1(2)-ymid1)*0.3, '\leftarrow Understeer Zone (+)', 'FontWeight', 'bold', 'Color', [0.2 0.5 0.2]);
text(wheelbase_sweep_in(2), ymid1 - (ymid1-yl1(1))*0.3, '\leftarrow Oversteer Zone (-)', 'FontWeight', 'bold', 'Color', [0.7 0.2 0.2]);

% =========================================================================
% FIGURE / TAB 2: HIGH-SPEED SECTOR ANALYSIS
% =========================================================================
figure('Name', 'High-Speed Analysis', 'Position', [150, 150, 750, 650]);

% Top Subplot: Steering Demand
subplot(2,1,1);
plot(wheelbase_sweep_in, steer_true_sector2, 'r-o', 'LineWidth', 2.5, 'DisplayName', sprintf('Solved 4-Wheel Model (R=%.1fm)', R2));
hold on;
yline(steer_kinematic_baseline2, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Current 62in Kinematic Base');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
ylabel('Steering Angle (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title(sprintf('HIGH-SPEED SWEEPER: Steering Demand (V = %.1f m/s)', V2), 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Bottom Subplot: True Understeer Characterization
subplot(2,1,2);
plot(wheelbase_sweep_in, alpha_split_sector2, 'r-o', 'LineWidth', 2.5, 'DisplayName', '\alpha_f - \alpha_r Balance');
hold on;
yline(0, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Neutral Steer Line (0 deg)');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('\alpha_f - \alpha_r Split (deg)', 'FontWeight', 'bold', 'FontSize', 11);
title('HIGH-SPEED SWEEPER: True Understeer Balance (Aero Coupled)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% FIXED YLIM BUG HERE: Store limits in 'yl2' first!
yl2 = ylim; 
ymid2 = mean(yl2);
text(wheelbase_sweep_in(2), ymid2 + (yl2(2)-ymid2)*0.3, '\leftarrow Understeer Zone (+)', 'FontWeight', 'bold', 'Color', [0.2 0.5 0.2]);
text(wheelbase_sweep_in(2), ymid2 - (ymid2-yl2(1))*0.3, '\leftarrow Oversteer Zone (-)', 'FontWeight', 'bold', 'Color', [0.7 0.2 0.2]);

% =========================================================================
% FIGURE / TAB 3: ACCELERATION EVENT PERFORMANCE
% =========================================================================
figure('Name', 'Acceleration Event', 'Position', [200, 200, 750, 450]);

plot(wheelbase_sweep_in, sim_accel_time, 'g-o', 'LineWidth', 3, 'MarkerFaceColor', 'g', 'DisplayName', 'Simulated 75m Sprint');
grid on; box on;
xlim([wheelbase_sweep_in(1) wheelbase_sweep_in(end)]);
xlabel('Wheelbase Length (inches)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('75m Sprint Time (seconds)', 'FontWeight', 'bold', 'FontSize', 11);
title('ACCELERATION EVENT: Longitudinal Load Transfer Impact', 'FontSize', 12, 'FontWeight', 'bold');

% Highlight current 62-inch baseline on acceleration graph
idx_62 = find(abs(wheelbase_sweep_in - 62) < 0.2, 1);
if ~isempty(idx_62)
    hold on;
    plot(wheelbase_sweep_in(idx_62), sim_accel_time(idx_62), 'ro', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Current 62in Baseline');
    legend('Location', 'best', 'FontSize', 10);
end

disp('All diagnostic tabs rendered successfully.');
%}