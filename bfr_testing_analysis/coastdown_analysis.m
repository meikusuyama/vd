%% FSAE SHOCKPOT & DOWNFORCE ANALYSIS (CUSTOM SENSOR SELECTION)
clear; clc; close all;

%% 1. USER PARAMETERS & VEHICLE CONSTANTS
% Vehicle setup
k_spring_front = 52.54;  % Front spring rate (N/mm)
k_spring_rear  = 43.78;  % Rear spring rate (N/mm)
MR_front       = 0.847;  % Front motion ratio (wheel travel / shock travel)
MR_rear        = 0.984;  % Rear motion ratio (wheel travel / shock travel)

% Effective Wheel Rates (N/mm)
k_wheel_front = k_spring_front * (MR_front^2); 
k_wheel_rear  = k_spring_rear  * (MR_rear^2);

% Air density and global compression sign (+1 if compress = value increases)
rho = 1.225; 
shock_sign = 1; 

%% -------------------------------------------------------------------------
%% 2. SELECT ACTIVE SHOCKPOTS (Set true to include, false to ignore)
use_FL = true;   % Front Left
use_FR = false;  % Front Right (Set false if dead/corrupt)
use_RL = false;   % Rear Left
use_RR = true;   % Rear Right

%% INDIVIDUAL SENSOR GAIN CORRECTIONS (Multiplier for raw channel)
% Leave at 1.0 if using raw values, or reduce (e.g., 0.05) to correct DBC scale
gain_FL = 0.015;  % Scales FL's ~40 unit jump down to 2.0 mm
gain_FR = 0.000;  % Ignored
gain_RL = 0.050;  % Scales RL's ~12 unit jump up to match RR
gain_RR = 0.040;  % Scales RR's ~18 unit jump down to 2.0 mm
%% -------------------------------------------------------------------------

% Low-pass filter settings (2 Hz cutoff)
fs = 100; fc = 2;   
[b, a] = butter(2, fc / (fs/2), 'low');

%% 3. LOAD DATA
data_base    = readtable('testing_data/23_may3rd_log_2026-07-15T13-33-40.csv'); 
data_lowdrag = readtable('testing_data/24_may3rd_log_2026-07-15T13-16-00.csv');

%% 4. PROCESS DATASETS
sensors_config.flags = [use_FL, use_FR, use_RL, use_RR];
sensors_config.gains = [gain_FL, gain_FR, gain_RL, gain_RR];

base_results    = process_aero_run(data_base, b, a, k_wheel_front, k_wheel_rear, shock_sign, rho, sensors_config);
lowdrag_results = process_aero_run(data_lowdrag, b, a, k_wheel_front, k_wheel_rear, shock_sign, rho, sensors_config);

%% 5. DISPLAY RESULTS & COMPARISON
active_list = {'FL', 'FR', 'RL', 'RR'};
active_str = strjoin(active_list(sensors_config.flags), ', ');
if isempty(active_str), active_str = 'NONE'; end

fprintf('===================================================\n');
fprintf('               AERO ANALYSIS RESULTS               \n');
fprintf('  Active Sensors Used (%d Total): [%s]\n', sum(sensors_config.flags), active_str);
fprintf('===================================================\n');
fprintf('Baseline  : Cl*A = %.3f m^2 | Front CoP = %.1f%%\n', base_results.ClA, base_results.CoP_front);
fprintf('Low Drag  : Cl*A = %.3f m^2 | Front CoP = %.1f%%\n', lowdrag_results.ClA, lowdrag_results.CoP_front);
fprintf('Delta Cl*A: %.3f m^2 (%.1f%%)\n', ...
    lowdrag_results.ClA - base_results.ClA, ...
    ((lowdrag_results.ClA - base_results.ClA)/base_results.ClA)*100);
fprintf('===================================================\n');

%% 6. PLOTTING
figure('Name', 'Downforce vs Airspeed', 'Color', 'w');
v_fit = linspace(0, 25, 100); % 0 to ~90 km/h in m/s

scatter(base_results.v_valid * 3.6, base_results.F_down_valid, 12, [0 0.4470 0.7410], 'filled', 'MarkerFaceAlpha', 0.3); hold on;
scatter(lowdrag_results.v_valid * 3.6, lowdrag_results.F_down_valid, 12, [0.8500 0.3250 0.0980], 'filled', 'MarkerFaceAlpha', 0.3);

plot(v_fit * 3.6, 0.5 * rho * base_results.ClA * (v_fit.^2), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', sprintf('Baseline (ClA = %.2f m^2)', base_results.ClA));
plot(v_fit * 3.6, 0.5 * rho * lowdrag_results.ClA * (v_fit.^2), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5, 'DisplayName', sprintf('Low Drag (ClA = %.2f m^2)', lowdrag_results.ClA));

xlabel('Speed (km/h)'); ylabel('Total Downforce (N)');
title(sprintf('Downforce Fit using Sensors: [%s]', active_str));
grid on; legend('Location', 'northwest');


%% =========================================================================
%% HELPER FUNCTION: DATA PROCESSING PIPELINE
%% =========================================================================
function res = process_aero_run(df, b, a, kw_f, kw_r, sign_conv, rho, cfg)
    % Unpack config
    use_FL = cfg.flags(1); use_FR = cfg.flags(2);
    use_RL = cfg.flags(3); use_RR = cfg.flags(4);
    
    g_FL = cfg.gains(1); g_FR = cfg.gains(2);
    g_RL = cfg.gains(3); g_RR = cfg.gains(4);

    % 1. Sanitize channels
    v_kmh   = sanitize_channel((df.WheelSpeedFrontLeft + df.WheelSpeedFrontRight) / 2);
    s_fl    = sanitize_channel(df.SHOCKFL) * g_FL;
    s_fr    = sanitize_channel(df.SHOCKFR) * g_FR;
    s_rl    = sanitize_channel(df.SHOCKRL) * g_RL;
    s_rr    = sanitize_channel(df.SHOCKRR) * g_RR;
    acc_x   = sanitize_channel(df.ACCELX);
    acc_y   = sanitize_channel(df.ACCELY);
    steer   = sanitize_channel(df.STEERINGANGLE);
    
    v_ms = v_kmh / 3.6; 
    
    % 2. Low-pass filter
    s_fl_f  = filtfilt(b, a, s_fl);
    s_fr_f  = filtfilt(b, a, s_fr);
    s_rl_f  = filtfilt(b, a, s_rl);
    s_rr_f  = filtfilt(b, a, s_rr);
    accel_y = filtfilt(b, a, acc_y);
    accel_x = filtfilt(b, a, acc_x);
    
    % 3. Static zero offset (v < 3 km/h)
    static_mask = v_kmh < 3;
    if ~any(static_mask)
        error('No static baseline found in log! Make sure dataset includes car standing still.');
    end
    
    s_fl_0 = mean(s_fl_f(static_mask));
    s_fr_0 = mean(s_fr_f(static_mask));
    s_rl_0 = mean(s_rl_f(static_mask));
    s_rr_0 = mean(s_rr_f(static_mask));
    
    % 4. Relative Displacements
    ds_fl = (s_fl_f - s_fl_0) * sign_conv;
    ds_fr = (s_fr_f - s_fr_0) * sign_conv;
    ds_rl = (s_rl_f - s_rl_0) * sign_conv;
    ds_rr = (s_rr_f - s_rr_0) * sign_conv;
    
    % 5. Calculate Front Axle Force
    front_count = use_FL + use_FR;
    if front_count == 2
        F_front = (ds_fl + ds_fr) * kw_f;
    elseif use_FL
        F_front = (2 * ds_fl) * kw_f;
    elseif use_FR
        F_front = (2 * ds_fr) * kw_f;
    else
        F_front = zeros(size(v_kmh));
    end
    
    % 6. Calculate Rear Axle Force
    rear_count = use_RL + use_RR;
    if rear_count == 2
        F_rear = (ds_rl + ds_rr) * kw_r;
    elseif use_RL
        F_rear = (2 * ds_rl) * kw_r;
    elseif use_RR
        F_rear = (2 * ds_rr) * kw_r;
    else
        F_rear = zeros(size(v_kmh));
    end
    
    F_total = F_front + F_rear;
    
    % 7. Clean straight-line mask
    clean_mask = (abs(accel_y) < 0.2) & ...  
                 (abs(accel_x) < 0.15) & ... 
                 (v_kmh > 20) & ...          
                 (abs(steer) < 5);           
             
    v_clean = v_ms(clean_mask);
    F_clean = F_total(clean_mask);
    F_front_clean = F_front(clean_mask);
    
    if isempty(v_clean)
        error('No valid straight-line points matched the filter criteria.');
    end
    
    % 8. Fit ClA
    q = 0.5 * rho * (v_clean.^2); 
    ClA = q \ F_clean;            
    
    % 9. Center of Pressure (% Front)
    if mean(F_clean) > 0
        CoP_front = mean(F_front_clean ./ F_clean) * 100;
    else
        CoP_front = NaN;
    end
    
    % Outputs
    res.ClA = ClA;
    res.CoP_front = CoP_front;
    res.v_valid = v_clean;
    res.F_down_valid = F_clean;
end

function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end

%{
%% SHOCKPOT DIAGNOSTIC & SIGNAL INSPECTION SCRIPT
clear; clc; close all;

% File to inspect
data_file = 'testing_data/24_may3rd_log_2026-07-15T13-16-00.csv'; 

% Filter settings (2 Hz Low-Pass)
fs = 100; fc = 2;
[b, a] = butter(2, fc / (fs/2), 'low');

% Read & Sanitize Data
df = readtable(data_file);
time = df.Time;

s_fl_raw = sanitize_channel(df.SHOCKFL);
s_fr_raw = sanitize_channel(df.SHOCKFR);
s_rl_raw = sanitize_channel(df.SHOCKRL);
s_rr_raw = sanitize_channel(df.SHOCKRR);

s_fl_f = filtfilt(b, a, s_fl_raw);
s_fr_f = filtfilt(b, a, s_fr_raw);
s_rl_f = filtfilt(b, a, s_rl_raw);
s_rr_f = filtfilt(b, a, s_rr_raw);

% Plotting Grid
figure('Name', 'Shockpot Diagnostic Inspection', 'Position', [100, 100, 1100, 700]);

subplot(2,2,1);
plot(time, s_fl_raw, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw'); hold on;
plot(time, s_fl_f, 'r', 'LineWidth', 1.8, 'DisplayName', '2 Hz Filtered');
title('Front Left (SHOCKFL)'); xlabel('Time (s)'); ylabel('Signal / Travel'); grid on; legend;

subplot(2,2,2);
plot(time, s_fr_raw, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw'); hold on;
plot(time, s_fr_f, 'b', 'LineWidth', 1.8, 'DisplayName', '2 Hz Filtered');
title('Front Right (SHOCKFR)'); xlabel('Time (s)'); ylabel('Signal / Travel'); grid on; legend;

subplot(2,2,3);
plot(time, s_rl_raw, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw'); hold on;
plot(time, s_rl_f, 'r', 'LineWidth', 1.8, 'DisplayName', '2 Hz Filtered');
title('Rear Left (SHOCKRL)'); xlabel('Time (s)'); ylabel('Signal / Travel'); grid on; legend;

subplot(2,2,4);
plot(time, s_rr_raw, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw'); hold on;
plot(time, s_rr_f, 'b', 'LineWidth', 1.8, 'DisplayName', '2 Hz Filtered');
title('Rear Right (SHOCKRR)'); xlabel('Time (s)'); ylabel('Signal / Travel'); grid on; legend;

sgtitle(['Shockpot Signal Quality Inspection: ', data_file], 'Interpreter', 'none');

function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end
%}




% ====================================================

%{

%% FSAE SHOCKPOT & DOWNFORCE ANALYSIS (BASELINE VS LOW-DRAG)
clear; clc; close all;

%% 1. USER PARAMETERS & VEHICLE CONSTANTS
% Vehicle setup
k_spring_front = 52.54;  % Front spring rate (N/mm)
k_spring_rear  = 43.78;  % Rear spring rate (N/mm)
MR_front       = 0.847;  % Front motion ratio (wheel travel / shock travel)
MR_rear        = 0.984;  % Rear motion ratio (wheel travel / shock travel)

% Effective Wheel Rates (N/mm)
k_wheel_front = k_spring_front * (MR_front^2); 
k_wheel_rear  = k_spring_rear  * (MR_rear^2);

% Air density & Shockpot gain
rho = 1.225; % Air density (kg/m^3)
shock_gain = 1.0; % Convert raw CAN units to mm (Set to 1.0 if already in mm)

% Shockpot Compression Sign Switch:
% Set to +1 if shockpot value INCREASES when suspension compresses under downforce.
% Set to -1 if shockpot value DECREASES when suspension compresses.
shock_sign = 1; 

% Low-pass filter settings
fs = 100; % Sample rate of CAN log (Hz) - adjust if different
fc = 2;   % Cutoff frequency for aero filtering (Hz)
[b, a] = butter(2, fc / (fs/2), 'low');

%% 2. LOAD DATA
data_base    = readtable('testing_data/23_may3rd_log_2026-07-15T13-33-40.csv'); 
data_lowdrag = readtable('testing_data/24_may3rd_log_2026-07-15T13-16-00.csv');

%% 3. PROCESS DATASETS
base_results    = process_aero_run(data_base, b, a, k_wheel_front, k_wheel_rear, shock_gain, shock_sign, rho);
lowdrag_results = process_aero_run(data_lowdrag, b, a, k_wheel_front, k_wheel_rear, shock_gain, shock_sign, rho);

%% 4. DISPLAY RESULTS & COMPARISON
fprintf('===================================================\n');
fprintf('               AERO ANALYSIS RESULTS               \n');
fprintf('===================================================\n');
fprintf('Baseline  : Cl*A = %.3f m^2 | Front CoP = %.1f%%\n', base_results.ClA, base_results.CoP_front);
fprintf('Low Drag  : Cl*A = %.3f m^2 | Front CoP = %.1f%%\n', lowdrag_results.ClA, lowdrag_results.CoP_front);
fprintf('Delta Cl*A: %.3f m^2 (%.1f%%)\n', ...
    lowdrag_results.ClA - base_results.ClA, ...
    ((lowdrag_results.ClA - base_results.ClA)/base_results.ClA)*100);
fprintf('===================================================\n');

%% 5. PLOTTING
figure('Name', 'Downforce vs Airspeed');
v_fit = linspace(0, 25, 100); % 0 to ~90 km/h in m/s

% Plot raw scatter points (Filtered dynamic load vs speed)
scatter(base_results.v_valid * 3.6, base_results.F_down_valid, 12, [0 0.4470 0.7410], 'filled', 'MarkerFaceAlpha', 0.3); hold on;
scatter(lowdrag_results.v_valid * 3.6, lowdrag_results.F_down_valid, 12, [0.8500 0.3250 0.0980], 'filled', 'MarkerFaceAlpha', 0.3);

% Plot parabolic downforce fits
plot(v_fit * 3.6, 0.5 * rho * base_results.ClA * (v_fit.^2), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', sprintf('Baseline (ClA = %.2f m^2)', base_results.ClA));
plot(v_fit * 3.6, 0.5 * rho * lowdrag_results.ClA * (v_fit.^2), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5, 'DisplayName', sprintf('Low Drag (ClA = %.2f m^2)', lowdrag_results.ClA));

xlabel('Speed (km/h)');
ylabel('Total Downforce (N)');
title('Aerodynamic Downforce Comparison');
grid on; legend('Location', 'northwest');


%% =========================================================================
%% HELPER FUNCTION: DATA PROCESSING PIPELINE
%% =========================================================================
function res = process_aero_run(df, b, a, kw_f, kw_r, gain, sign_conv, rho)
    % 1. SANITIZE CHANNELS (Removes NaNs/Infs that crash filtfilt)
    v_kmh   = sanitize_channel((df.WheelSpeedFrontLeft + df.WheelSpeedFrontRight) / 2);
    s_fl    = sanitize_channel(df.SHOCKFL * gain);
    s_fr    = sanitize_channel(df.SHOCKFR * gain);
    s_rl    = sanitize_channel(df.SHOCKRL * gain);
    s_rr    = sanitize_channel(df.SHOCKRR * gain);
    acc_x   = sanitize_channel(df.ACCELX);
    acc_y   = sanitize_channel(df.ACCELY);
    steer   = sanitize_channel(df.STEERINGANGLE);
    
    v_ms = v_kmh / 3.6; % Convert to m/s
    
    % 2. FILTER SIGNALS (Zero-Phase Butterworth Filtering)
    s_fl_f  = filtfilt(b, a, s_fl);
    s_fr_f  = filtfilt(b, a, s_fr);
    s_rl_f  = filtfilt(b, a, s_rl);
    s_rr_f  = filtfilt(b, a, s_rr);
    accel_y = filtfilt(b, a, acc_y);
    accel_x = filtfilt(b, a, acc_x);
    
    % 3. FIND STATIC BASELINE (Car nearly stopped: v < 3 km/h)
    static_mask = v_kmh < 3;
    if ~any(static_mask)
        error('No static baseline found in log! Make sure dataset includes car standing still.');
    end
    
    s_fl_0 = mean(s_fl_f(static_mask));
    s_fr_0 = mean(s_fr_f(static_mask));
    s_rl_0 = mean(s_rl_f(static_mask));
    s_rr_0 = mean(s_rr_f(static_mask));
    
    % 4. CALCULATE RELATIVE DISPLACEMENTS (Delta from static)
    ds_fl = (s_fl_f - s_fl_0) * sign_conv;
    ds_fr = (s_fr_f - s_fr_0) * sign_conv;
    ds_rl = (s_rl_f - s_rl_0) * sign_conv;
    ds_rr = (s_rr_f - s_rr_0) * sign_conv;
    
    % 5. CONVERT DISPLACEMENTS TO VERTICAL AXLE FORCES (N)
    F_front = (ds_fl + ds_fr) * kw_f;
    F_rear  = (ds_rl + ds_rr) * kw_r;
    F_total = F_front + F_rear;
    
    % 6. MASK FOR CLEAN STRAIGHT-LINE DRIVING
    clean_mask = (abs(accel_y) < 0.2) & ...  % < ~0.2g lateral accel
                 (abs(accel_x) < 0.15) & ... % < ~0.15g longitudinal accel
                 (v_kmh > 20) & ...          % Speed above 20 km/h
                 (abs(steer) < 5);           % Steering angle near center
             
    v_clean = v_ms(clean_mask);
    F_clean = F_total(clean_mask);
    F_front_clean = F_front(clean_mask);
    
    if isempty(v_clean)
        error('No valid straight-line points matched the filter criteria. Try relaxing threshold values in clean_mask.');
    end
    
    % 7. DOWNFORCE FIT (Force = 0.5 * rho * ClA * v^2)
    q = 0.5 * rho * (v_clean.^2); % Dynamic pressure factor
    ClA = q \ F_clean;            % Least-squares fit for ClA
    
    % 8. CENTER OF PRESSURE (% Front)
    CoP_front = mean(F_front_clean ./ F_clean) * 100;
    
    % Save to output structure
    res.ClA = ClA;
    res.CoP_front = CoP_front;
    res.v_valid = v_clean;
    res.F_down_valid = F_clean;
end

%% LOCAL FUNCTION TO CLEAN NON-FINITE DATA FOR FILTFILT
function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end

%}