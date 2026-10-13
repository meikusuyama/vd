%% STEPS 1, 2, & 3: AERO HEAVE ISOLATION & SKIDPAD GRIP CORRELATION
clear; clc; close all;

%% 1. FILE PATHS, ALIGNMENT OFFSET, & STATIC ZERO REFERENCES
file_racebox    = 'testing_data/RaceBox_Track_Sessionon_02-05-2026_19-52.csv';
file_canary     = 'testing_data/Alameda_Skidpad0384LLTD_2026-05-02T20-21-47.csv';

t_canary_offset = 1569.0; % Add 1569s offset to map Canary to RaceBox timeline

% Hardcoded Static Zero References (User Specified)
FL_0_mm = 151.0; % mm
RL_0_mm = 218.5; % mm

%% 2. HARDCODED CIRCLE DEFINITIONS (RaceBox Timestamps)
% [Circle#, Start_s, End_s, Speed_mph, Peak_g, Is_Right (1=Right, 0=Left)]
circles_def = [
    1,   148.36,  155.28,  22.8,  1.07,  0;
    2,   155.28,  161.60,  24.8,  1.32,  0;
    3,   161.60,  167.64,  25.9,  1.38,  0;
    4,   189.72,  197.28,  21.2,  0.96,  1;
    5,   197.28,  204.32,  22.6,  1.13,  1;
    6,   204.32,  210.84,  25.1,  1.46,  1;
    7,   217.16,  223.40,  25.6,  1.55,  1;
    8,   281.68,  288.72,  22.3,  1.01,  0;
    9,   288.72,  295.04,  24.7,  1.29,  0;
    10,  295.04,  301.04,  26.9,  1.52,  0;
    11,  343.68,  351.20,  21.3,  1.04,  1;
    12,  351.20,  358.12,  23.4,  1.26,  1;
    13,  358.12,  364.28,  26.0,  1.51,  1;
    14,  364.28,  370.24,  27.4,  1.62,  1;
    15,  370.24,  376.16,  27.6,  1.57,  1
];

%% 3. LOAD DATASETS & CONVERT CHANNELS
df_rb  = readtable(file_racebox);
df_can = readtable(file_canary);

% RaceBox Channels
t_rb     = sanitize_channel(df_rb.Time);
v_rb_ms  = sanitize_channel(df_racebox_or_rb(df_rb, 'Speed')); 
v_rb_mph = v_rb_ms * 2.23694;
g_lat_rb = sanitize_channel(df_rb.GForceY);

% Canary Channels
t_can     = sanitize_channel(df_can.Time);
t_can_alg = t_can - t_canary_offset; % Synchronized RaceBox timeline

shock_FL_raw = sanitize_channel(df_can.SHOCKFL);
shock_RL_raw = sanitize_channel(df_can.SHOCKRL);

% Apply low-pass filtering (0.5 Hz for noisy FL, 2.0 Hz for RL)
dt = mean(diff(t_can));
if isnan(dt) || dt <= 0, dt = 0.01; end
fs = 1/dt;

[b_fl, a_fl] = butter(2, 0.5 / (fs/2), 'low'); % Smoother 0.5 Hz cutoff for noisy FL
[b_rl, a_rl] = butter(2, 2.0 / (fs/2), 'low'); % Standard 2.0 Hz cutoff for RL

shock_FL = filtfilt(b_fl, a_fl, double(shock_FL_raw));
shock_RL = filtfilt(b_rl, a_rl, double(shock_RL_raw));

% STEP 1: CONVERT TO RELATIVE DISPLACEMENT (Standard: Compression = +, Extension = -)
dFL_mm = -(shock_FL - FL_0_mm) / 2.5; % Inverted & rescaled by 2.5x to fix DBC multiplier
dRL_mm = shock_RL - RL_0_mm;         % Kept as compression is positive (+)

%% 4. EXTRACT SHOCKPOT DISPLACEMENT PER CIRCLE
num_circles = size(circles_def, 1);
results = zeros(num_circles, 8); % [Circle#, Speed_mph, Speed_ms, PeakG, Avg_dFL_mm, Avg_dRL_mm, Is_Right, LapTime]

for i = 1:num_circles
    c_num    = circles_def(i, 1);
    t_start  = circles_def(i, 2);
    t_end    = circles_def(i, 3);
    spd_mph  = circles_def(i, 4);
    peak_g   = circles_def(i, 5);
    is_right = circles_def(i, 6);
    
    rb_mask  = (t_rb >= t_start) & (t_rb <= t_end);
    can_mask = (t_can_alg >= t_start) & (t_can_alg <= t_end);
    
    spd_ms   = mean(v_rb_ms(rb_mask));
    avg_dFL  = mean(dFL_mm(can_mask));
    avg_dRL  = mean(dRL_mm(can_mask));
    lap_time = t_end - t_start;
    
    results(i, :) = [c_num, spd_mph, spd_ms, peak_g, avg_dFL, avg_dRL, is_right, lap_time];
end

%% 5. STEP 2: ISOLATE PURE AERO HEAVE FROM ROLL (MIDPOINT METHOD)
% Group circles into Low, Med, High speed buckets
low_mask  = results(:, 2) <= 23.5;
med_mask  = results(:, 2) > 23.5 & results(:, 2) <= 25.5;
high_mask = results(:, 2) > 25.5;

calc_heave = @(sub) [ ...
    mean(sub(:, 2)), ...                                    % Speed (mph)
    mean(sub(:, 3)), ...                                    % Speed (m/s)
    (mean(sub(sub(:,7)==1, 5)) + mean(sub(sub(:,7)==0, 5))) / 2, ... % Pure Heave FL (mm)
    (mean(sub(sub(:,7)==1, 6)) + mean(sub(sub(:,7)==0, 6))) / 2, ... % Pure Heave RL (mm)
    mean(sub(:, 4)) ...                                     % Peak LatG
];

heave_low  = calc_heave(results(low_mask, :));
heave_med  = calc_heave(results(med_mask, :));
heave_high = calc_heave(results(high_mask, :));

heave_table = [heave_low; heave_med; heave_high];

% Fit Heave vs v^2 allowing for a Y-intercept (polyfit)
v_sq = heave_table(:, 2).^2; % (m/s)^2

fit_FL = polyfit(v_sq, heave_table(:, 3), 1); 
fit_RL = polyfit(v_sq, heave_table(:, 4), 1);

C_heave_FL = fit_FL(1); % Slope in mm / (m/s)^2
C_heave_RL = fit_RL(1);

%% 6. PRINT ANALYSIS TABLES
fprintf('=========================================================================================\n');
fprintf('         STEP 1: RELATIVE SHOCKPOT TRAVEL PER SKIDPAD CIRCLE (FL_0=151mm, RL_0=218.5mm)   \n');
fprintf('=========================================================================================\n');
fprintf('Circle # | Speed (mph) | Peak LatG | Direction |  dFL Travel (mm)  |  dRL Travel (mm)  \n');
fprintf('-----------------------------------------------------------------------------------------\n');

for i = 1:num_circles
    dir_str = 'LEFT ';
    if results(i, 7) == 1, dir_str = 'RIGHT'; end
    
    fprintf('   %02d    |   %5.1f mph |   %4.2fg   |   %s   |    %7.2f mm    |    %7.2f mm\n', ...
        results(i, 1), results(i, 2), results(i, 4), dir_str, results(i, 5), results(i, 6));
end
fprintf('=========================================================================================\n\n');

fprintf('=========================================================================================\n');
fprintf('         STEP 2: ISOLATED PURE AERODYNAMIC HEAVE (SQUAT) VS SPEED                         \n');
fprintf('=========================================================================================\n');
fprintf('Speed Range | Avg Speed | Peak LatG | Pure FL Heave (mm) | Pure RL Heave (mm) | Total Heave\n');
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('Low Speed   |  %4.1f mph |   %4.2fg   |    %7.2f mm    |    %7.2f mm    |   %7.2f mm\n', ...
        heave_low(1), heave_low(5), heave_low(3), heave_low(4), heave_low(3)+heave_low(4));
fprintf('Med Speed   |  %4.1f mph |   %4.2fg   |    %7.2f mm    |    %7.2f mm    |   %7.2f mm\n', ...
        heave_med(1), heave_med(5), heave_med(3), heave_med(4), heave_med(3)+heave_med(4));
fprintf('High Speed  |  %4.1f mph |   %4.2fg   |    %7.2f mm    |    %7.2f mm    |   %7.2f mm\n', ...
        heave_high(1), heave_high(5), heave_high(3), heave_high(4), heave_high(3)+heave_high(4));
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('FITTED AERO HEAVE SLOPES: FL = %+.5f mm/(m/s)^2 | RL = %+.5f mm/(m/s)^2\n', C_heave_FL, C_heave_RL);
fprintf('=========================================================================================\n\n');

%% 7. FIGURE 1: RELATIVE SUSPENSION TRAVEL OVER TIME
figure('Name', 'Relative Suspension Travel over Time', 'Position', [100, 100, 1100, 750]);

subplot(3,1,1);
plot(t_rb, g_lat_rb, 'w', 'LineWidth', 1.2);
ylabel('Lateral g'); title('1. RaceBox Lateral Acceleration'); grid on; xlim([130, 390]);

subplot(3,1,2);
plot(t_can_alg, dFL_mm, 'b', 'LineWidth', 1.2);
yline(0, 'w--', 'FL Static Baseline (151mm)');
ylabel('dFL (mm)'); title('2. Relative FL Suspension Travel (Compression = +, Extension = -)'); grid on; xlim([130, 390]);

subplot(3,1,3);
plot(t_can_alg, dRL_mm, 'r', 'LineWidth', 1.2);
yline(0, 'w--', 'RL Static Baseline (218.5mm)');
ylabel('dRL (mm)'); xlabel('Synchronized Time (s)'); title('3. Relative RL Suspension Travel (Compression = +, Extension = -)'); grid on; xlim([130, 390]);

sgtitle('Step 1: Relative Suspension Displacement Timeline', 'FontSize', 13, 'FontWeight', 'bold');

%% 8. FIGURE 2: STEP 2 - PURE AERO HEAVE DISPLACEMENT VS DYNAMIC PRESSURE
figure('Name', 'Aero Heave Compression vs v^2', 'Position', [150, 150, 1000, 500]);

v_fit = linspace(0, max(heave_table(:, 2))*1.1, 50); % m/s vector
v_fit_sq = v_fit.^2;

plot(heave_table(:, 2).^2, heave_table(:, 3), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'FL Measured Heave'); hold on;
plot(heave_table(:, 2).^2, heave_table(:, 4), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'RL Measured Heave');

% Use polyval to plot proper fit lines including the intercept
plot(v_fit_sq, polyval(fit_FL, v_fit_sq), 'b--', 'LineWidth', 1.8, 'DisplayName', sprintf('FL Fit (Slope: %+.4f mm/(m/s)^2)', C_heave_FL));
plot(v_fit_sq, polyval(fit_RL, v_fit_sq), 'r--', 'LineWidth', 1.8, 'DisplayName', sprintf('RL Fit (Slope: %+.4f mm/(m/s)^2)', C_heave_RL));

xlabel('Dynamic Pressure Proxy: Velocity Squared [v^2 in (m/s)^2]');
ylabel('Pure Aero Heave / Vertical Squat [mm] (+ = Compression)');
title('Step 2: Pure Aerodynamic Heave (Vertical Squat) vs. Speed Squared');
grid on; legend('Location', 'northwest');

%% 9. FIGURE 3: STEP 3 - CORRELATION OF HEAVE WITH SKIDPAD GRIP & LAP TIME
figure('Name', 'Downforce Grip & Lap Time Benefit', 'Position', [200, 200, 1100, 500]);

subplot(1,2,1);
scatter(results(:, 2), results(:, 4), 60, results(:, 5)+results(:, 6), 'filled');
colormap('jet'); c = colorbar; c.Label.String = 'Combined Heave Compression (mm)';
xlabel('Skidpad Speed (mph)'); ylabel('Peak Lateral Acceleration (g)');
title('1. Peak Grip (G_y) vs. Skidpad Speed'); grid on;

subplot(1,2,2);
scatter(results(:, 4), results(:, 8), 60, results(:, 2), 'filled');
c2 = colorbar; c2.Label.String = 'Speed (mph)';
xlabel('Peak Lateral Acceleration (g)'); ylabel('Skidpad Lap Time (seconds)');
title('2. Skidpad Lap Time vs. Lateral Grip (G_y)'); grid on;

sgtitle('Step 3: Quantifying Cornering Grip & Lap Time Sensitivity', 'FontSize', 13, 'FontWeight', 'bold');

%% HELPER FUNCTIONS
function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end

function vec = df_racebox_or_rb(df, colname)
    if ismember(colname, df.Properties.VariableNames)
        vec = df.(colname);
    else
        vec = df.Speed;
    end
end


% ====================== Find operational zero point of shockpot
% =================
%{
%% SYMMETRIC CORNERING MIDPOINT & OPERATIONAL ZERO CALCULATOR
clear; clc; close all;

%% 1. FILE PATH & ALIGNMENT OFFSET
file_canary     = 'testing_data/Alameda_Skidpad0384LLTD_2026-05-02T20-21-47.csv';
t_canary_offset = 1569.0; % Add 1569s offset to map Canary to RaceBox time

%% 2. HARDCODED CIRCLE DEFINITIONS (RaceBox Timestamps)
% [Circle#, Start_s, End_s, Speed_mph, Peak_g, Is_Right (1=Right, 0=Left)]
circles_def = [
    1,   148.36,  155.28,  22.8,  1.07,  0;
    2,   155.28,  161.60,  24.8,  1.32,  0;
    3,   161.60,  167.64,  25.9,  1.38,  0;
    4,   189.72,  197.28,  21.2,  0.96,  1;
    5,   197.28,  204.32,  22.6,  1.13,  1;
    6,   204.32,  210.84,  25.1,  1.46,  1;
    7,   217.16,  223.40,  25.6,  1.55,  1;
    8,   281.68,  288.72,  22.3,  1.01,  0;
    9,   288.72,  295.04,  24.7,  1.29,  0;
    10,  295.04,  301.04,  26.9,  1.52,  0;
    11,  343.68,  351.20,  21.3,  1.04,  1;
    12,  351.20,  358.12,  23.4,  1.26,  1;
    13,  358.12,  364.28,  26.0,  1.51,  1;
    14,  364.28,  370.24,  27.4,  1.62,  1;
    15,  370.24,  376.16,  27.6,  1.57,  1
];

%% 3. LOAD DATASET & CONVERT TIMELINE
df_can = readtable(file_canary);

t_can     = sanitize_channel(df_can.Time);
t_can_alg = t_can - t_canary_offset; % Synchronized RaceBox timeline

shock_FL_raw = sanitize_channel(df_can.SHOCKFL);
shock_RL_raw = sanitize_channel(df_can.SHOCKRL);

% Apply 2 Hz low-pass filter to smooth out high-frequency track noise
dt = mean(diff(t_can));
if isnan(dt) || dt <= 0, dt = 0.01; end
fs = 1/dt;
[b, a] = butter(2, 2.0 / (fs/2), 'low');

shock_FL = filtfilt(b, a, double(shock_FL_raw));
shock_RL = filtfilt(b, a, double(shock_RL_raw));

%% 4. PROCESS AVERAGE SHOCKPOT VALUES PER CIRCLE
num_circles = size(circles_def, 1);
results = zeros(num_circles, 7); % [Circle#, Speed_mph, LatG, FL_avg, RL_avg, Is_Right, LapTime]

for i = 1:num_circles
    c_num    = circles_def(i, 1);
    t_start  = circles_def(i, 2);
    t_end    = circles_def(i, 3);
    spd_mph  = circles_def(i, 4);
    peak_g   = circles_def(i, 5);
    is_right = circles_def(i, 6);
    
    mask = (t_can_alg >= t_start) & (t_can_alg <= t_end);
    
    avg_FL = mean(shock_FL(mask));
    avg_RL = mean(shock_RL(mask));
    
    results(i, :) = [c_num, spd_mph, peak_g, avg_FL, avg_RL, is_right, (t_end - t_start)];
end

%% 5. DISPLAY ALL 15 CIRCLES WITH SHOCKPOT MEASUREMENTS
fprintf('=========================================================================================\n');
fprintf('                  INDIVIDUAL SKIDPAD CIRCLE SHOCKPOT MEASUREMENTS (mm)                   \n');
fprintf('=========================================================================================\n');
fprintf('Circle # | Speed (mph) | Peak LatG | Direction | FL Shockpot (mm) | RL Shockpot (mm)\n');
fprintf('-----------------------------------------------------------------------------------------\n');

for i = 1:num_circles
    dir_str = 'LEFT ';
    if results(i, 6) == 1, dir_str = 'RIGHT'; end
    
    fprintf('   %02d    |   %5.1f mph |   %4.2fg   |   %s   |    %7.2f mm    |    %7.2f mm\n', ...
        results(i, 1), results(i, 2), results(i, 3), dir_str, results(i, 4), results(i, 5));
end
fprintf('=========================================================================================\n\n');

%% 6. PAIR CIRCLES BY SPEED BUCKETS & COMPUTE SYMMETRIC MIDPOINTS
% Bucket 1: Low Speed  (21.0 - 23.5 mph)
% Bucket 2: Med Speed  (24.0 - 25.5 mph)
% Bucket 3: High Speed (25.6 - 27.6 mph)

low_left   = results(results(:, 6) == 0 & results(:, 2) <= 23.5, :);
low_right  = results(results(:, 6) == 1 & results(:, 2) <= 23.5, :);

med_left   = results(results(:, 6) == 0 & results(:, 2) > 23.5 & results(:, 2) <= 25.5, :);
med_right  = results(results(:, 6) == 1 & results(:, 2) > 23.5 & results(:, 2) <= 25.5, :);

high_left  = results(results(:, 6) == 0 & results(:, 2) > 25.5, :);
high_right = results(results(:, 6) == 1 & results(:, 2) > 25.5, :);

% Compute Averages for Buckets
calc_bucket = @(L, R) [
    mean(L(:, 2)), mean(R(:, 2)), ... % Speed L, R
    mean(L(:, 4)), mean(R(:, 4)), ... % FL L, R
    mean(L(:, 5)), mean(R(:, 5))      % RL L, R
];

b_low  = calc_bucket(low_left, low_right);
b_med  = calc_bucket(med_left, med_right);
b_high = calc_bucket(high_left, high_right);

% Calculate Midpoints: (Right + Left) / 2
FL_mid_low  = (b_low(4)  + b_low(3))  / 2;
RL_mid_low  = (b_low(6)  + b_low(5))  / 2;

FL_mid_med  = (b_med(4)  + b_med(3))  / 2;
RL_mid_med  = (b_med(6)  + b_med(5))  / 2;

FL_mid_high = (b_high(4) + b_high(3)) / 2;
RL_mid_high = (b_high(6) + b_high(5)) / 2;

% Calculate Pure Roll Component: (Right - Left) / 2
FL_roll_low  = (b_low(4)  - b_low(3))  / 2;
RL_roll_low  = (b_low(6)  - b_low(5))  / 2;

FL_roll_high = (b_high(4) - b_high(3)) / 2;
RL_roll_high = (b_high(6) - b_high(5)) / 2;

%% 7. PRINT SYMMETRIC MIDPOINT BREAKDOWN TABLE
fprintf('=========================================================================================\n');
fprintf('                 SYMMETRIC CORNERING MIDPOINT ANALYSIS (OPERATIONAL ZERO)                 \n');
fprintf('=========================================================================================\n');
fprintf('Speed Range | Avg Speed | FL Left (In) | FL Right (Out) | FL MIDPOINT | Pure Roll FL Delta\n');
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('Low Speed   |  %4.1f mph |   %6.2f mm  |   %6.2f mm    |  %6.2f mm  |    +/- %5.2f mm\n', ...
        mean([b_low(1), b_low(2)]), b_low(3), b_low(4), FL_mid_low, FL_roll_low);
fprintf('Med Speed   |  %4.1f mph |   %6.2f mm  |   %6.2f mm    |  %6.2f mm  |    +/- %5.2f mm\n', ...
        mean([b_med(1), b_med(2)]), b_med(3), b_med(4), FL_mid_med, (b_med(4)-b_med(3))/2);
fprintf('High Speed  |  %4.1f mph |   %6.2f mm  |   %6.2f mm    |  %6.2f mm  |    +/- %5.2f mm\n', ...
        mean([b_high(1), b_high(2)]), b_high(3), b_high(4), FL_mid_high, FL_roll_high);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('REAR LEFT (RL) MIDPOINTS -> Low: %.2f mm | Med: %.2f mm | High: %.2f mm\n', ...
        RL_mid_low, RL_mid_med, RL_mid_high);
fprintf('=========================================================================================\n');

%% 8. PLOT MIDPOINT SHIFT VS SPEED
figure('Name', 'Shockpot Midpoint Shift Analysis', 'Color', 'w', 'Position', [100, 100, 1000, 600]);

% Subplot 1: FL Shockpot Left vs Right & Midpoint Line
subplot(2,1,1);
plot(results(results(:,6)==0, 2), results(results(:,6)==0, 4), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'FL Left Turn (Inside)'); hold on;
plot(results(results(:,6)==1, 2), results(results(:,6)==1, 4), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'FL Right Turn (Outside)');
plot([mean([b_low(1), b_low(2)]), mean([b_med(1), b_med(2)]), mean([b_high(1), b_high(2)])], ...
     [FL_mid_low, FL_mid_med, FL_mid_high], 'k--s', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'FL Operational Zero (Midpoint)');

ylabel('FL Shockpot (mm)'); xlabel('Speed (mph)');
title('1. Front Left (FL) Shockpot Symmetric Midpoints vs. Speed');
grid on; legend('Location', 'west');

% Subplot 2: RL Shockpot Left vs Right & Midpoint Line
subplot(2,1,2);
plot(results(results(:,6)==0, 2), results(results(:,6)==0, 5), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'RL Left Turn (Inside)'); hold on;
plot(results(results(:,6)==1, 2), results(results(:,6)==1, 5), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'RL Right Turn (Outside)');
plot([mean([b_low(1), b_low(2)]), mean([b_med(1), b_med(2)]), mean([b_high(1), b_high(2)])], ...
     [RL_mid_low, RL_mid_med, RL_mid_high], 'k--s', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'RL Operational Zero (Midpoint)');

ylabel('RL Shockpot (mm)'); xlabel('Speed (mph)');
title('2. Rear Left (RL) Shockpot Symmetric Midpoints vs. Speed');
grid on; legend('Location', 'west');

sgtitle('Symmetric Midpoint Method: Operational Zero & Pure Roll Isolation', 'FontSize', 13, 'FontWeight', 'bold');

%% HELPER FUNCTION
function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end

%}