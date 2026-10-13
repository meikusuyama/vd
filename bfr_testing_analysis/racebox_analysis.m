%% RACEBOX WHEEL HORSEPOWER CALCULATOR
clear; clc; close all;

%% 1. FILE PATH & USER PARAMETERS
file_racebox = 'testing_data/Racebox-endurance1-05.04.2026.csv';

% Vehicle Parameters (Adjust total weight if needed)
m_total_kg = 230;      % Total mass of car + driver (kg) 162.8 + 67.2kg
CdA        = 1.35;     % Drag Area from Coastdown testing (m^2)
rho        = 1.225;    % Air density (kg/m^3)
C_rr       = 0.025;    % Tire rolling resistance coefficient (from coastdown)
g_acc      = 9.81;     % Gravity (m/s^2)

%% 2. LOAD RACEBOX DATA
df_rb = readtable(file_racebox);

t_rb     = sanitize_channel(df_rb.Time);
v_rb_ms  = sanitize_channel(df_racebox_or_rb(df_rb, 'Speed')); 
v_rb_mph = v_rb_ms * 2.23694;

g_lat_rb = sanitize_channel(df_rb.GForceY);

% Check for Longitudinal Acceleration channel (GForceX or calc from speed)
if ismember('GForceX', df_rb.Properties.VariableNames)
    g_long_rb = sanitize_channel(df_rb.GForceX);
else
    % Calculate acceleration derivative dv/dt if GForceX isn't present
    dt = mean(diff(t_rb));
    g_long_rb = [0; diff(v_rb_ms) / dt] / g_acc;
end

%% 3. FILTER ACCELERATION & DETECT STRAIGHTAWAY WOT PULLS
dt = mean(diff(t_rb));
if isnan(dt) || dt <= 0, dt = 0.01; end
fs = 1/dt;

% Smooth acceleration with 1.5 Hz low-pass filter
[b, a] = butter(2, 1.5 / (fs/2), 'low');
g_long_filt = filtfilt(b, a, double(g_long_rb));
g_lat_filt  = filtfilt(b, a, double(g_lat_rb));

% Straightaway Acceleration Mask:
% 1. Near zero cornering (|G_y| < 0.2g)
% 2. Active acceleration (G_x > 0.15g)
% 3. Rolling speed > 10 mph (4.47 m/s)
wot_mask = (abs(g_lat_filt) < 0.20) & (g_long_filt > 0.15) & (v_rb_ms > 4.47);

%% 4. CALCULATE HORSEPOWER COMPONENTS
% Force Components (N)
F_accel = m_total_kg * (g_long_filt * g_acc);              % Inertial Force (m*a)
F_drag  = 0.5 * rho * (v_rb_ms.^2) * CdA;                   % Aero Drag Force
F_rr    = C_rr * m_total_kg * g_acc;                        % Rolling Resistance

% Power Components (Watts)
P_accel_W = F_accel .* v_rb_ms;
P_drag_W  = F_drag .* v_rb_ms;
P_rr_W    = F_rr .* v_rb_ms;

P_total_W  = P_accel_W + P_drag_W + P_rr_W;
P_total_hp = P_total_W / 745.7; % Convert Watts to Horsepower (1 hp = 745.7 W)

% Zero-out non-WOT regions
P_total_hp(~wot_mask) = NaN;

%% 5. DISPLAY RESULTS & STATS
peak_hp = max(P_total_hp(wot_mask));
avg_wot_hp = mean(P_total_hp(wot_mask));

fprintf('=========================================================================================\n');
fprintf('                 ESTIMATED WHEEL HORSEPOWER FROM RACEBOX LOG                             \n');
fprintf('=========================================================================================\n');
fprintf('Assumed Total Mass: %d kg | CdA: %.2f m^2 | Air Density: %.3f kg/m^3\n', m_total_kg, CdA, rho);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('Peak Calculated Wheel Power:   %5.1f hp\n', peak_hp);
fprintf('Average Power during WOT:      %5.1f hp\n', avg_wot_hp);
fprintf('=========================================================================================\n\n');

%% 6. PLOT HORSEPOWER VS SPEED & TIMELINE
figure('Name', 'RaceBox Power Extraction', 'Position', [100, 100, 1100, 600]);

% Subplot 1: Power vs. Speed (Scatter/Curve)
subplot(1, 2, 1);
scatter(v_rb_mph(wot_mask), P_total_hp(wot_mask), 25, g_long_filt(wot_mask), 'filled');
colormap('jet'); c = colorbar; c.Label.String = 'Longitudinal Acceleration (g)';
xlabel('Speed (mph)'); ylabel('Calculated Wheel Horsepower (hp)');
title('1. Wheel Horsepower vs. Vehicle Speed'); grid on;

% Subplot 2: Power Timeline during Session
subplot(1, 2, 2);
plot(t_rb, v_rb_mph, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Speed (mph)'); hold on;
plot(t_rb, P_total_hp, 'r', 'LineWidth', 1.5, 'DisplayName', 'Calculated Wheel HP');
ylabel('Power (hp) / Speed (mph)'); xlabel('Time (seconds)');
title('2. Power Output Over Session Timeline');
grid on; legend('Location', 'northeast');

sgtitle('RaceBox GPS Wheel Power Extraction', 'FontSize', 13, 'FontWeight', 'bold');

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

%=================== Speed Alignment=================
%{
%% DATA LOGGER SPEED ALIGNMENT: RACEBOX vs CANARY DAQ
clear; clc; close all;

%% 1. FILE PATHS & CANARY SLICE WINDOW
file_racebox = 'testing_data/RaceBox_Track_Sessionon_02-05-2026_19-52.csv';
file_canary  = 'testing_data/Alameda_Skidpad0384LLTD_2026-05-02T20-21-47.csv';

% Time offset (Adjust this if you need to nudge by a few tenths of a second)
t_canary_offset = 1569.0; % seconds

%% 2. LOAD DATASETS
df_racebox = readtable(file_racebox);
df_canary  = readtable(file_canary);

%% 3. EXTRACT & SANITIZE CHANNELS
% RaceBox (Raw speed is m/s -> convert to km/h)
t_rb     = sanitize_channel(df_racebox.Time);
v_rb_ms  = sanitize_channel(df_racebox.Speed); 
v_rb_kmh = v_rb_ms * 3.6;

% Canary DAQ
t_can    = sanitize_channel(df_canary.Time);
v_fl_kmh = sanitize_channel(df_canary.WheelSpeedFrontLeft);
v_fr_kmh = sanitize_channel(df_canary.WheelSpeedFrontRight);

% Average front wheel speeds (km/h)
v_can_kmh = (v_fl_kmh + v_fr_kmh) / 2;

%% 4. ALIGN CANARY TIMESTAMPS
% Shift Canary time vector so 1569s maps to t = 0s
t_can_aligned = t_can - t_canary_offset;

%% 5. SINGLE-GRAPH OVERLAY PLOT
figure('Name', 'Overlay Speed Alignment', 'Position', [100, 100, 1100, 600]);

% Plot RaceBox
plot(t_rb, v_rb_kmh, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.8, ...
     'DisplayName', 'RaceBox GPS Speed (km/h)');
hold on;

% Plot Aligned Canary Wheel Speed
plot(t_can_aligned, v_can_kmh, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.2, ...
     'DisplayName', sprintf('Canary Wheel Speed (Shifted -%.1f s)', t_canary_offset));

xlabel('Synchronized Time (seconds)');
ylabel('Speed (km/h)');
title(sprintf('Speed Profile Direct Overlay (Canary Offset = -%.1f s)', t_canary_offset));
grid on; 
legend('Location', 'northeast');

% Focus view around the active window (0 to 450 seconds after offset)
xlim([0, 450]);

%% HELPER FUNCTION
function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end

%}



%=========================== Skidpad Extraction ==============

%{

%% SKIDPAD CIRCLE & LAP TIME EXTRACTOR (SPATIAL ANGULAR ROTATION)
clear; clc; close all;

%% 1. FILE PATHS & ALIGNMENT OFFSET
file_racebox = 'testing_data/RaceBox_Track_Sessionon_02-05-2026_19-52.csv';

%% 2. LOAD DATASET
df_rb = readtable(file_racebox);

t_rb     = sanitize_channel(df_rb.Time);
lat      = sanitize_channel(df_rb.Latitude);
lon      = sanitize_channel(df_rb.Longitude);
v_rb_ms  = sanitize_channel(df_rb.Speed); 
v_rb_kmh = v_rb_ms * 3.6;
g_lat    = sanitize_channel(df_rb.GForceY);

dt = mean(diff(t_rb));
if isnan(dt) || dt <= 0, dt = 0.01; end

%% 3. CONVERT LATITUDE / LONGITUDE TO LOCAL METERS (X, Y)
R_earth = 6371000; % Earth radius in meters

lat0 = mean(lat);
lon0 = mean(lon);

dLat = (lat - lat0) * (pi / 180);
dLon = (lon - lon0) * (pi / 180);

x_m = R_earth * dLon * cos(lat0 * pi / 180); 
y_m = R_earth * dLat;                       

%% 4. ISOLATE SKIDPAD CANDIDATE POINTS & DETECT CENTER
% Skidpad criteria: high lateral g (|g_lat| > 0.75g) and speed between 15 and 55 km/h
skid_mask = (abs(g_lat) > 0.75) & (v_rb_kmh > 15) & (v_rb_kmh < 55);

if ~any(skid_mask)
    error('No points matched skidpad criteria. Try lowering threshold to |g_lat| > 0.6g.');
end

% Approximate skidpad center coordinates (X_c, Y_c)
x_center = mean(x_m(skid_mask));
y_center = mean(y_m(skid_mask));

% Radial distance from center (meters)
radius = sqrt((x_m - x_center).^2 + (y_m - y_center).^2);

% Angle around center (radians)
angle_rad = atan2(y_m - y_center, x_m - x_center);

%% 5. TRACK CUMULATIVE HEADING & DETECT COMPLETE 360-DEGREE CIRCLES
% Filter contiguous skidpad window
skid_indices = find(skid_mask);

% Group into continuous runs
d_idx = [0; diff(skid_indices) > 1];
run_starts = skid_indices(d_idx == 1 | [1; zeros(length(skid_indices)-1, 1)]);
run_ends   = [skid_indices(d_idx(2:end) == 1); skid_indices(end)];

% Table to store completed circle results
circle_results = [];
circle_count = 0;

for r = 1:length(run_starts)
    idx_run = run_starts(r):run_ends(r);
    
    if length(idx_run) < (3.0 / dt) % Ignore short bursts (< 3 seconds)
        continue;
    end
    
    t_run = t_rb(idx_run);
    ang_unwrapped = unwrap(angle_rad(idx_run));
    ang_total = abs(ang_unwrapped - ang_unwrapped(1));
    
    % Check for full 2*pi (360 deg) revolutions
    num_revs = floor(max(ang_total) / (2 * pi));
    
    if num_revs >= 1
        % Determine rotation direction
        direction = 'RIGHT';
        if mean(diff(ang_unwrapped)) > 0, direction = 'LEFT'; end
        
        for rev = 1:num_revs
            target_ang = rev * (2 * pi);
            
            % Locate start and end index for this specific 360-degree circle
            if rev == 1
                idx_start = idx_run(1);
            else
                [~, rel_idx_start] = min(abs(ang_total - (rev - 1) * (2 * pi)));
                idx_start = idx_run(rel_idx_start);
            end
            
            [~, rel_idx_end] = min(abs(ang_total - target_ang));
            idx_end = idx_run(rel_idx_end);
            
            % Compute circle metrics
            t_start  = t_rb(idx_start);
            t_end    = t_rb(idx_end);
            lap_time = t_end - t_start;
            
            if lap_time > 3.0 && lap_time < 12.0 % Realistic skidpad lap time window
                circle_count = circle_count + 1;
                
                v_avg_kmh = mean(v_rb_kmh(idx_start:idx_end));
                v_avg_mph = v_avg_kmh / 1.60934;
                glat_max  = max(abs(g_lat(idx_start:idx_end)));
                glat_avg  = mean(abs(g_lat(idx_start:idx_end)));
                
                circle_results = [circle_results; ...
                    circle_count, t_start, t_end, lap_time, v_avg_kmh, v_avg_mph, glat_max, glat_avg, strcmp(direction, 'RIGHT')];
            end
        end
    end
end

%% 6. PRINT RESULTS TABLE
fprintf('=========================================================================================\n');
fprintf('                           DETAILED SKIDPAD CIRCLE BREAKDOWN                             \n');
fprintf('=========================================================================================\n');
fprintf('Circle # | Start (s) |  End (s)  | Lap Time (s) | Avg Speed (mph) | Peak LatG | Direction\n');
fprintf('-----------------------------------------------------------------------------------------\n');

for i = 1:size(circle_results, 1)
    dir_str = 'LEFT';
    if circle_results(i, 9) == 1, dir_str = 'RIGHT'; end
    
    fprintf('   %02d    |  %7.2f  |  %7.2f  |   %6.3f s    |    %5.1f mph    |   %4.2fg   |   %s\n', ...
        circle_results(i, 1), circle_results(i, 2), circle_results(i, 3), ...
        circle_results(i, 4), circle_results(i, 6), circle_results(i, 7), dir_str);
end
fprintf('=========================================================================================\n');
fprintf('TOTAL COMPLETED CIRCLES DETECTED: %d\n', circle_count);
fprintf('=========================================================================================\n');

%% 7. PLOT MAP WITH INDIVIDUAL HIGHLIGHTED CIRCLES
figure('Name', 'Skidpad Individual Circles', 'Position', [100, 100, 1100, 700]);

% Subplot 1: Spatial Map of Circles
subplot(1, 2, 1);
plot(x_m, y_m, 'Color', [0.8 0.8 0.8], 'DisplayName', 'Full Log Track'); hold on;
plot(x_center, y_center, 'k+', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Skidpad Center');

colors = lines(max(1, circle_count));
for i = 1:size(circle_results, 1)
    % Reconstruct indices for plotting
    t_s = circle_results(i, 2);
    t_e = circle_results(i, 3);
    idx_c = find(t_rb >= t_s & t_rb <= t_e);
    
    plot(x_m(idx_c), y_m(idx_c), 'LineWidth', 2.5, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Circle #%d (%.2fs)', i, circle_results(i, 4)));
end

axis equal; grid on;
xlabel('Position East (m)'); ylabel('Position North (m)');
title('1. Spatial Trajectory of Detected Skidpad Circles');
legend('Location', 'northeast');

% Subplot 2: Cumulative Rotation Angle Over Time
subplot(1, 2, 2);
plot(t_rb, g_lat, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw LatG'); hold on;

for i = 1:size(circle_results, 1)
    t_s = circle_results(i, 2);
    t_e = circle_results(i, 3);
    idx_c = find(t_rb >= t_s & t_rb <= t_e);
    
    plot(t_rb(idx_c), g_lat(idx_c), 'LineWidth', 2.0, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Circle #%d', i));
end

xlabel('Time (s)'); ylabel('Lateral Acceleration (g)');
title('2. Lateral Acceleration Profile per Circle');
grid on; legend('Location', 'northeast');

%% HELPER FUNCTION
function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end

%}