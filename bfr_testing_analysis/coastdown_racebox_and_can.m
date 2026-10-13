%% FSAE FUSED COASTDOWN ANALYSIS: HARDCODED ZERO WINDOWS & FILTERED DATA ONLY
clc; close all; clear;

%% 1. FILE PATHS, CONSTANTS & SUSPENSION PARAMETERS
% Baseline Configuration
file_racebox_base = 'testing_data/racebox_data/05-03-26Baseline_Coastdown_Part2of2.csv';
file_canary_base  = 'testing_data/23_may3rd_log_2026-07-15T13-33-40.csv';
time_offset_base  = 712.0; % t_can_aligned = t_can - 712s

% Low Drag Configuration
file_racebox_ld   = 'testing_data/racebox_data/05-03-26LowDragConfig_Coastdown_Part1of2.csv';
file_canary_ld   = 'testing_data/24_may3rd_log_2026-07-15T13-16-00.csv';
time_offset_ld   = 179.0; % t_can_aligned = t_can - 179s

% Physics & Vehicle Stiffness Parameters
rho           = 1.225; % Air density (kg/m^3)
K_wheel_front = 38.0;  % Front wheel rate (N/mm)
K_wheel_rear  = 42.0;  % Rear wheel rate (N/mm)

% Low-pass filter settings (0.5 Hz for FL, 2.0 Hz for RL & IMU)
fs = 100;   
[b_fl, a_fl] = butter(2, 0.5 / (fs/2), 'low');
[b_rl, a_rl] = butter(2, 2.0 / (fs/2), 'low');

%% 2. HARDCODED ANALYSIS TIME RANGES & STATIC ZERO WINDOWS (CANary Timeline)
t_range_base     = [850, 1250];   
t_static_base    = [1094, 1096]; % Hardcoded zero-speed baseline window

t_range_lowdrag  = [100, 620];    
t_static_lowdrag = [335, 337];   % Hardcoded zero-speed baseline window
max_rb_time_ld   = 500.0;        % End Low Drag CANary analysis at 500s RaceBox time

%% 3. LOAD DATASETS
opts_rb = detectImportOptions(file_racebox_base);
df_rb_base = readtable(file_racebox_base, opts_rb);
df_can_base = readtable(file_canary_base);

opts_rb_ld = detectImportOptions(file_racebox_ld);
df_rb_ld = readtable(file_racebox_ld, opts_rb_ld);
df_can_ld = readtable(file_canary_ld);

%% 4. RUN FUSED COASTDOWN ANALYSIS
base_res = process_fused_coastdown(df_rb_base, df_can_base, time_offset_base, ...
    b_fl, a_fl, b_rl, a_rl, rho, t_range_base, t_static_base, ...
    K_wheel_front, K_wheel_rear, Inf);

lowdrag_res = process_fused_coastdown(df_rb_ld, df_can_ld, time_offset_ld, ...
    b_fl, a_fl, b_rl, a_rl, rho, t_range_lowdrag, t_static_lowdrag, ...
    K_wheel_front, K_wheel_rear, max_rb_time_ld);

%% 5. CALCULATE PERCENTAGE DELTAS
percent_delta_FL   = ((lowdrag_res.slope_FL - base_res.slope_FL) / base_res.slope_FL) * 100;
percent_delta_RL   = ((lowdrag_res.slope_RL - base_res.slope_RL) / base_res.slope_RL) * 100;
percent_delta_CLA  = ((lowdrag_res.CLA_total - base_res.CLA_total) / base_res.CLA_total) * 100;
percent_delta_drag = ((lowdrag_res.K_drag - base_res.K_drag) / base_res.K_drag) * 100;

%% 6. PRINT RESULTS
fprintf('===================================================\n');
fprintf('  FUSED DOWNFORCE ANALYSIS (HARDCODED ZERO WINDOWS)\n');
fprintf('===================================================\n');
fprintf('Baseline Total CLA           : %.3f m^2\n', base_res.CLA_total);
fprintf('Low Drag Total CLA            : %.3f m^2\n', lowdrag_res.CLA_total);
fprintf('---------------------------------------------------\n');
fprintf('EXACT TOTAL DOWNFORCE CHANGE  : %.2f%%\n', percent_delta_CLA);
fprintf('  * Front Axle Delta (FL)    : %.2f%%\n', percent_delta_FL);
fprintf('  * Rear Axle Delta (RL)     : %.2f%%\n', percent_delta_RL);
fprintf('===================================================\n');
fprintf('  IMU DRAG DECELERATION COMPARISON (RACEBOX G_X)   \n');
fprintf('===================================================\n');
fprintf('Baseline Decel Drag Slope (K_d)  : %.6f g / (m/s)^2\n', base_res.K_drag);
fprintf('Low Drag Decel Drag Slope (K_d)  : %.6f g / (m/s)^2\n', lowdrag_res.K_drag);
fprintf('EXACT DRAG DECELERATION CHANGE   : %.2f%%\n', percent_delta_drag);
fprintf('===================================================\n');
fprintf('  ABSOLUTE DOWNFORCE (CLA) & COP AERO BALANCE      \n');
fprintf('===================================================\n');
fprintf('BASELINE CONFIGURATION:\n');
fprintf('  * Total CLA                : %.3f m^2\n', base_res.CLA_total);
fprintf('  * Front Axle CLA           : %.3f m^2\n', base_res.CLA_front);
fprintf('  * Rear Axle CLA            : %.3f m^2\n', base_res.CLA_rear);
fprintf('  * Downforce @ 60 km/h      : %.1f N (%.1f lbs)\n', base_res.F_60kmh_N, base_res.F_60kmh_N * 0.224809);
fprintf('  * Center of Pressure (CoP) : %.1f%% Front / %.1f%% Rear\n', base_res.CoP_front, 100 - base_res.CoP_front);
fprintf('---------------------------------------------------\n');
fprintf('LOW DRAG CONFIGURATION:\n');
fprintf('  * Total CLA                : %.3f m^2\n', lowdrag_res.CLA_total);
fprintf('  * Front Axle CLA           : %.3f m^2\n', lowdrag_res.CLA_front);
fprintf('  * Rear Axle CLA            : %.3f m^2\n', lowdrag_res.CLA_rear);
fprintf('  * Downforce @ 60 km/h      : %.1f N (%.1f lbs)\n', lowdrag_res.F_60kmh_N, lowdrag_res.F_60kmh_N * 0.224809);
if abs(lowdrag_res.CLA_total) > 0.05
    fprintf('  * Center of Pressure (CoP) : %.1f%% Front / %.1f%% Rear\n', lowdrag_res.CoP_front, 100 - lowdrag_res.CoP_front);
else
    fprintf('  * Center of Pressure (CoP) : Undefined (Total CLA near zero: %.3f m^2)\n', lowdrag_res.CLA_total);
end
fprintf('===================================================\n');

%% 7. FIGURE 1: PLOTTING PHYSICAL SHOCKPOT DISPLACEMENT VS GPS AIRSPEED
figure('Name', 'GPS-Correlated Physical Travel Comparison', 'Position', [100, 100, 1000, 500]);
v_fit = linspace(0, 22, 100); % 0 to ~80 km/h in m/s
q_fit = 0.5 * rho * (v_fit.^2);

scatter(base_res.v_gps_clean * 3.6, base_res.d_mm_clean_total, 14, [0 0.4470 0.7410], 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Baseline Data'); 
hold on;
scatter(lowdrag_res.v_gps_clean * 3.6, lowdrag_res.d_mm_clean_total, 14, [0.8500 0.3250 0.0980], 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Low Drag Data');

plot(v_fit * 3.6, polyval(base_res.fit_mm_total, q_fit), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', sprintf('Baseline Fit (CLA = %.2f m^2)', base_res.CLA_total));
plot(v_fit * 3.6, polyval(lowdrag_res.fit_mm_total, q_fit), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5, 'DisplayName', sprintf('Low Drag Fit (CLA = %.2f m^2)', lowdrag_res.CLA_total));

xlabel('GPS Speed (km/h)'); 
ylabel('Relative Suspension Compression');
title('Aerodynamic Load Comparison (GPS Speed vs. Suspension Travel)');
grid on; legend('Location', 'northwest'); xlim([0, 80]);

%% 8. FIGURE 2: FILTERED SHOCKPOT TIME HISTORY & SPEED OVERLAY
figure('Name', 'Fused Telemetry Time History', 'Position', [100, 50, 1200, 850]);

% Custom color definitions
col_fl_filtered = [0.00, 0.45, 0.95]; % Bright blue
col_rl_filtered = [0.90, 0.10, 0.10]; % Bright red
col_can_speed   = [0.55, 0.55, 0.55]; % Light slate gray (non-black)

% --- BASELINE RUN ---
subplot(4,1,1);
plot(base_res.t_aligned, base_res.dFL_mm, 'Color', col_fl_filtered, 'LineWidth', 1.5, 'DisplayName', 'FL Filtered (mm)'); hold on;
plot(base_res.t_aligned, base_res.dRL_mm, 'Color', col_rl_filtered, 'LineWidth', 1.5, 'DisplayName', 'RL Filtered (mm)');
plot(base_res.t_aligned(base_res.clean_mask), base_res.dFL_mm(base_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'FL Used in Fit');
plot(base_res.t_aligned(base_res.clean_mask), base_res.dRL_mm(base_res.clean_mask), 'm.', 'MarkerSize', 5, 'DisplayName', 'RL Used in Fit');
xline(t_static_base(1) - time_offset_base, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Static Baseline Window');
xline(t_static_base(2) - time_offset_base, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('Travel (mm)'); title('Baseline Run: Filtered Shockpot Travel');
xlim([t_range_base(1)-time_offset_base-10, t_range_base(2)-time_offset_base+10]); grid on; legend('Location', 'northeast');

subplot(4,1,2);
plot(base_res.t_aligned, base_res.v_gps_kmh, 'r', 'LineWidth', 1.4, 'DisplayName', 'RaceBox GPS Speed'); hold on;
plot(base_res.t_aligned, base_res.v_can_kmh, '--', 'Color', col_can_speed, 'LineWidth', 1.2, 'DisplayName', 'CAN Wheel Speed');
plot(base_res.t_aligned(base_res.clean_mask), base_res.v_gps_kmh(base_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'Speed Used in Fit');
xline(t_static_base(1) - time_offset_base, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(t_static_base(2) - time_offset_base, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('RaceBox Time (s)'); ylabel('Speed (km/h)'); title('Baseline Run: GPS vs Wheel Speed Synchronization');
xlim([t_range_base(1)-time_offset_base-10, t_range_base(2)-time_offset_base+10]); ylim([0, 85]); grid on; legend('Location', 'northeast');

% --- LOW DRAG RUN ---
subplot(4,1,3);
plot(lowdrag_res.t_aligned, lowdrag_res.dFL_mm, 'Color', col_fl_filtered, 'LineWidth', 1.5, 'DisplayName', 'FL Filtered (mm)'); hold on;
plot(lowdrag_res.t_aligned, lowdrag_res.dRL_mm, 'Color', col_rl_filtered, 'LineWidth', 1.5, 'DisplayName', 'RL Filtered (mm)');
plot(lowdrag_res.t_aligned(lowdrag_res.clean_mask), lowdrag_res.dFL_mm(lowdrag_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'FL Used in Fit');
plot(lowdrag_res.t_aligned(lowdrag_res.clean_mask), lowdrag_res.dRL_mm(lowdrag_res.clean_mask), 'm.', 'MarkerSize', 5, 'DisplayName', 'RL Used in Fit');
xline(t_static_lowdrag(1) - time_offset_ld, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Static Baseline Window');
xline(t_static_lowdrag(2) - time_offset_ld, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('Travel (mm)'); title('Low Drag Run: Filtered Shockpot Travel');
xlim([t_range_lowdrag(1)-time_offset_ld-10, max_rb_time_ld+10]); grid on; legend('Location', 'northeast');

subplot(4,1,4);
plot(lowdrag_res.t_aligned, lowdrag_res.v_gps_kmh, 'r', 'LineWidth', 1.4, 'DisplayName', 'RaceBox GPS Speed'); hold on;
plot(lowdrag_res.t_aligned, lowdrag_res.v_can_kmh, '--', 'Color', col_can_speed, 'LineWidth', 1.2, 'DisplayName', 'CAN Wheel Speed');
plot(lowdrag_res.t_aligned(lowdrag_res.clean_mask), lowdrag_res.v_gps_kmh(lowdrag_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'Speed Used in Fit');
xline(t_static_lowdrag(1) - time_offset_ld, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(t_static_lowdrag(2) - time_offset_ld, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(max_rb_time_ld, 'm--', 'LineWidth', 1.5, 'DisplayName', '500s RaceBox Cutoff');
xlabel('RaceBox Time (s)'); ylabel('Speed (km/h)'); title('Low Drag Run: GPS vs Wheel Speed Synchronization');
xlim([t_range_lowdrag(1)-time_offset_ld-10, max_rb_time_ld+10]); ylim([0, 85]); grid on; legend('Location', 'northeast');

sgtitle('Fused Telemetry: Filtered Shockpot Travel (FL Scaled Down 6.50x)', 'FontSize', 12, 'FontWeight', 'bold');

%% =========================================================================
%% HELPER FUNCTION: FUSED DATA PROCESSING WITH HARDCODED ZERO WINDOWS
%% =========================================================================
function res = process_fused_coastdown(df_rb, df_can, time_offset, b_fl, a_fl, b_rl, a_rl, rho, t_range_can, t_static_can, K_wf, K_wr, max_rb_time)
    % 1. Extract RaceBox Data
    t_rb     = sanitize_channel(df_rb.Time);
    v_rb_mph = sanitize_channel(df_rb.Speed);
    a_x_rb   = sanitize_channel(df_rb.GForceX); % Direct IMU reading in g's
    v_rb_ms  = v_rb_mph * 0.44704;              % Convert mph to m/s
    
    % 2. Extract CANary Data & Align Timeline
    t_can = sanitize_channel(df_can.Time);
    t_can_aligned = t_can - time_offset;        % Map CANary timeline to RaceBox clock
    
    v_fl = sanitize_channel(df_can.WheelSpeedFrontLeft);
    v_fr = sanitize_channel(df_can.WheelSpeedFrontRight);
    v_can_kmh = (v_fl + v_fr) / 2;
    v_can_kmh(v_can_kmh > 80) = NaN;
    v_can_kmh = fillmissing(v_can_kmh, 'linear', 'EndValues', 'nearest');
    
    tps   = sanitize_channel(df_can.ThrottlePosition);
    steer = sanitize_channel(df_can.STEERINGANGLE);
    
    % 3. Interpolate RaceBox GPS Speed & Acceleration onto CANary Timeline
    v_gps_ms  = interp1(t_rb, v_rb_ms, t_can_aligned, 'linear', 'extrap');
    v_gps_kmh = v_gps_ms * 3.6;
    a_x_imu_g = interp1(t_rb, a_x_rb, t_can_aligned, 'linear', 'extrap');
    
    % 4. DIRECT LOGGED SHOCKPOT CHANNELS
    mm_FL_raw = sanitize_channel(df_can.SHOCKFL);
    mm_RL_raw = sanitize_channel(df_can.SHOCKRL);
    
    % Low-pass filter channels
    mm_FL_f   = filtfilt(b_fl, a_fl, double(mm_FL_raw));
    mm_RL_f   = filtfilt(b_rl, a_rl, double(mm_RL_raw));
    steer_f   = filtfilt(b_rl, a_rl, double(steer));
    a_x_imu_f = filtfilt(b_rl, a_rl, double(a_x_imu_g));
    
    steer_corrected = steer_f - (-15.0);
    
    % 5. HARDCODED STATIC ZERO BASELINE CALCULATION
    static_mask = (t_can >= t_static_can(1)) & (t_can <= t_static_can(2));
    if ~any(static_mask)
        error('No data found in static zero window [%.1f s - %.1f s].', t_static_can(1), t_static_can(2));
    end
    
    mm_FL_0 = mean(mm_FL_f(static_mask));
    mm_RL_0 = mean(mm_RL_f(static_mask));
    
    % Scaling factor applied to FL to correct sensor over-reading
    fl_scale_factor = 3.50; 

    % Filtered Physical Displacement (mm) relative to hardcoded static zero
    dFL_mm = (mm_FL_f - mm_FL_0) / fl_scale_factor;
    dRL_mm = mm_RL_f - mm_RL_0;
    
    % 6. Active Coastdown Mask
    clean_mask = (t_can >= t_range_can(1)) & (t_can <= t_range_can(2)) & ...
                 (t_can_aligned <= max_rb_time) & ...       % Low Drag 500s RaceBox cutoff
                 (v_gps_kmh > 25) & (v_gps_kmh < 75) & ...  % GPS Speed bounds
                 (tps < 3.0) & ...                          % Throttle off
                 (abs(steer_corrected) < 7.0) & ...         % Straight line
                 (a_x_imu_f < -0.03) & (a_x_imu_f > -0.35);% Deceleration window from IMU
             
    v_clean_ms    = v_gps_ms(clean_mask);
    dFL_clean_mm  = dFL_mm(clean_mask);
    dRL_clean_mm  = dRL_mm(clean_mask);
    decel_clean_g = -a_x_imu_f(clean_mask); % Positive deceleration magnitude
    
    if isempty(v_clean_ms)
        error('No valid fused coastdown points found.');
    end
    
    % 7. Dynamic Pressure & Polyfits against GPS Speed
    q_gps = 0.5 * rho * (v_clean_ms.^2);
    
    fit_mm_total = polyfit(q_gps, (2*dFL_clean_mm + 2*dRL_clean_mm), 1);
    fit_mm_FL    = polyfit(q_gps, dFL_clean_mm, 1);
    fit_mm_RL    = polyfit(q_gps, dRL_clean_mm, 1);
    
    slope_mm_FL  = fit_mm_FL(1);
    slope_mm_RL  = fit_mm_RL(1);
    
    % Fit Drag Deceleration Slope against GPS Speed Squared
    fit_drag = polyfit(v_clean_ms.^2, decel_clean_g, 1);
    
    % 8. CLA (m^2) and CoP (%)
    CLA_front = 2 * (slope_mm_FL * K_wf); 
    CLA_rear  = 2 * (slope_mm_RL * K_wr);
    CLA_total = CLA_front + CLA_rear;
    
    CoP_front = (CLA_front / CLA_total) * 100;
    
    v_60_ms = 60 / 3.6;
    q_60    = 0.5 * rho * (v_60_ms^2);
    F_60kmh_N = CLA_total * q_60;
    
    % Package Outputs
    res.slope_FL          = slope_mm_FL;
    res.slope_RL          = slope_mm_RL;
    res.K_drag            = fit_drag(1);
    
    res.CLA_total         = CLA_total;
    res.CLA_front         = CLA_front;
    res.CLA_rear          = CLA_rear;
    res.CoP_front         = CoP_front;
    res.F_60kmh_N         = F_60kmh_N;
    
    res.fit_mm_total      = fit_mm_total;
    res.v_gps_clean       = v_clean_ms;
    res.d_mm_clean_total  = 2*dFL_clean_mm + 2*dRL_clean_mm;
    
    res.t_aligned         = t_can_aligned;
    res.v_gps_kmh         = v_gps_kmh;
    res.v_can_kmh         = v_can_kmh;
    
    res.dFL_mm            = dFL_mm;
    res.dRL_mm            = dRL_mm;
    res.clean_mask        = clean_mask;
end

function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end