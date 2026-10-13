
%% FSAE CAN DOWNFORCE, DRAG, CLA, & COP COMPLETE ANALYSIS SCRIPT
clc; close all; clear;

%% 1. FILE PATHS, CONSTANTS & B26 WHEEL RATES
file_base    = 'testing_data/23_may3rd_log_2026-07-15T13-33-40.csv';
file_lowdrag = 'testing_data/24_may3rd_log_2026-07-15T13-16-00.csv';
rho = 1.225; % Air density (kg/m^3)

% B26 Spec Sheet Wheel Rates (N/mm per wheel)
K_wheel_front = 38.0; % N/mm (Front wheel rate)
K_wheel_rear  = 42.0; % N/mm (Rear wheel rate)

% Low-pass filter settings (0.5 Hz for noisy FL, 2.0 Hz for RL)
fs = 100;   
[b_fl, a_fl] = butter(2, 0.5 / (fs/2), 'low');
[b_rl, a_rl] = butter(2, 2.0 / (fs/2), 'low');

%% 2. HARDCODED TIME RANGES & VERIFIED STATIC ZERO WINDOWS
t_range_base      = [850, 1250];   
t_static_base     = [1094, 1096]; % Stationary 0 km/h baseline zero window

t_range_lowdrag   = [100, 620];    
t_static_lowdrag  = [335, 337];   % Stationary 0 km/h baseline zero window

%% 3. LOAD & PROCESS DATASETS
data_base    = readtable(file_base);
data_lowdrag = readtable(file_lowdrag);

base_res    = process_raw_bits(data_base, b_fl, a_fl, b_rl, a_rl, rho, t_range_base, t_static_base, K_wheel_front, K_wheel_rear);
lowdrag_res = process_raw_bits(data_lowdrag, b_fl, a_fl, b_rl, a_rl, rho, t_range_lowdrag, t_static_lowdrag, K_wheel_front, K_wheel_rear);

%% 4. CALCULATE PERCENTAGE DELTAS
percent_delta_total = ((lowdrag_res.K_total - base_res.K_total) / base_res.K_total) * 100;
percent_delta_FL    = ((lowdrag_res.K_FL - base_res.K_FL) / base_res.K_FL) * 100;
percent_delta_RL    = ((lowdrag_res.K_RL - base_res.K_RL) / base_res.K_RL) * 100;

% Drag Deceleration Delta
percent_delta_drag  = ((lowdrag_res.K_drag - base_res.K_drag) / base_res.K_drag) * 100;

%% 5. DISPLAY RESULTS
fprintf('===================================================\n');
fprintf('  RAW CAN INTEGER DOWNFORCE ANALYSIS (FL & RL)     \n');
fprintf('===================================================\n');
fprintf('Baseline Bit-Displacement Slope  : %.4f counts / (m/s)^2\n', base_res.K_total);
fprintf('Low Drag Bit-Displacement Slope  : %.4f counts / (m/s)^2\n', lowdrag_res.K_total);
fprintf('---------------------------------------------------\n');
fprintf('EXACT TOTAL DOWNFORCE REDUCTION  : %.2f%%\n', percent_delta_total);
fprintf('  * Front Left Axle Delta (FL)   : %.2f%%\n', percent_delta_FL);
fprintf('  * Rear Left Axle Delta (RL)    : %.2f%%\n', percent_delta_RL);
fprintf('===================================================\n');
fprintf('  COASTDOWN DRAG DECELERATION COMPARISON           \n');
fprintf('===================================================\n');
fprintf('Baseline Decel Drag Slope (K_d)  : %.6f g / (m/s)^2\n', base_res.K_drag);
fprintf('Low Drag Decel Drag Slope (K_d)  : %.6f g / (m/s)^2\n', lowdrag_res.K_drag);
fprintf('EXACT DRAG DECELERATION REDUCTION: %.2f%%\n', percent_delta_drag);
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

%% 6. DISPLAY TIME WINDOW VERIFICATION
fprintf('===================================================\n');
fprintf('  HARDCODED TIME WINDOW VERIFICATION               \n');
fprintf('===================================================\n');
fprintf('BASELINE LOG:\n');
fprintf('  * Analysis Range       : %.1f s to %.1f s\n', t_range_base(1), t_range_base(2));
fprintf('  * Static Baseline Range: %.1f s to %.1f s\n', t_static_base(1), t_static_base(2));
fprintf('  * Clean Coast Points   : %d samples\n', base_res.n_coast_points);
fprintf('---------------------------------------------------\n');
fprintf('LOW DRAG LOG:\n');
fprintf('  * Analysis Range       : %.1f s to %.1f s\n', t_range_lowdrag(1), t_range_lowdrag(2));
fprintf('  * Static Baseline Range: %.1f s to %.1f s\n', t_static_lowdrag(1), t_static_lowdrag(2));
fprintf('  * Clean Coast Points   : %d samples\n', lowdrag_res.n_coast_points);
fprintf('===================================================\n');

%% 7. FIGURE 1: PLOTTING PHYSICAL SHOCKPOT DISPLACEMENT VS AIRSPEED
figure('Name', 'Physical Travel Comparison (FL & RL)', 'Position', [100, 100, 1000, 500]);
v_fit = linspace(0, 22, 100); % 0 to ~80 km/h in m/s
q_fit = 0.5 * rho * (v_fit.^2); % Dynamic pressure vector

scatter(base_res.v_valid * 3.6, base_res.d_mm_clean_total, 14, [0 0.4470 0.7410], 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Baseline Data'); 
hold on;
scatter(lowdrag_res.v_valid * 3.6, lowdrag_res.d_mm_clean_total, 14, [0.8500 0.3250 0.0980], 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Low Drag Data');

plot(v_fit * 3.6, polyval(base_res.fit_mm_total, q_fit), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', sprintf('Baseline Fit (CLA = %.2f m^2)', base_res.CLA_total));
plot(v_fit * 3.6, polyval(lowdrag_res.fit_mm_total, q_fit), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5, 'DisplayName', sprintf('Low Drag Fit (CLA = %.2f m^2)', lowdrag_res.CLA_total));

xlabel('Speed (km/h)'); 
ylabel('Relative Suspension Compression (Physical mm)');
title('Aerodynamic Load Comparison (Physical Travel mm - FL & RL Combined)');
grid on; legend('Location', 'northwest');

xlim([0, 80]);

%% 8. FIGURE 2: FILTERED SHOCKPOT TIME HISTORY & WHEEL SPEED
figure('Name', 'Filtered Shockpot & Wheel Speed Time History', 'Position', [100, 50, 1200, 850]);

% --- BASELINE RUN ---
subplot(4,1,1);
plot(base_res.time, base_res.dFL_mm, 'b', 'LineWidth', 1.2, 'DisplayName', 'FL Travel (mm)'); hold on;
plot(base_res.time, base_res.dRL_mm, 'r', 'LineWidth', 1.2, 'DisplayName', 'RL Travel (mm)');
plot(base_res.time(base_res.clean_mask), base_res.dFL_mm(base_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'FL Used in Fit');
plot(base_res.time(base_res.clean_mask), base_res.dRL_mm(base_res.clean_mask), 'm.', 'MarkerSize', 5, 'DisplayName', 'RL Used in Fit');
xline(t_static_base(1), 'w--', 'LineWidth', 1.2, 'DisplayName', 'Static Baseline Window');
xline(t_static_base(2), 'w--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('Displacement (mm)');
title('Baseline Run: Physical Shockpot Travel Over Time');
xlim([t_range_base(1)-10, t_range_base(2)+10]);
grid on; legend('Location', 'northeast');

subplot(4,1,2);
plot(base_res.time, base_res.v_kmh, 'w', 'LineWidth', 1.2, 'DisplayName', 'Wheel Speed (Front Avg)'); hold on;
plot(base_res.time(base_res.clean_mask), base_res.v_kmh(base_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'Speed Used in Fit');
xline(t_static_base(1), 'w--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(t_static_base(2), 'w--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Speed (km/h)');
title('Baseline Run: Front Wheel Speed Over Time');
xlim([t_range_base(1)-10, t_range_base(2)+10]); ylim([0, 85]);
grid on; legend('Location', 'northeast');

% --- LOW DRAG RUN ---
subplot(4,1,3);
plot(lowdrag_res.time, lowdrag_res.dFL_mm, 'b', 'LineWidth', 1.2, 'DisplayName', 'FL Travel (mm)'); hold on;
plot(lowdrag_res.time, lowdrag_res.dRL_mm, 'r', 'LineWidth', 1.2, 'DisplayName', 'RL Travel (mm)');
plot(lowdrag_res.time(lowdrag_res.clean_mask), lowdrag_res.dFL_mm(lowdrag_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'FL Used in Fit');
plot(lowdrag_res.time(lowdrag_res.clean_mask), lowdrag_res.dRL_mm(lowdrag_res.clean_mask), 'm.', 'MarkerSize', 5, 'DisplayName', 'RL Used in Fit');
xline(t_static_lowdrag(1), 'w--', 'LineWidth', 1.2, 'DisplayName', 'Static Baseline Window');
xline(t_static_lowdrag(2), 'w--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('Displacement (mm)');
title('Low Drag Run: Physical Shockpot Travel Over Time');
xlim([t_range_lowdrag(1)-10, t_range_lowdrag(2)+10]);
grid on; legend('Location', 'northeast');

subplot(4,1,4);
plot(lowdrag_res.time, lowdrag_res.v_kmh, 'w', 'LineWidth', 1.2, 'DisplayName', 'Wheel Speed (Front Avg)'); hold on;
plot(lowdrag_res.time(lowdrag_res.clean_mask), lowdrag_res.v_kmh(lowdrag_res.clean_mask), 'g.', 'MarkerSize', 5, 'DisplayName', 'Speed Used in Fit');
xline(t_static_lowdrag(1), 'w--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(t_static_lowdrag(2), 'w--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Speed (km/h)');
title('Low Drag Run: Front Wheel Speed Over Time');
xlim([t_range_lowdrag(1)-10, t_range_lowdrag(2)+10]); ylim([0, 85]);
grid on; legend('Location', 'northeast');

sgtitle('Filtered Shockpot & Wheel Speed Telemetry (Physical Travel mm)', 'FontSize', 12, 'FontWeight', 'bold');

%% =========================================================================
%% HELPER FUNCTION: DATA PROCESSING, CLA, AND COP CALCULATION
%% =========================================================================
function res = process_raw_bits(df, b_fl, a_fl, b_rl, a_rl, rho, t_range, t_static, K_wf, K_wr)
    % 1. Extract and sanitize channels
    time  = sanitize_channel(df.Time);
    v_kmh = sanitize_channel((df.WheelSpeedFrontLeft + df.WheelSpeedFrontRight) / 2);
    tps   = sanitize_channel(df.ThrottlePosition);
    steer = sanitize_channel(df.STEERINGANGLE);
    
    % Reject CAN wheel speed spikes (>80 km/h)
    v_kmh(v_kmh > 80) = NaN;
    v_kmh = fillmissing(v_kmh, 'linear', 'EndValues', 'nearest');
    v_ms  = v_kmh / 3.6; % m/s
    
    % 2. Acceleration Derivative
    dt = mean(diff(time));
    if isnan(dt) || dt <= 0, dt = 0.01; end
    a_x_g = gradient(v_ms, dt) / 9.81; % Acceleration in g's
    
    % 3. Reverse DBC math to extract raw CAN integers (0 - 4095)
    raw_FL = round((sanitize_channel(df.SHOCKFL) - 74.620)  / 0.048991);
    raw_RL = round((sanitize_channel(df.SHOCKRL) - 178.200) / 0.019159);
    
    % 4. Low-pass filter channels
    raw_FL_f = filtfilt(b_fl, a_fl, double(raw_FL));
    raw_RL_f = filtfilt(b_rl, a_rl, double(raw_RL));
    a_x_g_f  = filtfilt(b_rl, a_rl, double(a_x_g));
    steer_f  = filtfilt(b_rl, a_rl, double(steer));
    
    steer_corrected = steer_f - (-15.0);
    
    % 5. Zero-Speed Static Reference Window
    static_mask = (time >= t_static(1)) & (time <= t_static(2));
    if ~any(static_mask)
        error('No data found in static zero window [%.1f s - %.1f s].', t_static(1), t_static(2));
    end
    
    raw_FL_0 = mean(raw_FL_f(static_mask));
    raw_RL_0 = mean(raw_RL_f(static_mask));
    
    % 6. Physical Displacement (mm) & Normalized Bit Displacement
    % FL DBC Gain = 0.048991 mm/bit / 3.5 correction factor
    % RL DBC Gain = 0.019159 mm/bit
    dFL_mm = (raw_FL_f - raw_FL_0) * (0.048991 / 3.5);
    dRL_mm = (raw_RL_f - raw_RL_0) * 0.019159;
    
    dbits_FL = (raw_FL_f - raw_FL_0) / 3.5;
    dbits_RL = raw_RL_f - raw_RL_0;
    dbits_total = (2 * dbits_FL) + (2 * dbits_RL);
    
    % 7. Active Coastdown Mask
    clean_mask = (time >= t_range(1)) & (time <= t_range(2)) & ...
                 (v_kmh > 25) & (v_kmh < 75) & ...          % Speed bounds
                 (tps < 3.0) & ...                          % Throttle off
                 (abs(steer_corrected) < 7.0) & ...         % Straight line
                 (a_x_g_f > -0.35) & (a_x_g_f < -0.03);     % Actively decelerating
             
    v_clean           = v_ms(clean_mask);
    dFL_clean_mm      = dFL_mm(clean_mask);
    dRL_clean_mm      = dRL_mm(clean_mask);
    dbits_clean_total = dbits_total(clean_mask);
    decel_clean       = -a_x_g_f(clean_mask); % Positive deceleration magnitude
    
    if isempty(v_clean)
        error('No valid coastdown points found in range [%.1f s - %.1f s].', t_range(1), t_range(2));
    end
    
    % 8. Dynamic Pressure & Polyfits
    q = 0.5 * rho * (v_clean.^2);
    
    % Fit Bit Slopes
    fit_total = polyfit(q, dbits_clean_total, 1);
    fit_FL    = polyfit(q, dbits_FL(clean_mask), 1);
    fit_RL    = polyfit(q, dbits_RL(clean_mask), 1);
    
    % Fit Physical Displacement Slopes (mm / (N/m^2))
    fit_mm_total = polyfit(q, (2*dFL_clean_mm + 2*dRL_clean_mm), 1);
    fit_mm_FL    = polyfit(q, dFL_clean_mm, 1);
    fit_mm_RL    = polyfit(q, dRL_clean_mm, 1);
    
    slope_mm_FL  = fit_mm_FL(1);
    slope_mm_RL  = fit_mm_RL(1);
    
    % Fit Drag Deceleration Slope
    fit_drag = polyfit(v_clean.^2, decel_clean, 1);
    
    % 9. Calculate CLA (m^2) and CoP (%)
    % Force slope per axle (N / (N/m^2)) = m^2
    CLA_front = 2 * (slope_mm_FL * K_wf); 
    CLA_rear  = 2 * (slope_mm_RL * K_wr);
    CLA_total = CLA_front + CLA_rear;
    
    CoP_front = (CLA_front / CLA_total) * 100;
    
    % Force @ 60 km/h (16.67 m/s)
    v_60_ms = 60 / 3.6;
    q_60    = 0.5 * rho * (v_60_ms^2);
    F_60kmh_N = CLA_total * q_60;
    
    % Package Outputs
    res.K_total           = fit_total(1);
    res.K_FL              = fit_FL(1);
    res.K_RL              = fit_RL(1);
    res.K_drag            = fit_drag(1);
    
    res.CLA_total         = CLA_total;
    res.CLA_front         = CLA_front;
    res.CLA_rear          = CLA_rear;
    res.CoP_front         = CoP_front;
    res.F_60kmh_N         = F_60kmh_N;
    
    res.fit_total         = fit_total;
    res.fit_mm_total      = fit_mm_total;
    res.v_valid           = v_clean;
    res.d_mm_clean_total  = 2*dFL_clean_mm + 2*dRL_clean_mm;
    res.n_coast_points    = length(v_clean);
    
    % Time-Series Plotting Signals
    res.time              = time;
    res.v_kmh             = v_kmh;
    res.dFL_mm            = dFL_mm;
    res.dRL_mm            = dRL_mm;
    res.clean_mask        = clean_mask;
end

function clean_vec = sanitize_channel(vec)
    clean_vec = vec(:); 
    clean_vec(isinf(clean_vec)) = NaN; 
    clean_vec = fillmissing(clean_vec, 'linear', 'EndValues', 'nearest');
end
