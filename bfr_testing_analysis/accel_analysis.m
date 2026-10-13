%% FSAE TELEMETRY: COMPLETE AERO ACCEL ANALYSIS & DOCUMENTATION
clc; close all; clear;

% 1. DEFINE FILE PATHS 
file_on  = 'testing_data/26-10-4-alameda_testing/2026-10-04_alameda_accel_run1_aero_on.csv';
file_off = 'testing_data/26-10-4-alameda_testing/2026-10-04_alameda_accel_run2_aero_off.csv';

% 2. DEFINE VALID RUN WINDOWS (Widened by -1s start, +1s end)
windows_on  = [326, 333; 
               358, 364];
              
windows_off = [185, 191; 
               244, 251; 
               315, 323];

% 3. DEFINE SHOCKPOT SCALING FACTORS (Derived from nearest Skidpad runs)
% Aero ON (e.g., from Skidpad Run 3)
cal_on.FL_scale = 0.243; 
cal_on.RL_scale = 4.111;

% Aero OFF (e.g., from Skidpad Run 5)
cal_off.FL_scale = 0.284;
cal_off.RL_scale = 3.936;

% Note: Manual static shifts (e.g., -3.3, +2.0) are intentionally excluded here 
% because the helper function now dynamically zeroes the sensors at the start 
% of each acceleration file, automatically aligning all traces to 0 mm.

% 4. EXTRACT, CALIBRATE, AND SEPARATE DATA
[runs_on, aero_on]   = process_and_calibrate(file_on, windows_on, cal_on);
[runs_off, aero_off] = process_and_calibrate(file_off, windows_off, cal_off);

%% 5. DOCUMENTATION DASHBOARDS (OVERALL PICTURE PER CONFIG)

% --- DASHBOARD 1: AERO ON OVERVIEW ---
num_runs_on = size(windows_on, 1);
figure('Name', 'Aero ON: Run Documentation', 'Position', [50, 100, 1400, 800]);
for i = 1:num_runs_on
    plot_run_column(runs_on{i}, i, num_runs_on, sprintf('Aero ON: Run %d', i+1)); 
end

% --- DASHBOARD 2: AERO OFF OVERVIEW ---
num_runs_off = size(windows_off, 1);
figure('Name', 'Aero OFF: Run Documentation', 'Position', [100, 100, 1600, 800]);
for i = 1:num_runs_off
    plot_run_column(runs_off{i}, i, num_runs_off, sprintf('Aero OFF: Run %d', i));
end

%% 6. AERO ANALYSIS (DOWNFORCE & DRAG DELTAS)

v_sq_on  = (aero_on.v / 3.6).^2; 
v_sq_off = (aero_off.v / 3.6).^2;

% --- FIGURE 3: DOWNFORCE EXTRACTION (Travel vs V^2) ---
figure('Name', 'Downforce Comparison (Weight Transfer Cancelled)', 'Position', [150, 150, 800, 500]);
scatter(v_sq_off, aero_off.dTotal, 15, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.5); hold on;
scatter(v_sq_on, aero_on.dTotal, 15, 'b', 'filled', 'MarkerFaceAlpha', 0.5); 

valid_on  = aero_on.v > 20; 
valid_off = aero_off.v > 20;

p_on  = polyfit(v_sq_on(valid_on), aero_on.dTotal(valid_on), 1);
p_off = polyfit(v_sq_off(valid_off), aero_off.dTotal(valid_off), 1);

v_sq_fit = linspace(0, max(v_sq_on), 50);
plot(v_sq_fit, polyval(p_off, v_sq_fit), 'w-', 'LineWidth', 2, 'DisplayName', sprintf('Aero OFF Slope: %.4f', p_off(1)));
plot(v_sq_fit, polyval(p_on, v_sq_fit), 'b-', 'LineWidth', 2, 'DisplayName', sprintf('Aero ON Slope: %.4f', p_on(1)));

xlabel('Speed Squared (m/s)^2', 'FontWeight', 'bold');
ylabel('Total Compression (Front + Rear) [mm]', 'FontWeight', 'bold');
title('Total Downforce Compression vs Speed^2', 'FontWeight', 'bold');
grid on; legend('Location', 'northwest');

% --- FIGURE 4: DRAG EXTRACTION (Ax vs Speed) ---
figure('Name', 'Drag Penalty (Ax vs Speed)', 'Position', [970, 150, 800, 500]);
scatter(aero_off.v, aero_off.ax, 15, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.5); hold on;
scatter(aero_on.v, aero_on.ax, 15, 'r', 'filled', 'MarkerFaceAlpha', 0.5);

p_ax_on  = polyfit(aero_on.v(valid_on), aero_on.ax(valid_on), 2);
p_ax_off = polyfit(aero_off.v(valid_off), aero_off.ax(valid_off), 2);

v_fit = linspace(20, max(aero_on.v), 50);
plot(v_fit, polyval(p_ax_off, v_fit), 'w-', 'LineWidth', 2, 'DisplayName', 'Aero OFF Fit');
plot(v_fit, polyval(p_ax_on, v_fit), 'r-', 'LineWidth', 2, 'DisplayName', 'Aero ON Fit');

xlabel('Speed (km/h)', 'FontWeight', 'bold');
ylabel('Forward Acceleration (G)', 'FontWeight', 'bold');
title('Acceleration Capability vs Speed (Drag Penalty)', 'FontWeight', 'bold');
grid on; legend('Location', 'southwest');

%% 7. EXECUTIVE SUMMARY PRINTOUT
target_v_kmh = 60.0;
ax_at_60_off = polyval(p_ax_off, target_v_kmh);
ax_at_60_on  = polyval(p_ax_on, target_v_kmh);
drag_penalty_G = ax_at_60_off - ax_at_60_on;

fprintf('\n===================================================\n');
fprintf('  AERO ON VS AERO OFF DELTA REPORT\n');
fprintf('===================================================\n');
fprintf('DOWNFORCE (CLA PROXY):\n');
fprintf('  Aero OFF Compression Slope : %.4f mm / (m/s)^2\n', p_off(1));
fprintf('  Aero ON Compression Slope  : %.4f mm / (m/s)^2\n', p_on(1));
fprintf('  Net Aero Compression       : %.4f mm / (m/s)^2\n', p_on(1) - p_off(1));
fprintf('DRAG (CDA PROXY) @ %.1f km/h:\n', target_v_kmh);
fprintf('  Aero OFF Accel   : %.3f G\n', ax_at_60_off);
fprintf('  Aero ON Accel    : %.3f G\n', ax_at_60_on);
fprintf('  Net Drag Penalty : -%.3f G\n', drag_penalty_G);
fprintf('===================================================\n\n');

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function [runs, aero] = process_and_calibrate(filepath, windows, cal)
    if ~isfile(filepath)
        time = (0:0.01:500)'; df = table(time);
        df.ThrottlePosition = zeros(size(time)); df.EngineSpeed = zeros(size(time));
        df.WheelSpeedFrontLeft = zeros(size(time)); df.WheelSpeedFrontRight = zeros(size(time));
        df.SHOCKFL = 129.4*ones(size(time)); df.SHOCKFR = 178.7*ones(size(time));
        df.SHOCKRL = 164.8*ones(size(time)); df.SHOCKRR = 177.4*ones(size(time));
    else
        df = readtable(filepath);
    end
    
    time = df.Time;
    tps  = df.ThrottlePosition;
    rpm  = df.EngineSpeed;
    
    v_kmh = (df.WheelSpeedFrontLeft + df.WheelSpeedFrontRight) / 2;
    v_ms  = v_kmh / 3.6; 
    
    dt = mean(diff(time)); if isnan(dt) || dt <= 0; dt = 0.01; end
    a_x_raw_g = gradient(v_ms, dt) / 9.81; 
    fs = 1/dt; [b, a] = butter(2, 2.0 / (fs/2), 'low');
    a_x_g = filtfilt(b, a, double(a_x_raw_g));
    
    % --- DYNAMIC ZEROING ---
    % Find stationary window at the start of the file to establish absolute 0 mm
    static_mask = (v_kmh < 1) & (time < time(1) + 30); 
    if sum(static_mask) < 50; static_mask = 1:50; end
    
    FL_0 = mean(df.SHOCKFL(static_mask));
    FR_0 = mean(df.SHOCKFR(static_mask));
    RL_0 = mean(df.SHOCKRL(static_mask));
    RR_0 = mean(df.SHOCKRR(static_mask));

    % Apply dynamic baselines and passed-in scaling factors
    dFR = -(df.SHOCKFR - FR_0); 
    dRR = -(df.SHOCKRR - RR_0);
    dFL = -((df.SHOCKFL - FL_0) * cal.FL_scale);
    dRL = -((df.SHOCKRL - RL_0) * cal.RL_scale);
    
    dFront = (dFL + dFR) / 2;
    dRear  = (dRL + dRR) / 2;
    dTotal = dFront + dRear;
    
    runs = cell(size(windows, 1), 1);
    aero.v = []; aero.ax = []; aero.dTotal = []; aero.dFront = []; aero.dRear = [];
    
    for i = 1:size(windows, 1)
        full_mask = (time >= windows(i,1)) & (time <= windows(i,2));
        run_data.time = time(full_mask);
        run_data.v = v_kmh(full_mask);
        run_data.rpm = rpm(full_mask);
        run_data.tps = tps(full_mask);
        run_data.ax = a_x_g(full_mask);
        run_data.dFL = dFL(full_mask); run_data.dFR = dFR(full_mask);
        run_data.dRL = dRL(full_mask); run_data.dRR = dRR(full_mask);
        runs{i} = run_data;
        
        wot_mask = full_mask & (tps > 85);
        aero.v = [aero.v; v_kmh(wot_mask)];
        aero.ax = [aero.ax; a_x_g(wot_mask)];
        aero.dTotal = [aero.dTotal; dTotal(wot_mask)];
        aero.dFront = [aero.dFront; dFront(wot_mask)];
        aero.dRear  = [aero.dRear; dRear(wot_mask)];
    end
end

function plot_run_column(run_data, col_idx, total_cols, col_title)
    ax1 = subplot(3, total_cols, col_idx);
    
    yyaxis left; 
    plot(run_data.time, run_data.v, 'b', 'LineWidth', 1.5);
    ylabel('Speed (km/h)'); ylim([0, max(max(run_data.v)*1.1, 100)]);
    ax1.YAxis(1).Color = 'b';
    
    yyaxis right; 
    plot(run_data.time, run_data.rpm, 'w', 'LineWidth', 1.2);
    ylabel('RPM'); ylim([0, 12000]);
    ax1.YAxis(2).Color = 'w';
    
    title(col_title); grid on;
    
    ax2 = subplot(3, total_cols, col_idx + total_cols);
    
    yyaxis left; 
    plot(run_data.time, run_data.tps, 'r', 'LineWidth', 1.5);
    ylabel('Throttle (%)'); ylim([0, 110]);
    ax2.YAxis(1).Color = 'r';
    
    yyaxis right; 
    ax_color = [0.8500 0.3250 0.0980];
    plot(run_data.time, run_data.ax, 'Color', ax_color, 'LineWidth', 1.5);
    ylabel('Derived Ax (G)'); ylim([-0.5, 1.5]);
    ax2.YAxis(2).Color = ax_color;
    
    grid on;
    
    subplot(3, total_cols, col_idx + 2*total_cols);
    plot(run_data.time, run_data.dFL, 'r--', 'LineWidth', 1.2, 'DisplayName', 'FL'); hold on;
    plot(run_data.time, run_data.dFR, 'b', 'LineWidth', 1.2, 'DisplayName', 'FR');
    plot(run_data.time, run_data.dRL, 'g--', 'LineWidth', 1.2, 'DisplayName', 'RL');
    plot(run_data.time, run_data.dRR, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.2, 'DisplayName', 'RR');
    ylabel('Compression (mm)'); xlabel('Time (s)');
    grid on; 
    
    if col_idx == 1; legend('Location', 'northwest'); end
end