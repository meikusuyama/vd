%% FSAE TELEMETRY: SHOCKPOT SYMMETRY CALIBRATION VIA SKIDPAD
clc; close all; clear;

% 1. LOAD DATA (Replace with your skidpad file)
filename = 'testing_data/26-10-4-alameda_testing/2026-10-04_alameda_skidpad_run5_MR2_long(4)_aero_off.csv'; 
df = readtable(filename); 

% --- 2. EXTRACT & FILTER CHANNELS ---
time  = df.Time;
steer = df.STEERINGANGLE;
v_kmh = df.WheelSpeedFrontLeft; % Proxy for active speed

raw_FL = df.SHOCKFL;
raw_FR = df.SHOCKFR;
raw_RL = df.SHOCKRL;
raw_RR = df.SHOCKRR;

% Light low-pass filter (5Hz) to strip raw electrical noise for clean scaling
dt = mean(diff(time)); if isnan(dt) || dt <= 0; dt = 0.01; end
[b, a] = butter(2, 5.0 / ( (1/dt)/2 ), 'low');
FL_f = filtfilt(b, a, double(raw_FL));
FR_f = filtfilt(b, a, double(raw_FR));
RL_f = filtfilt(b, a, double(raw_RL));
RR_f = filtfilt(b, a, double(raw_RR));

% --- 3. ZERO THE SENSORS (RELATIVE TRAVEL) ---
% Dynamically find the exact moment the car launches for the skidpad
run_start_idx = find(v_kmh > 10, 1, 'first');
if isempty(run_start_idx)
    run_start_idx = length(time); 
end
t_start = time(run_start_idx);

% Define static window strictly as the 10 seconds immediately BEFORE launch
static_mask = (time >= t_start - 12) & (time <= t_start - 2);

% Fallback in case the run starts immediately in the log
if sum(static_mask) < 50
    static_mask = (v_kmh < 1);
    if sum(static_mask) < 50
        static_mask = 1:50; 
    end
end

FL_0 = mean(FL_f(static_mask));
FR_0 = mean(FR_f(static_mask));
RL_0 = mean(RL_f(static_mask));
RR_0 = mean(RR_f(static_mask));

% Calculate relative travel
dFL_raw = FL_f - FL_0;
dFR     = FR_f - FR_0; % Healthy Reference
dRL_raw = RL_f - RL_0;
dRR     = RR_f - RR_0; % Healthy Reference

% --- 4. CALCULATE SYMMETRY SCALING MULTIPLIERS ---
% Isolate the active skidpad cornering to compare amplitudes
active_mask = (v_kmh > 20) & (abs(steer) > 20);

% Using Standard Deviation to find average amplitude, ignoring random spikes
amp_FR = std(dFR(active_mask));
amp_FL = std(dFL_raw(active_mask));
amp_RR = std(dRR(active_mask));
amp_RL = std(dRL_raw(active_mask));

% Compute Multipliers
scale_FL = amp_FR / amp_FL;
scale_RL = amp_RR / amp_RL;

% Apply Multipliers to the bad sensors
dFL_fixed = dFL_raw * scale_FL;
dRL_fixed = dRL_raw * scale_RL;

% --- MANUAL ABSOLUTE ZERO CORRECTION ---
% Tune these numbers to manually shift the final traces UP or DOWN.
manual_shift_FL = 0.0; 
manual_shift_RL = -3.0; 

dFL_fixed = dFL_fixed - manual_shift_FL;
dRL_fixed = dRL_fixed - manual_shift_RL;

% --- 5. PRINT CALIBRATION REPORT ---
fprintf('===================================================\n');
fprintf('  SHOCKPOT CALIBRATION & SCALING REPORT            \n');
fprintf('===================================================\n');
fprintf('FRONT AXLE:\n');
fprintf('  FR Static Baseline : %5.1f mm (Reference)\n', FR_0);
fprintf('  FL Static Baseline : %5.1f mm (Shift Applied: %+.1f mm)\n', FL_0, FR_0 - FL_0);
fprintf('  FL Scaling Factor  : x%.3f\n\n', scale_FL);
fprintf('REAR AXLE:\n');
fprintf('  RR Static Baseline : %5.1f mm (Reference)\n', RR_0);
fprintf('  RL Static Baseline : %5.1f mm\n', RL_0);
fprintf('  RL Scaling Factor  : x%.3f\n', scale_RL);
fprintf('===================================================\n');

% --- 6. PLOT BEFORE & AFTER ---
figure('Name', 'Shockpot Symmetry Calibration', 'Position', [100, 100, 1200, 800]);

% Uncalibrated Raw Plot
subplot(2,1,1);
plot(time, raw_FR, 'b', 'LineWidth', 1.2, 'DisplayName', 'FR (Healthy)'); hold on;
plot(time, raw_RR, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.2, 'DisplayName', 'RR (Healthy)');
plot(time, raw_RL, 'g', 'LineWidth', 1.2, 'DisplayName', 'RL (Low Amp)');
plot(time, raw_FL, 'r', 'LineWidth', 1.2, 'DisplayName', 'FL (Bad Baseline/Amp)');
title('1. RAW UNCALIBRATED TRAVEL'); ylabel('Raw Position');
grid on; legend('Location', 'northeast');

% Calibrated Relative Plot
subplot(2,1,2);
plot(time, dFR, 'b', 'LineWidth', 1.2, 'DisplayName', 'dFR (Healthy)'); hold on;
plot(time, dRR, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.2, 'DisplayName', 'dRR (Healthy)');
plot(time, dRL_fixed, 'g--', 'LineWidth', 1.5, 'DisplayName', 'dRL (Scaled)');
plot(time, dFL_fixed, 'r--', 'LineWidth', 1.5, 'DisplayName', 'dFL (Shifted & Scaled)');
title(sprintf('2. RELATIVE TRAVEL (ZEROED & SCALED) | FL Scale = %.2f, RL Scale = %.2f', scale_FL, scale_RL));
xlabel('Time (s)'); ylabel('Relative Travel (\Delta)');
grid on; legend('Location', 'northeast');