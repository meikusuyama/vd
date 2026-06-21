data = readtable('testing_data/4-18-endurance-stint1-uprights-failure.csv');

%{
% Display only sensors that contain values other than NaN and 0

isUseful = varfun(@(x) isnumeric(x) && any(x ~= 0 & ~isnan(x)), data, 'OutputFormat', 'uniform');
% 2. Create a new table containing only the useful columns
activeData = data(:, isUseful);

% 3. Display the first few rows of the filtered table
head(activeData)

%}

%{
t = data.Time;

clean_time = data.Time(~isnan(data.Time));

% Calculate differences
dt = diff(clean_time);

% Filter out zeros (duplicate timestamps) and check average
dt = dt(dt > 0); 
fs_calc = 1/mean(dt);

fprintf('The cleaned logging frequency is: %.2f Hz\n', fs_calc);



%% FSAE Front Right Shock Analysis Script
% This script cleans, filters, and visualizes SHOCKFR data.

%% 1. Configuration & Signal Cleaning
fs = fs_calc; 
fc = 12;      %Cutoff frequency (Hz) for handling
raw_signal = data.SHOCKFR;

% Fix finite errors
clean_raw = fillmissing(raw_signal, 'linear');

% Filtering
[b, a] = butter(2, fc/(fs/2));
FR_smooth = filtfilt(b, a, clean_raw);

% Velocity Calculation
FR_vel = [0; diff(FR_smooth)] .* fs;

% --- THE TIME FILTER SECTION ---
% Create a logical index where time is greater than set time
% Use a different name like 'idx' to avoid clashing with the 'filter' function
idx = data.Time > 0; 

%% 2. Visualization
figure('Name', 'Front Right Suspension Analysis', 'NumberTitle', 'off');

% Apply the index [idx] to both the X and Y axes
plot(data.Time(idx), clean_raw(idx), 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5); hold on;
plot(data.Time(idx), FR_smooth(idx), 'r', 'LineWidth', 1.5);

title('Time Trace: Displacement of Shocks');
xlabel('Time (s)'); ylabel('Travel (mm)');
legend('Raw + Noise', 'Filtered (Handling)');
grid on;

%% 4. Summary Stats (Updated to only analyze the filtered range)
fprintf('--- Front Right Analysis Summary (Post-700s) ---\n');
fprintf('Max Compression: %.2f mm\n', min(FR_smooth(idx)));
fprintf('Max Extension:   %.2f mm\n', max(FR_smooth(idx)));
fprintf('Max Velocity:    %.2f mm/s\n', max(abs(FR_vel(idx))));

%}





%% FSAE Comprehensive Suspension & Wheel Speed Analysis 
% Assumes 'data' table is already loaded in your workspace


%% 1. Configuration & Signal Processing
clean_time = data.Time(~isnan(data.Time));
dt = diff(clean_time);
dt = dt(dt > 0); 
fs = 1/mean(dt);

% --- FILTER SETUP ---
startTime = 390; 
idx = data.Time >= startTime; 
t_plot = data.Time(idx);

fc_shock = 12;      
fc_speed = 5;       

[b_shock, a_shock] = butter(2, fc_shock/(fs/2));
[b_speed, a_speed] = butter(2, fc_speed/(fs/2));

% --- Extract & Process Shocks ---
shocks_raw = [data.SHOCKFL, data.SHOCKFR, data.SHOCKRL, data.SHOCKRR];
shocks_smooth = filtfilt(b_shock, a_shock, fillmissing(shocks_raw, 'linear'));

% --- Extract & Process Wheel Speeds ---
speeds_raw = [data.WheelSpeedFrontLeft, data.WheelSpeedFrontRight, ...
              data.WheelSpeedRearLeft, data.WheelSpeedRearRight];
speeds_smooth = filtfilt(b_speed, a_speed, fillmissing(speeds_raw, 'linear'));

% --- Extract & Process Driver Inputs & Engine ---
steer_smooth = filtfilt(b_shock, a_shock, fillmissing(data.STEERINGANGLE, 'linear'));
tps_smooth   = filtfilt(b_speed, a_speed, fillmissing(data.ThrottlePosition, 'linear'));
rpm_smooth   = filtfilt(b_speed, a_speed, fillmissing(data.EngineSpeed, 'linear'));

% Define MoTeC Hex Colors for Dark Mode
c_fl = [1.0 0.2 0.2];   % Vibrant Red
c_fr = [0.2 0.6 1.0];   % Electric Blue
c_rl = [0.2 1.0 0.4];   % Neon Green
c_rr = [1.0 0.8 0.2];   % Bright Yellow
c_st = [1.0 0.4 1.0];   % Magenta
c_rpm = [0.4 1.0 1.0];  % Cyan
c_tps = [0.9 0.9 0.9];  % Light Off-White

%% ========================================================================
%% DASHBOARD 1: Isolated Suspension Stack (Vertical Breakdown)
%% ========================================================================
fig1 = figure('Name', 'Dashboard 1: Corner Suspension Deflection', 'Color', [0.08 0.08 0.08]);

% 1. Front Left
subplot(4,1,1); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, shocks_smooth(idx, 1), 'Color', c_fl, 'LineWidth', 1.2);
ylabel('FL (mm)'); title('Front Left Travel', 'Color', 'w'); grid on; xticklabels({});

% 2. Front Right
subplot(4,1,2); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, shocks_smooth(idx, 2), 'Color', c_fr, 'LineWidth', 1.2);
ylabel('FR (mm)'); title('Front Right Travel', 'Color', 'w'); grid on; xticklabels({});

% 3. Rear Left
subplot(4,1,3); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, shocks_smooth(idx, 3), 'Color', c_rl, 'LineWidth', 1.2);
ylabel('RL (mm)'); title('Rear Left Travel', 'Color', 'w'); grid on; xticklabels({});

% 4. Rear Right
subplot(4,1,4); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, shocks_smooth(idx, 4), 'Color', c_rr, 'LineWidth', 1.2);
ylabel('RR (mm)'); xlabel('Time (s)'); title('Rear Right Travel', 'Color', 'w'); grid on;

linkaxes(findall(fig1, 'Type', 'axes'), 'x');

%% ========================================================================
%% DASHBOARD 2: Isolated Wheel Speed Stack (Vertical Breakdown)
%% ========================================================================
fig2 = figure('Name', 'Dashboard 2: Individual Wheel Speeds', 'Color', [0.08 0.08 0.08]);

% 1. Front Left Speed
subplot(4,1,1); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, speeds_smooth(idx, 1), 'Color', c_fl, 'LineWidth', 1.2);
ylabel('FL Spd'); title('Front Left Wheel Speed', 'Color', 'w'); grid on; xticklabels({});

% 2. Front Right Speed
subplot(4,1,2); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, speeds_smooth(idx, 2), 'Color', c_fr, 'LineWidth', 1.2);
ylabel('FR Spd'); title('Front Right Wheel Speed', 'Color', 'w'); grid on; xticklabels({});

% 3. Rear Left Speed
subplot(4,1,3); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, speeds_smooth(idx, 3), 'Color', c_rl, 'LineWidth', 1.2);
ylabel('RL Spd'); title('Rear Left Wheel Speed', 'Color', 'w'); grid on; xticklabels({});

% 4. Rear Right Speed
subplot(4,1,4); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, speeds_smooth(idx, 4), 'Color', c_rr, 'LineWidth', 1.2);
ylabel('RR Spd'); xlabel('Time (s)'); title('Rear Right Wheel Speed', 'Color', 'w'); grid on;

linkaxes(findall(fig2, 'Type', 'axes'), 'x');

%% ========================================================================
%% DASHBOARD 3: Combined Overlays & Engine Diagnostics
%% ========================================================================
fig3 = figure('Name', 'Dashboard 3: Integrated Vehicle Dynamics Overview', 'Color', [0.08 0.08 0.08]);

% Top Panel: All 4 Shocks Overlaid (Useful to spot full car pitch/roll trends at a glance)
subplot(4,1,1); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, shocks_smooth(idx, 1), 'Color', c_fl); 
plot(t_plot, shocks_smooth(idx, 2), 'Color', c_fr);
plot(t_plot, shocks_smooth(idx, 3), 'Color', c_rl); 
plot(t_plot, shocks_smooth(idx, 4), 'Color', c_rr);
ylabel('Travel (mm)'); title('Suspension Overlay', 'Color', 'w');
legend('FL', 'FR', 'RL', 'RR', 'TextColor', 'w', 'Box', 'off', 'Location', 'northeast'); grid on; xticklabels({});

% Second Panel: Steering Angle Tracking
subplot(4,1,2); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, steer_smooth(idx), 'Color', c_st, 'LineWidth', 1.2);
ylabel('Steer (deg)'); title('Steering Input', 'Color', 'w'); grid on; xticklabels({});

% Third Panel: All 4 Wheel Speeds Overlaid (Instantly spots wheel slip/lock separation)
subplot(4,1,3); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(t_plot, speeds_smooth(idx, 1), 'Color', c_fl); 
plot(t_plot, speeds_smooth(idx, 2), 'Color', c_fr);
plot(t_plot, speeds_smooth(idx, 3), 'Color', c_rl); 
plot(t_plot, speeds_smooth(idx, 4), 'Color', c_rr);
ylabel('Speed'); title('Wheel Velocities Overlay', 'Color', 'w');
legend('FL', 'FR', 'RL', 'RR', 'TextColor', 'w', 'Box', 'off', 'Location', 'northeast'); grid on; xticklabels({});

% Bottom Panel: NEW Engine Diagnostics (Throttle Position vs Engine Speed)
subplot(4,1,4); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
yyaxis left
plot(t_plot, rpm_smooth(idx), 'Color', c_rpm, 'LineWidth', 1.2); 
ylabel('Engine RPM'); set(gca, 'YColor', c_rpm);
yyaxis right
plot(t_plot, tps_smooth(idx), 'Color', c_tps, 'LineWidth', 1.0); 
ylabel('Throttle %'); set(gca, 'YColor', c_tps);
xlabel('Time (s)'); title('Powertrain Deployment', 'Color', 'w'); grid on;

linkaxes(findall(fig3, 'Type', 'axes'), 'x');