data = readtable('testing_data/2026-10-04_alameda_skidpad_run1_MR2_long(4)_aero_on.csv');


%{
% Display only sensors that contain values other than NaN and 0

isUseful = varfun(@(x) isnumeric(x) && any(x ~= 0 & ~isnan(x)), data, 'OutputFormat', 'uniform');
% 2. Create a new table containing only the useful columns
activeData = data(:, isUseful);

% 3. Display the first few rows of the filtered table
head(activeData)


%}

%% FSAE Comprehensive Suspension & Wheel Speed Analysis 
% Assumes 'data' table is already loaded in your workspace


%% 1. Configuration & Signal Processing
clean_time = data.Time(~isnan(data.Time));
dt = diff(clean_time);
dt = dt(dt > 0); 
fs = 1/mean(dt);

% --- FILTER SETUP ---
startTime = 0; 
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

