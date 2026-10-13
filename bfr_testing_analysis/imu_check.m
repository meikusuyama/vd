
data = readtable('testing_data/2026-10-04_alameda_skidpad_run1_MR2_long(4)_aero_on.csv');


%{

% Display only sensors that contain values other than NaN and 0

isUseful = varfun(@(x) isnumeric(x) && any(x ~= 0 & ~isnan(x)), data, 'OutputFormat', 'uniform');
% 2. Create a new table containing only the useful columns
activeData = data(:, isUseful);

% 3. Display the first few rows of the filtered table
head(activeData)

%}


%% ========================================================================
%% IMU VALIDATION & SANITY CHECK DASHBOARD (Standalone)
%% ========================================================================

% 1. Configuration & Signal Processing Setup
% (Assuming 'data' table or struct is already loaded into the workspace)
clean_time = data.Time(~isnan(data.Time));
dt = diff(clean_time);
dt = dt(dt > 0); 
fs = 1/mean(dt);

startTime = 0; 
idx = data.Time >= startTime; 
t_plot = data.Time(idx);

% --- FILTER SETUP ---
fc_imu = 4;   % 4Hz low-pass for chassis IMU dynamics
fc_driver = 5; % 5Hz for driver inputs
[b_imu, a_imu] = butter(2, fc_imu/(fs/2));
[b_drv, a_drv] = butter(2, fc_driver/(fs/2));

% --- Extract & Process Data ---
% IMU (Adjust variable names to match your datalogger)
accelX_smooth = filtfilt(b_imu, a_imu, fillmissing(data.ACCELX, 'linear'));
accelY_smooth = filtfilt(b_imu, a_imu, fillmissing(data.ACCELY, 'linear'));
yaw_smooth    = filtfilt(b_imu, a_imu, fillmissing(data.ANGRATEZ, 'linear'));

% Driver Inputs
steer_smooth = filtfilt(b_drv, a_drv, fillmissing(data.STEERINGANGLE, 'linear'));
tps_smooth   = filtfilt(b_drv, a_drv, fillmissing(data.ThrottlePosition, 'linear'));

% Vehicle Speed (Average of all 4 wheels)
speeds_raw = [data.WheelSpeedFrontLeft, data.WheelSpeedFrontRight, ...
              data.WheelSpeedRearLeft, data.WheelSpeedRearRight];
speeds_smooth = filtfilt(b_drv, a_drv, fillmissing(speeds_raw, 'linear'));
speed_avg = mean(speeds_smooth, 2);

% --- Define MoTeC Hex Colors for Dark Mode ---
c_st  = [1.0 0.4 1.0];  % Magenta (Steering)
c_tps = [0.9 0.9 0.9];  % Off-White (Throttle)
c_ax  = [1.0 0.5 0.0];  % Orange (Longitudinal Accel)
c_ay  = [0.8 0.2 1.0];  % Purple (Lateral Accel)
c_yaw = [0.0 0.8 0.4];  % Mint Green (Yaw Rate)

%% ========================================================================
%% PLOTTING: DASHBOARD GENERATION
%% ========================================================================
fig_imu = figure('Name', 'IMU Validation & Sanity Check', 'Color', [0.08 0.08 0.08], 'Position', [100, 100, 1000, 800]);

% -------------------------------------------------------------------------
% Panel 1: Lateral Dynamics (Steering vs. Lateral G's & Yaw Rate)
% -------------------------------------------------------------------------
ax1 = subplot(3, 1, 1); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
yyaxis left
plot(t_plot, steer_smooth(idx), 'Color', c_st, 'LineWidth', 1.2);
ylabel('Steering Angle (deg)'); set(gca, 'YColor', c_st);

yyaxis right
plot(t_plot, accelY_smooth(idx), 'Color', c_ay, 'LineWidth', 1.2); hold on;
plot(t_plot, yaw_smooth(idx), 'Color', c_yaw, 'LineWidth', 1.2, 'LineStyle', '--');
ylabel('Lat Accel (G) / Yaw (deg/s)'); set(gca, 'YColor', 'w');

title('Lateral Sanity Check: Steering vs. Ay & Yaw', 'Color', 'w');
legend('Steering', 'Lateral Accel (Ay)', 'Yaw Rate', 'TextColor', 'w', 'Box', 'off', 'Location', 'northwest');
grid on; xticklabels({});

% -------------------------------------------------------------------------
% Panel 2: Longitudinal Dynamics (Throttle/Speed vs. Longitudinal G's)
% -------------------------------------------------------------------------
ax2 = subplot(3, 1, 2); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
yyaxis left
plot(t_plot, speed_avg(idx), 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2);
ylabel('Avg Wheel Speed'); set(gca, 'YColor', [0.7 0.7 0.7]);

yyaxis right
plot(t_plot, accelX_smooth(idx), 'Color', c_ax, 'LineWidth', 1.2); hold on;
plot(t_plot, tps_smooth(idx) / 100, 'Color', c_tps, 'LineStyle', ':'); % Scaled throttle for visibility
ylabel('Long Accel (G) / TPS'); set(gca, 'YColor', c_ax);

title('Longitudinal Sanity Check: Speed/TPS vs. Ax', 'Color', 'w');
legend('Speed', 'Long Accel (Ax)', 'Throttle Input', 'TextColor', 'w', 'Box', 'off', 'Location', 'northwest');
grid on;

% Link time axes for the top two panels
linkaxes([ax1, ax2], 'x');

% -------------------------------------------------------------------------
% Panel 3: G-G Diagram (Friction Circle)
% -------------------------------------------------------------------------
subplot(3, 1, 3); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
scatter(accelY_smooth(idx), accelX_smooth(idx), 15, speed_avg(idx), 'filled', 'MarkerFaceAlpha', 0.6);
colormap(gca, 'jet');
cb = colorbar; 
cb.Color = 'w'; 
cb.Label.String = 'Vehicle Speed';

% Draw Crosshairs
xline(0, 'w--', 'LineWidth', 1);
yline(0, 'w--', 'LineWidth', 1);

xlabel('Lateral Accel (Ay)');
ylabel('Longitudinal Accel (Ax)');
title('G-G Diagram (Friction Circle Sanity Check)', 'Color', 'w');
grid on; axis equal; % Forces 1:1 aspect ratio so the circle isn't warped
