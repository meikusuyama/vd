
data = readtable('testing_data/4-18-endurance-stint1-uprights-failure.csv');

%% FSAE Traction vs. Power Limit Analysis (G-V Diagram)
% Assumes 'data' table is loaded in your workspace

%% 1. Configuration & Signal Clean
fs = 200; % 200Hz frequency from your 0.005s interval

% CRITICAL: Filter to the healthy endurance window (Post-390s, Pre-Upright Failure)
idx = data.Time >= 550 & data.Time <= 605;

t_plot   = data.Time(idx);
long_g   = fillmissing(data.ACCELX(idx), 'linear');
lat_g    = fillmissing(data.ACCELY(idx), 'linear');
tps      = fillmissing(data.ThrottlePosition(idx), 'linear');

% Extract the 3 working wheel speeds
ws_fl = fillmissing(data.WheelSpeedFrontLeft(idx), 'linear');
ws_fr = fillmissing(data.WheelSpeedFrontRight(idx), 'linear');
ws_rl = fillmissing(data.WheelSpeedRearLeft(idx), 'linear');

% --- Signal Filtering ---
[b, a] = butter(2, 4/(fs/2)); % 4Hz filter to get clean, steady-state limits
long_g_smooth = filtfilt(b, a, long_g);
lat_g_smooth  = filtfilt(b, a, lat_g);

%% 2. Calculate Vehicle Speed & Rear Wheel Slip
% Front wheels are non-driven, so their average is our true ground speed
v_vehicle = (ws_fl + ws_fr) / 2;

% Calculate how much the working rear wheel is spinning compared to the fronts
rear_slip_delta = ws_rl - v_vehicle; 

%% 3. Visualization Dashboard (MoTeC Dark Theme)
figure('Name', 'Aero & Traction Limit Dashboard', 'Color', [0.08 0.08 0.08]);

% --- PLOT 1: The G-V Diagram (The Core Answer) ---
subplot(2,2,1); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
% Scatter plot of AccelX vs Speed, colored by Throttle Position
scatter(v_vehicle, long_g_smooth, 12, tps, 'filled');
xlabel('Vehicle Speed'); ylabel('Longitudinal Accel (G)');
title('G-V Diagram: Colored by Throttle %', 'Color', 'w');
grid on; cb1 = colorbar; ylabel(cb1, 'Throttle %', 'Color', 'w');
colormap('jet');

% --- PLOT 2: G-V Diagram colored by Rear Wheel Slip ---
subplot(2,2,2); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
% Scatter plot colored by how much the rear tire is spinning
scatter(v_vehicle, long_g_smooth, 12, rear_slip_delta, 'filled');
xlabel('Vehicle Speed'); ylabel('Longitudinal Accel (G)');
title('G-V Diagram: Colored by Rear Wheel Slip', 'Color', 'w');
grid on; cb2 = colorbar; ylabel(cb2, 'Rear Wheel Spin Delta', 'Color', 'w');

% --- SAFETY CHECK FOR COLOR LIMITS ---
max_slip = max(rear_slip_delta);
if max_slip > 0
    clim([0 max_slip * 0.5]); % Dynamically scale if there is wheel spin
else
    clim([0 5]); % Fallback scale if the tires never spun
end

% --- PLOT 3: The Traction Circle (G-G Diagram) ---
subplot(2,2,3); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
scatter(lat_g_smooth, long_g_smooth, 12, v_vehicle, 'filled');
xlabel('Lateral G (ACCELY)'); ylabel('Longitudinal G (ACCELX)');
title('Traction Circle Colored by Speed', 'Color', 'w');
grid on; axis equal; cb3 = colorbar; ylabel(cb3, 'Speed', 'Color', 'w');

% --- PLOT 4: Time-Series Diagnostic (Where does it happen?) ---
subplot(2,2,4); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
yyaxis left
plot(t_plot, long_g_smooth, 'Color', [0.2 0.6 1.0], 'LineWidth', 1.2); 
ylabel('Longitudinal Accel (G)'); set(gca, 'YColor', [0.2 0.6 1.0]);
yyaxis right
plot(t_plot, v_vehicle, 'Color', [1.0 0.8 0.2], 'LineWidth', 1.2); 
ylabel('Vehicle Speed'); set(gca, 'YColor', [1.0 0.8 0.2]);
xlabel('Time (s)'); title('Acceleration Profile Over Time', 'Color', 'w');
grid on;