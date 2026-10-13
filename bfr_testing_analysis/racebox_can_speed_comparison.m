%% QUICK FILE MATCHING & TIME SHIFT INSPECTION TOOL
clc; close all; clear;

%% 1. FILE PATHS & TIME OFFSET PARAMETER
% Replace with the file paths you want to test and compare
file_racebox = 'testing_data/racebox_data/RaceBox_Track_Sessionon_04-05-2026_05-46.csv';
file_canary  = 'testing_data/rotortemp_fullendurance_05_04_2026.csv';

% Initial time offset guess in seconds (t_can_aligned = t_can - time_offset)
time_offset = 69.3; 

%% 2. LOAD DATASETS
opts_rb = detectImportOptions(file_racebox);
df_rb   = readtable(file_racebox, opts_rb);
df_can  = readtable(file_canary);

%% 3. EXTRACT AND SANITIZE SPEED CHANNELS
% RaceBox GPS Speed (Converted from raw MPH to km/h)
t_rb     = df_rb.Time;
v_rb_kmh = df_rb.Speed * 1.60934; 

% CANary Wheel Speed (Average Front Left and Right)
t_can = df_can.Time;
v_fl  = df_can.WheelSpeedFrontLeft;
v_fr  = df_can.WheelSpeedFrontRight;
v_can_kmh = (v_fl + v_fr) / 2;

% Reject CAN wheel speed spikes (>80 km/h)
v_can_kmh(v_can_kmh > 80) = NaN;
v_can_kmh = fillmissing(v_can_kmh, 'linear', 'EndValues', 'nearest');

% Apply Time Offset to CANary timeline
t_can_aligned = t_can - time_offset;

%% 4. STACKED SUBPLOTS WITH LINKED X-AXES
figure('Name', 'Speed Profile File Matcher & Shift Finder', 'Position', [100, 50, 1100, 750]);

% Subplot 1: RaceBox GPS Speed
ax1 = subplot(2,1,1);
plot(t_rb, v_rb_kmh, 'r', 'LineWidth', 1.4);
ylabel('Speed (km/h)');
title('1. RaceBox GPS Speed (Reference Timeline)');
ylim([0, 85]); grid on;

% Subplot 2: CANary Wheel Speed (Shifted)
ax2 = subplot(2,1,2);
plot(t_can_aligned, v_can_kmh, 'b', 'LineWidth', 1.4);
xlabel('Aligned Timeline (seconds)');
ylabel('Speed (km/h)');
title(sprintf('2. CANary Wheel Speed (Shifted by -%.1f seconds)', time_offset));
ylim([0, 85]); grid on;

% Link X-axes: Zooming/panning on one plot automatically shifts the other!
linkaxes([ax1, ax2], 'x');

sgtitle('RaceBox vs. CANary Speed Profile Inspection', 'FontSize', 13, 'FontWeight', 'bold');