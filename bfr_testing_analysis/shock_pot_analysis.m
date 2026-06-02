data = readtable('Alameda_Skidpad0384LLTD_2026-05-02T20-21-47');

% Display the first few rows to see sensor names
%head(data)


% Display only sensors that contain values other than NaN and 0

isUseful = varfun(@(x) isnumeric(x) && any(x ~= 0 & ~isnan(x)), data, 'OutputFormat', 'uniform');
% 2. Create a new table containing only the useful columns
activeData = data(:, isUseful);

% 3. Display the first few rows of the filtered table
head(activeData)

%{

t = data.Time;

clean_time = data.Time(~isnan(data.Time));

% Calculate differences
dt = diff(clean_time);

% Filter out zeros (duplicate timestamps) and check average
dt = dt(dt > 0); 
fs_calc = 1/mean(dt);

fprintf('The cleaned logging frequency is: %.2f Hz\n', fs_calc);




% Front Right
% subplot(2,1,1);
% plot(t, data.SHOCKFR, 'b');
% title('Front Right Shock Travel');
% ylabel('mm'); grid on;



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