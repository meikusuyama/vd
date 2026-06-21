%% FSAE Aero Coast-Down Analysis Framework

data_base = readtable('testing_data/racebox_data/05-03-26Baseline_Coastdown_Part2of2.csv', ...
    'VariableNamesLine', 13, ...
    'DataLines', [14, Inf]);
data_low = readtable('testing_data/racebox_data/05-03-26LowDragConfig_Coastdown_Part1of2.csv', ...
    'VariableNamesLine', 13, ...
    'DataLines', [14, Inf]);
% Assumes you have loaded two tables: 'data_base' and 'data_low'

fs = 20; % Adjust based on your GPS logger rate (RaceBox is usually 10Hz or 20Hz)
[b, a] = butter(2, 1/(fs/2)); % 1Hz low-pass filter to clean up GPS noise

%% 1. Process Baseline Setup
% Convert speed to m/s if logged in km/h (multiply by 1000/3600)
v_base = fillmissing(data_base.Speed, 'linear') * (1000/3600); 
% Extract deceleration (flip sign if your logger shows slowing down as negative)
accel_base = fillmissing(data_base.GForceX, 'linear'); 
decel_base = -filtfilt(b, a, accel_base); 

%% 2. Process Low Drag Setup
v_low = fillmissing(data_low.Speed, 'linear') * (1000/3600);
accel_low = fillmissing(data_low.GForceX, 'linear');
decel_low = -filtfilt(b, a, accel_low);

%% 3. Mathematical Curve Fitting (The Aero Extraction)
% We fit a second-order polynomial: Decel = C2*v^2 + C1*v + C0
% C2 is directly proportional to your Aerodynamic Drag (Cd*A)
poly_base = polyfit(v_base, decel_base, 2);
poly_low  = polyfit(v_low, decel_low, 2);

% Generate smooth curves for plotting
v_range = linspace(5, max([v_base; v_low]), 100);
fit_base = polyval(poly_base, v_range);
fit_low  = polyval(poly_low, v_range);

%% 4. Visualization Dashboard
figure('Name', 'Aerodynamic Coast-Down Evaluation', 'Color', [0.08 0.08 0.08]);

% --- PLOT 1: Velocity Decay Comparison ---
subplot(1,2,1); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
plot(data_base.Time, v_base, 'r-', 'LineWidth', 1.5);
plot(data_low.Time, v_low, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity Decay Rate', 'Color', 'w');
legend('Baseline Config', 'Low Drag Config', 'TextColor', 'w', 'Box', 'off');
grid on;

% --- PLOT 2: Deceleration vs. Speed Profile (The Core Metric) ---
subplot(1,2,2); set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on;
% Plot raw scatter data lightly
scatter(v_base, decel_base, 6, [1.0 0.4 0.4], 'filled');
scatter(v_low, decel_low, 6, [0.4 0.6 1.0], 'filled');
% Plot the clean physical fits
plot(v_range, fit_base, 'r-', 'LineWidth', 2.5);
plot(v_range, fit_low, 'b-', 'LineWidth', 2.5);
xlabel('Velocity (m/s)'); ylabel('Deceleration Force (G)');
title('Deceleration Resistance Profile', 'Color', 'w');
legend('Base Raw', 'Low Drag Raw', 'Base Aero Fit', 'Low Drag Aero Fit', 'TextColor', 'w', 'Box', 'off', 'Location', 'northwest');
grid on;

%% 5. Print Insights to Command Window
fprintf('\n--- AERODYNAMIC EFFECTIVENESS MATRIX ---\n');
fprintf('Baseline Drag Coefficient Factor (C2): %.6f\n', poly_base(1));
fprintf('Low Drag Drag Coefficient Factor (C2):  %.6f\n', poly_low(1));
drag_reduction = ((poly_base(1) - poly_low(1)) / poly_base(1)) * 100;
fprintf('Calculated Reduction in Aero Drag:    %.1f%%\n', drag_reduction);