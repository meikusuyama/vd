
data = readtable('Alameda_Endurance_Stint_1_LL_2026-04-26T05-41-55.csv');

figure;
%{
subplot(2,2,1); % Top Left
% scatter(x, y, size, color)
scatter(data.ACCELY, data.ACCELX, 10, 'filled');
xlabel('Lateral Accel'); ylabel('Longitudinal Accel');
title('G-G Diagram');
grid on; axis equal;
colorbar; % Adds the color scale 
%}


idx = data.Time > 0; 

% --- NOISE REDUCTION SECTION ---
fs = 100; 
fc = 5;   % Cutoff frequency 

% Create the filter coefficients
[b, a] = butter(2, fc/(fs/2));

% Apply the filter to each wheel speed
% fillmissing ensures the filter doesn't crash on NaNs
WSFL_smooth = filtfilt(b, a, fillmissing(data.WheelSpeedFrontLeft, 'linear'));
WSFR_smooth = filtfilt(b, a, fillmissing(data.WheelSpeedFrontRight, 'linear'));

% Now update your index for the smoothed data
WSFL_plot = WSFL_smooth(idx);
WSFR_plot = WSFR_smooth(idx);

% --- Wheel Speed ---
axes('Position',[0.10 0.50 0.83 0.45]);

% Ensure you are plotting the smoothed/filtered variables 
plot(data.Time(idx), WSFL_plot, 'r', 'LineWidth', 1.2); hold on;
plot(data.Time(idx), WSFR_plot, 'b', 'LineWidth', 1.2);

ylabel('Wheel Speed'); 
grid on; 
xticklabels({}); % Keeps the stack tight by hiding middle X-axis labels

% --- THE LEGEND CODE ---
% 'Location', 'best' tells MATLAB to move it so it doesn't block the lines
% 'Box', 'off' removes the white border for a cleaner look
legend('FL Wheel Speed', 'FR Wheel Speed', ...
       'Location', 'northeast', ...
       'Box', 'off', ...
       'FontSize', 9);


% --- Steering Angle (Wide) ---
axes('Position',[0.10 0.10 0.83 0.35]);
plot(data.Time(idx), data.STEERINGANGLE(idx), 'm', 'LineWidth', 1.2);
ylabel('Steer (deg)'); xlabel('Time (s)'); 
grid on;
legend('Steering Angle', 'Location', 'northeast', 'Box', 'off');
