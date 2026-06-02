data = readtable('alameda_skipad2_2026-05-03T16-45-03.csv');

lat_g = filtfilt(b, a, fillmissing(data.ACCELY, 'linear'));
yaw_rate = filtfilt(b, a, fillmissing(data.ANGRATEZ, 'linear'));
steer = filtfilt(b, a, fillmissing(data.STEERINGANGLE, 'linear'));

figure;
xlabel('Time'); ylabel('Lateral G');
scatter(data.Time, lat_g);

figure;
xlabel('Time'); ylabel('Yaw Rate');
scatter(data.Time, yaw_rate);

figure;
xlabel('Time'); ylabel('Steer Angle');
scatter(data.Time, steer);



%{
%% FSAE Skidpad Analysis Script
fs = 200; % Your 0.005 interval = 200Hz
idx = data.Time > 700; % Keeping your time filter

% --- 1. Signal Cleaning ---
[b, a] = butter(2, 5/(fs/2)); % 5Hz filter to remove engine vibration
lat_g = filtfilt(b, a, fillmissing(data.ACCELY, 'linear'));
yaw_rate = filtfilt(b, a, fillmissing(data.ANGRATEZ, 'linear'));
steer = filtfilt(b, a, fillmissing(data.STEERINGANGLE, 'linear'));

% --- 2. Calculate Derived Metrics ---
% Calculate "Understeer Gradient" (Steer vs LatG)
% Ideally, this is a linear slope.
[p, S] = polyfit(lat_g(idx), steer(idx), 1); 

%% 3. Visualization Dashboard
figure;

% --- TOP LEFT: The "Circle" (Yaw Rate vs Steering) ---
% Shows if the driver is having to "saw" at the wheel
subplot(2,2,1);
scatter(steer(idx), yaw_rate(idx), 10, lat_g(idx), 'filled');
xlabel('Steering Angle (deg)'); ylabel('Yaw Rate (deg/s)');
title('Steering Input vs. Car Rotation');
colorbar; colormap('jet'); grid on;

% --- TOP RIGHT: Lateral G vs. Time ---
% Shows the "Limit" of the car
subplot(2,2,2);
plot(data.Time(idx), lat_g(idx), 'LineWidth', 1.5);
hold on;
yline(mean(lat_g(idx)), 'r--', 'Avg Grip');
ylabel('Lateral Accel (G)'); xlabel('Time (s)');
title('Steady State Lateral Grip');
grid on;

% --- BOTTOM LEFT: Understeer Gradient ---
% If the line curves UP, the car is understeering.
subplot(2,2,3);
plot(lat_g(idx), steer(idx), '.');
hold on;
plot(lat_g(idx), polyval(p, lat_g(idx)), 'r-', 'LineWidth', 2);
xlabel('Lateral G'); ylabel('Steering Angle');
title('Understeer Characteristic');
grid on;

% --- BOTTOM RIGHT: Chassis Roll Rate ---
subplot(2,2,4);
plot(data.Time(idx), data.ANGRATEX(idx), 'Color', [0.5 0.5 0.5]);
ylabel('Roll Rate (deg/s)'); xlabel('Time (s)');
title('Body Roll Stability');
grid on;

%}