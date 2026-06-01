
lat_g = car.comp.autocross.lat_accel / 9.81;   % convert to g's
vel = car.comp.autocross.long_vel;             % longitudinal velocity

% ---- Plot histogram for common Lateral Acceleration ----
figure;
histogram(abs(lat_g), 'BinWidth', 0.05); % adjust BinWidth as needed
xlabel('Lateral Acceleration (in g)');
ylabel('Frequency');
title("Frequency of Lateral Accelerration in Autocross");
grid on;



% ---- Some g range ----
g_min = 1.24;
g_max = 1.26;

% ---- Find indices where lateral g is within that range ----
idx = lat_g >= g_min & lat_g <= g_max;

% ---- Extract corresponding velocities ----
vel_selected = vel(idx);

% ---- Plot histogram ----
figure;
histogram(vel_selected, 'BinWidth', 0.1); % adjust BinWidth as needed
xlabel('Longitudinal Velocity [m/s]');
ylabel('Frequency');
title(sprintf('Velocity Distribution for %.1f–%.1f g Lateral Acceleration', g_min, g_max));
grid on;

% Round velocity to 1 decimal place
vel_rounded = round(vel_selected, 1);

% Find the mode (most frequent value)
V_common = mode(vel_rounded);

display(V_common);

