%% Standalone Tire Stiffness Extractor
clear; clc;

% 1. Initialize environment paths (same as your main lapsim)
setup_paths; 

% 2. Run your team's configuration to generate the vehicle array
disp('Loading vehicle configuration...');
carCell = carConfig(); 
car = carCell{1,1}; % Grab the first car configuration in the array

% 3. Extract the pre-instantiated tire object and vehicle properties
myTire = car.tire;  % Reuses the exact Tire2 object created by parameters_loop
total_mass = car.M; % Total mass (car + driver) automatically computed by your code
g = 9.81;
total_weight = total_mass * g;

% Get weight distribution (assuming 0.512 rear bias from your config)
weight_dist_r = 0.512; 
weight_dist_f = 1 - weight_dist_r;

% Calculate static single-tire vertical loads (Newtons)
Fz_front_tire = (total_weight * weight_dist_f) / 2;
Fz_rear_tire  = (total_weight * weight_dist_r) / 2;

% 4. Compute Lateral Force at a tiny slip angle to isolate the linear slope
alpha_deg = 0.1; 
alpha_rad = alpha_deg * (pi / 180); % Convert slip angle to radians

% Call the F_y method directly from your tire object
% Syntax matching your Tire2 definition: F_y(alpha, kappa, F_z, gamma)
Fy_front = myTire.F_y(alpha_deg, 0, Fz_front_tire, 0);
Fy_rear  = myTire.F_y(alpha_deg, 0, Fz_rear_tire, 0);

% 5. Calculate Axle Stiffness (2 tires per axle)
Cf = 2 * (Fy_front / alpha_rad);
Cr = 2 * (Fy_rear / alpha_rad);

% Print the outputs to your Command Window
fprintf('\n====================================\n');
fprintf('  EXTRACTED TIRE STIFFNESS VALUES\n');
fprintf('====================================\n');
fprintf('Front Axle Stiffness (Cf): %7.2f N/rad\n', Cf);
fprintf('Rear Axle Stiffness (Cr):  %7.2f N/rad\n', Cr);
fprintf('====================================\n');