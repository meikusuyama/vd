% Load the data file referenced in your code
%load('michigantrack2024.mat'); 
load('michigantrack2024.mat');

% Open a figure to inspect the track's curvature profile
figure;
plot(arclength, curvature, 'w', 'LineWidth', 1.5);
grid on;
xlabel('Distance Along Track');
ylabel('Curvature (\kappa = 1/R)');
title('2024 Michigan Autocross Curvature Map');