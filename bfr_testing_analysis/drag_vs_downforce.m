%% STEP 4: REAL TRACK 1D LAP TIME SIMULATOR (.MAT FILE)
clear; clc; close all;

%% 1. FILE PATH & VEHICLE PARAMETERS
track_file = 'track_autocross_2023.mat'; % MAT file path

% Tested Vehicle Parameters
m_total  = 230;      % Total vehicle mass + driver (kg)
P_engine = 30574;    % Measured Wheel Power (41.0 hp in Watts)
rho      = 1.225;    % Air density (kg/m^3)
g_acc    = 9.81;     % Gravity (m/s^2)
G_brake  = 1.40;     % Max longitudinal braking limit (g)

% Aero Setup Parameters
CdA_high   = 1.35;   G_lat_high = 1.57; % High Downforce Setup
CdA_low    = 1.05;   G_lat_low  = 1.46; % Low Drag Setup

%% 2. LOAD & PREPROCESS TRACK DATA FROM .MAT FILE
mat_contents = load(track_file);

% --- Extract Arclength (s) ---
if isfield(mat_contents, 's')
    s_vec = mat_contents.s;
elseif isfield(mat_contents, 'arclength')
    s_vec = mat_contents.arclength;
else
    fields = fieldnames(mat_contents);
    track_struct = mat_contents.(fields{1});
    if isstruct(track_struct) && isfield(track_struct, 's')
        s_vec = track_struct.s;
    elseif istable(track_struct) && ismember('s', track_struct.Properties.VariableNames)
        s_vec = track_struct.s;
    elseif isstruct(track_struct) && isfield(track_struct, 'arclength')
        s_vec = track_struct.arclength;
    elseif istable(track_struct) && ismember('arclength', track_struct.Properties.VariableNames)
        s_vec = track_struct.arclength;
    else
        error('Could not locate arclength/s in the .mat file.');
    end
end

% --- Extract Curvature (kappa) ---
if isfield(mat_contents, 'kappa')
    kappa_vec = abs(mat_contents.kappa);
elseif isfield(mat_contents, 'curvature')
    kappa_vec = abs(mat_contents.curvature);
elseif isfield(mat_contents, 'radius')
    kappa_vec = 1 ./ abs(mat_contents.radius);
else
    fields = fieldnames(mat_contents);
    track_struct = mat_contents.(fields{1});
    if isstruct(track_struct) && isfield(track_struct, 'kappa')
        kappa_vec = abs(track_struct.kappa);
    elseif istable(track_struct) && ismember('kappa', track_struct.Properties.VariableNames)
        kappa_vec = abs(track_struct.kappa);
    elseif isstruct(track_struct) && isfield(track_struct, 'curvature')
        kappa_vec = abs(track_struct.curvature);
    elseif istable(track_struct) && ismember('curvature', track_struct.Properties.VariableNames)
        kappa_vec = abs(track_struct.curvature);
    elseif isstruct(track_struct) && isfield(track_struct, 'radius')
        kappa_vec = 1 ./ abs(track_struct.radius);
    elseif istable(track_struct) && ismember('radius', track_struct.Properties.VariableNames)
        kappa_vec = 1 ./ abs(track_struct.radius);
    else
        error('Could not locate curvature/kappa/radius in the .mat file.');
    end
end

s_vec     = s_vec(:);
kappa_vec = kappa_vec(:);

kappa_vec(kappa_vec < 0.0001) = 0.0001; 

ds = mean(diff(s_vec)); % Track resolution step (meters)
num_points = length(s_vec);

%% 3. RUN VELOCITY PROFILE SOLVER FOR BOTH SETUPS
[t_lap_high, v_profile_high] = solve_track_lap(s_vec, kappa_vec, G_lat_high, CdA_high, P_engine, m_total, rho, g_acc, G_brake, ds);
[t_lap_low,  v_profile_low]  = solve_track_lap(s_vec, kappa_vec, G_lat_low,  CdA_low,  P_engine, m_total, rho, g_acc, G_brake, ds);

net_time_delta = t_lap_low - t_lap_high;

%% 4. DISPLAY RESULTS
fprintf('=========================================================================================\n');
fprintf('               REAL TRACK LAP TIME SIMULATION REPORT (STEP 4)                            \n');
fprintf('=========================================================================================\n');
fprintf('Track File Loaded              : %s\n', track_file);
fprintf('Track Length                   : %.2f meters (%.3f km)\n', max(s_vec), max(s_vec)/1000);
fprintf('Track Resolution (ds)          : %.3f meters (%d points)\n', ds, num_points);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('High Downforce Package Time    : %6.3f seconds | Avg Speed: %5.1f mph\n', t_lap_high, mean(v_profile_high)*2.23694);
fprintf('Low Drag Package Time          : %6.3f seconds | Avg Speed: %5.1f mph\n', t_lap_low,  mean(v_profile_low)*2.23694);
fprintf('-----------------------------------------------------------------------------------------\n');
if net_time_delta > 0
    fprintf('NET RESULT: HIGH DOWNFORCE IS FASTER BY %6.3f SECONDS ON THIS TRACK!\n', net_time_delta);
else
    fprintf('NET RESULT: LOW DRAG IS FASTER BY %6.3f SECONDS ON THIS TRACK!\n', abs(net_time_delta));
end
fprintf('=========================================================================================\n\n');

%% 5. PLOT SPEED PROFILE ALONG TRACK ARCLENGTH
figure('Name', 'Real Track Speed Profile Comparison', 'Position', [100, 100, 1100, 500]);
plot(s_vec, v_profile_high * 2.23694, 'b', 'LineWidth', 1.8, 'DisplayName', sprintf('High Downforce (%.3fs)', t_lap_high)); hold on;
plot(s_vec, v_profile_low * 2.23694,  'r--', 'LineWidth', 1.8, 'DisplayName', sprintf('Low Drag Setup (%.3fs)', t_lap_low));

xlabel('Track Distance / Arclength (meters)');
ylabel('Speed (mph)');
title('Velocity Profile Comparison Across Real Autocross Track (.mat File)');
grid on; legend('Location', 'northeast');

%% 6. HELPER FUNCTION: 3-PASS 1D LAP TIME SOLVER
function [t_lap, v_final] = solve_track_lap(s, kappa, G_lat, CdA, P, m, rho, g, G_brake, ds)
    N = length(s);
    
    % Pass 1: Cornering Speed Limit Curve
    v_limit = sqrt((G_lat * g) ./ kappa);
    
    % Pass 2: Forward Acceleration Pass
    v_fwd = zeros(N, 1);
    v_fwd(1) = v_limit(1);
    
    for i = 1:N-1
        v_curr = v_fwd(i);
        F_drag  = 0.5 * rho * (v_curr^2) * CdA;
        F_drive = min(P / max(v_curr, 1.0), 3000); % Traction limit cap
        F_net   = F_drive - F_drag;
        a_x     = F_net / m;
        
        v_next = sqrt(max(v_curr^2 + 2 * a_x * ds, 0.1));
        v_fwd(i+1) = min(v_next, v_limit(i+1));
    end
    
    % Pass 3: Backward Braking Pass
    v_bwd = zeros(N, 1);
    v_bwd(N) = v_fwd(N);
    
    for i = N:-1:2
        v_curr = v_bwd(i);
        F_drag = 0.5 * rho * (v_curr^2) * CdA;
        a_brake = (G_brake * g) + (F_drag / m);
        
        v_prev = sqrt(max(v_curr^2 + 2 * a_brake * ds, 0.1));
        v_bwd(i-1) = min(v_prev, v_fwd(i-1));
    end
    
    v_final = v_bwd;
    dt = ds ./ v_final;
    t_lap = sum(dt);
end


%{
%% STEP 4: COMPREHENSIVE AERO TRADE-OFF & REPORT GENERATOR
clear; clc; close all;

%% 1. INPUT PARAMETERS (TESTED TELEMETRY & COASTDOWN DATA)
% Vehicle & Engine Specs
m_total  = 230;      % Total vehicle mass + driver (kg)
P_engine = 30574;    % Measured Wheel Power (41.0 hp in Watts from RaceBox log)
rho      = 1.225;    % Air density (kg/m^3)
g_acc    = 9.81;     % Gravity acceleration (m/s^2)

% Aerodynamic Coefficients (From Coastdown Testing)
CdA_high = 1.35;     % m^2 (Baseline High Downforce / Drag Setup)
CdA_low  = 1.05;     % m^2 (Low Drag Configuration)

% =========================================================================
% CORNERING GRIP (G_LAT) DERIVATION EXPLANATION:
% 1. G_lat_high (1.57g): Measured directly from high-speed Skidpad (Circle 15 @ 27.6 mph).
% 2. G_base_mech (1.08g): Measured directly from low-speed Skidpad (Avg of lower speed skidpad runs),
%    where aero load is near zero, isolating pure mechanical tire grip.
% 3. Aero Grip Boost (Baseline): 1.57g - 1.08g = +0.49g gained purely from downforce.
% 4. G_lat_low (1.46g): Calculated for the untested Low Drag package by scaling 
%    the baseline aero grip boost (+0.49g) down by the coastdown drag reduction ratio 
%    (1.05 / 1.35 = 77.8% downforce retained -> +0.38g boost -> 1.08g + 0.38g = 1.46g).
% =========================================================================
G_lat_high  = 1.57;   % Peak Tested Grip (Baseline Aero @ 27.6 mph)
G_base_mech = 1.08;   % Low-Speed Mechanical Grip (Baseline Aero @ 22.3 mph)

% Calculated Grip for Low Drag Package (Assuming constant L/D ratio)
aero_grip_boost_base = G_lat_high - G_base_mech; % +0.49g downforce grip boost
G_lat_low = G_base_mech + (aero_grip_boost_base * (CdA_low / CdA_high)); % 1.46g

% Shockpot Telemetry Results (From Step 2 Heave/Pitch Analysis)
FL_squat_mm = +0.17; % mm (Front compression under aero load)
RL_lift_mm  = -2.08; % mm (Rear extension under high yaw cornering)

%% 2. TRACK LAYOUT DEFINITION (TYPICAL FSAE AUTOCROSS PROFILE)
num_corners     = 10;
corner_radius   = 12.0; % meters
corner_angle    = 90.0; % degrees per corner

num_straights   = 4;
straight_length = 75.0; % meters per straightaway

%% 3. CORNERING ANALYSIS
v_c_high = sqrt(G_lat_high * g_acc * corner_radius); % m/s
v_c_low  = sqrt(G_lat_low  * g_acc * corner_radius); % m/s

dist_1_corner = (corner_angle / 360.0) * (2 * pi * corner_radius); % meters
tot_corner_dist = num_corners * dist_1_corner;

t_c_1_high = dist_1_corner / v_c_high;
t_c_1_low  = dist_1_corner / v_c_low;

t_c_tot_high = num_corners * t_c_1_high;
t_c_tot_low  = num_corners * t_c_1_low;
t_saved_corners = t_c_tot_low - t_c_tot_high;

%% 4. STRAIGHTAWAY ANALYSIS (TWO CASES)
dt = 0.001; % 1ms integration step

% CASE A: Isolated Drag Penalty (Fixed Entry Speed = 12.0 m/s)
v_entry_fixed = 12.0; % m/s
[t_s_1_high_iso, v_top_high_iso] = sim_straight(straight_length, v_entry_fixed, CdA_high, P_engine, m_total, rho, dt);
[t_s_1_low_iso,  v_top_low_iso]  = sim_straight(straight_length, v_entry_fixed, CdA_low,  P_engine, m_total, rho, dt);

t_s_tot_high_iso = num_straights * t_s_1_high_iso;
t_s_tot_low_iso  = num_straights * t_s_1_low_iso;
t_drag_penalty_iso = t_s_tot_high_iso - t_s_tot_low_iso; % Pure drag time lost

% CASE B: Coupled Racing Reality (Entry Speed = Apex Speed)
[t_s_1_high_real, v_top_high_real] = sim_straight(straight_length, v_c_high, CdA_high, P_engine, m_total, rho, dt);
[t_s_1_low_real,  v_top_low_real]  = sim_straight(straight_length, v_c_low,  CdA_low,  P_engine, m_total, rho, dt);

t_s_tot_high_real = num_straights * t_s_1_high_real;
t_s_tot_low_real  = num_straights * t_s_1_low_real;
t_straight_delta_real = t_s_tot_low_real - t_s_tot_high_real; % Time gained/lost on straights with exit speed

%% 5. SENSITIVITY METRICS & RULES OF THUMB
% Grip Sensitivity: Time saved per +0.10g increase in lateral grip
v_c_base = sqrt(1.35 * g_acc * corner_radius);
v_c_p01  = sqrt(1.45 * g_acc * corner_radius);
t_saved_per_01g = (tot_corner_dist / v_c_base) - (tot_corner_dist / v_c_p01);

% Drag Sensitivity: Time lost per +0.10 m^2 CdA on 75m straights
[t_s_base_cda, ~] = sim_straight(straight_length, v_c_high, 1.25, P_engine, m_total, rho, dt);
[t_s_plus_cda, ~] = sim_straight(straight_length, v_c_high, 1.35, P_engine, m_total, rho, dt);
t_lost_per_01cda  = num_straights * (t_s_plus_cda - t_s_base_cda);

% Breakeven Threshold: Required +Δg to offset +0.10 m^2 CdA
breakeven_dg_per_01cda = (t_lost_per_01cda / t_saved_per_01g) * 0.10;

% ROI Ratio
roi_ratio = t_saved_corners / max(t_drag_penalty_iso, 0.001);

%% 6. PRINT DETAILED REPORT TERMINAL OUTPUT
fprintf('=========================================================================================\n');
fprintf('               AERODYNAMIC PERFORMANCE & DESIGN TRADE-OFF REPORT DATA                    \n');
fprintf('=========================================================================================\n');
fprintf('1. VEHICLE & TEST BENCH CONTEXT\n');
fprintf('   - Total Vehicle Mass:           %d kg (~507 lbs)\n', m_total);
fprintf('   - Engine Wheel Power:           41.0 hp (30,574 W) [KTM 450 Single]\n');
fprintf('   - Baseline High Downforce CdA:  %.2f m^2\n', CdA_high);
fprintf('   - Low Drag Setup CdA:          %.2f m^2 (-22.2%% Drag Reduction)\n', CdA_low);
fprintf('   - High Downforce Skidpad Grip:  %.2fg\n', G_lat_high);
fprintf('   - Low Downforce Skidpad Grip:   %.2fg\n', G_lat_low);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('2. CORNERING PERFORMANCE BREAKDOWN (%d Corners @ 12m Radius)\n', num_corners);
fprintf('   - Apex Speed (High Downforce):  %5.1f mph (%5.2f m/s)\n', v_c_high*2.23694, v_c_high);
fprintf('   - Apex Speed (Low Downforce):   %5.1f mph (%5.2f m/s)\n', v_c_low*2.23694, v_c_low);
fprintf('   - Time per Corner (High vs Low): %5.3f s  vs  %5.3f s\n', t_c_1_high, t_c_1_low);
fprintf('   - Total Corner Time Saved:      %5.3f seconds per lap (GAINED)\n', t_saved_corners);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('3. STRAIGHTAWAY TRADE-OFF BREAKDOWN (%d Straights @ 75m)\n', num_straights);
fprintf('   [CASE A: Isolated Drag Penalty at Same Entry Speed (12 m/s)]\n');
fprintf('   - Straight Time (High Drag):    %5.3f s\n', t_s_tot_high_iso);
fprintf('   - Straight Time (Low Drag):     %5.3f s\n', t_s_tot_low_iso);
fprintf('   - Pure Drag Time Penalty:       %5.3f seconds lost across %d straights\n', t_drag_penalty_iso, num_straights);
fprintf('\n   [CASE B: Coupled Racing Reality (Accounting for Higher Corner Exit Speed)]\n');
fprintf('   - Entry Speed into Straight:    %5.1f mph (High DF)  vs  %5.1f mph (Low Drag)\n', v_c_high*2.23694, v_c_low*2.23694);
fprintf('   - Top Speed at End of 75m:     %5.1f mph (High DF)  vs  %5.1f mph (Low Drag)\n', v_top_high_real*2.23694, v_top_low_real*2.23694);
fprintf('   - Straight Time (High DF):      %5.3f s\n', t_s_tot_high_real);
fprintf('   - Straight Time (Low Drag):     %5.3f s\n', t_s_tot_low_real);
fprintf('   - Net Straight Time Delta:      %5.3f s (High DF is FASTER on straights due to exit speed!)\n', t_straight_delta_real);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('4. OVERALL TRADE-OFF & DESIGN SENSITIVITY METRICS\n');
fprintf('   - Time ROI Exchange Ratio:      %5.1f : 1 (Gained in corners vs Pure Drag lost)\n', roi_ratio);
fprintf('   - Net Lap Time Delta:           High DF is %5.3f seconds FASTER per lap\n', (t_c_tot_low + t_s_tot_low_real) - (t_c_tot_high + t_s_tot_high_real));
fprintf('   - Grip Sensitivity:             Every +0.10g grip saves ~%5.3f s/lap\n', t_saved_per_01g);
fprintf('   - Drag Sensitivity:             Every +0.10 m^2 CdA costs ~%5.3f s/lap on straights\n', t_lost_per_01cda);
fprintf('   - Design Breakeven Rule:        Adding +0.10 m^2 CdA is worth it if it yields > +%5.4fg grip\n', breakeven_dg_per_01cda);
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('5. TELEMETRY SHOCKPOT BALANCE SUMMARY\n');
fprintf('   - Front Suspension Aero Squat:  %+.2f mm (Front wing actively loaded)\n', FL_squat_mm);
fprintf('   - Rear Suspension Aero Lift:   %+.2f mm (Rear wing loses downforce in yaw/cornering)\n', RL_lift_mm);
fprintf('   - Aero Balance Insight:         Car pitches forward in steady-state corners.\n');
fprintf('=========================================================================================\n');

%% HELPER FUNCTIONS
function [t_end, v_end] = sim_straight(d_target, v0, CdA, P, m, rho, dt)
    x = 0; v = v0; t = 0;
    while x < d_target
        F_aero = 0.5 * rho * (v^2) * CdA;
        F_drive = min(P / max(v, 1.0), 3000); % Traction limit cap
        F_net = F_drive - F_aero;
        a = F_net / m;
        v = v + a * dt;
        x = x + v * dt;
        t = t + dt;
    end
    t_end = t; v_end = v;
end
%}