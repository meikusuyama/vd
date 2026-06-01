%Based on the car generated using SteadyStateLapsim
%Run SteadyStateLapsim, then select the car to base off of below

clear; clc;

% loads carCell from SteadyState lapsim into workspace
load('DesignBinderFinalDriveSweep2.mat')
car = carCell{1,1};    % choose a car


%% ---------------- USER INPUT ----------------
g = car.g;

target_g = 1.25;                % select lateral acceleration to look at
P_target = car.tire.p_i;        % tire pressure filter
P_tol    = 0.5;

g_tol = 0.05;          % tolerance around target g
FZ_tol = 10;           % lb
SA_tol = 0.3;          % deg
IA_tol = 0.2;          % deg
N_to_lb = 0.224809;

% Load tire dataset
load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

%% =============================================================
% STEP 1 — Find representative index at target lateral acceleration
%% =============================================================

lat_g_all = car.comp.interp_info.x_table_corner_vel.lat_accel / g;
vel_all   = car.comp.interp_info.x_table_corner_vel.long_vel;

% --- 1) Filter by target lateral g ---
idx_g = abs(lat_g_all - target_g) < g_tol;

vel_selected = vel_all(idx_g);
indices_selected = find(idx_g);   % preserve original indices

%% -------------------------------------------------
% STEP 1A — Bin velocities (0.5 m/s bins)
%% -------------------------------------------------

bin_width = 0.5;

v_min = floor(min(vel_selected));
v_max = ceil(max(vel_selected));
edges = v_min : bin_width : v_max;

[counts, edges] = histcounts(vel_selected, edges);

% Most populated bin
[~, maxBinIdx] = max(counts);

bin_lower = edges(maxBinIdx);
bin_upper = edges(maxBinIdx+1);

% Logical mask inside selected bin
in_bin = vel_selected >= bin_lower & vel_selected < bin_upper;

vel_bin = vel_selected(in_bin);
indices_bin = indices_selected(in_bin);   % ORIGINAL indices

%% -------------------------------------------------
% STEP 1B — Find most representative velocity
%% -------------------------------------------------

% Round inside bin to 0.1 m/s
vel_bin_rounded = round(vel_bin, 1);

% Mode of rounded values
V_common = mode(vel_bin_rounded);

% Now find the row whose velocity is closest to that mode
[~, best_local_idx] = min(abs(vel_bin - V_common));

% This is the FINAL index in original dataset
best_idx = indices_bin(best_local_idx);

a_y = target_g * g;

fprintf("Target g: %.2f g\n", target_g)
fprintf("Most populated velocity bin: %.2f–%.2f m/s\n", bin_lower, bin_upper)
fprintf("Representative velocity: %.2f m/s\n", vel_all(best_idx))
fprintf("Selected row index: %d\n", best_idx)


%% =============================================================
% STEP 2 — Extract tire loads and slip angles
%% =============================================================

data = car.comp.interp_info.x_table_corner_vel;

FZ_outside_f = data.Fz_1(best_idx);
FZ_inside_r  = data.Fz_2(best_idx);
FZ_outside_r = data.Fz_3(best_idx);
FZ_inside_f  = data.Fz_4(best_idx);

SA_outside_f = data.alpha_1(best_idx);
SA_inside_r  = data.alpha_2(best_idx);
SA_outside_r = data.alpha_3(best_idx);
SA_inside_f  = data.alpha_4(best_idx);

fprintf("\nTire Loads (N):\n")
fprintf("  front_outside : %.1f\n", FZ_outside_f)
fprintf("  front_inside  : %.1f\n", FZ_inside_f)
fprintf("  rear_outside  : %.1f\n", FZ_outside_r)
fprintf("  rear_inside   : %.1f\n", FZ_inside_r)

fprintf("\nTire Loads (lb):\n")
fprintf("  front_outside : %.1f\n", FZ_outside_f * N_to_lb)
fprintf("  front_inside  : %.1f\n", FZ_inside_f * N_to_lb)
fprintf("  rear_outside  : %.1f\n", FZ_outside_r * N_to_lb)
fprintf("  rear_inside   : %.1f\n", FZ_inside_r * N_to_lb)


fprintf("\nSlip Angles (deg):\n")
fprintf("  front_outside : %.3f\n", SA_outside_f)
fprintf("  front_inside  : %.3f\n", SA_inside_f)
fprintf("  rear_outside  : %.3f\n", SA_outside_r)
fprintf("  rear_inside   : %.3f\n", SA_inside_r)



%% =============================================================
% Step 3 — Camber that Minimizes Mx
% =============================================================

figure; hold on;

% ---- 1) Get slip angle range from lapsim results ----
SA_all = abs([ ...
    SA_outside_f, ...
    SA_inside_f, ...
    SA_outside_r, ...
    SA_inside_r]);

SA_min = min(SA_all);
SA_max = max(SA_all);

%% =============================================================
% STEP A — Build Camber Map from TTC Loads
% =============================================================

% TTC discrete loads (lb, positive magnitude)
FZ_TTC = [50 100 150 200 250];

% Slip angle sweep (positive)
SA_sweep = linspace(1, SA_max, 10);

cmap = turbo(length(SA_sweep));

% Storage: rows = slip angles, cols = FZ loads
IA_map = zeros(length(SA_sweep), length(FZ_TTC));

for t = 1:length(FZ_TTC)

    FZ_target = -FZ_TTC(t);   % TTC dataset uses negative

    maskFZ = abs(FZ - FZ_target) < FZ_tol;
    maskP  = abs(P - P_target) < P_tol;

    SA_f = SA(maskFZ & maskP);
    IA_f = IA(maskFZ & maskP);
    MX_f = MX(maskFZ & maskP);

    if numel(SA_f) < 50
        warning("Not enough TTC data for FZ %.1f", FZ_target);
        continue
    end

    F_Mx_local = scatteredInterpolant(SA_f, IA_f, MX_f, ...
        'natural', 'none');

    IA_candidates = linspace(min(IA_f), max(IA_f), 400);

    for s = 1:length(SA_sweep)

        SA_target = SA_sweep(s);

        Mx_vals = F_Mx_local( ...
            SA_target * ones(size(IA_candidates)), ...
            IA_candidates);

        valid = ~isnan(Mx_vals);
        IA_valid = IA_candidates(valid);
        Mx_valid = Mx_vals(valid);

        if isempty(Mx_valid)
            IA_map(s,t) = NaN;
            continue
        end

        [~, idx] = min(abs(Mx_valid));
        IA_map(s,t) = IA_valid(idx);
    end
end


%% =============================================================
% STEP B — Plot FZ vs IA based on TTC tire data
% =============================================================

figure; hold on;

for s = 1:length(SA_sweep)
    plot(-FZ_TTC, IA_map(s,:), '-o', ...
        'Color', cmap(s,:), ...
        'LineWidth', 1.8);
end

xlabel('Vertical Load FZ (lb)')
ylabel('Camber IA that Minimizes Mx (deg)')
title(sprintf('Tire Camber Demand Map (%.2f psi)', P_target))
grid on;

cb = colorbar;
cb.Label.String = 'Slip Angle (deg)';
colormap(turbo);
caxis([min(SA_sweep) max(SA_sweep)]);

%% =============================================================
% STEP C — Required Camber for Outside Tires
% =============================================================

FZ_sim = abs([FZ_outside_f, FZ_outside_r] * N_to_lb);

SA_sim = abs([SA_outside_f, SA_outside_r]);

IA_required = zeros(1,2);

for i = 1:2

    SA_target = SA_sim(i);
    IA_vs_FZ = zeros(size(FZ_TTC));

    for t = 1:length(FZ_TTC)

        FZ_target = -FZ_TTC(t);

        maskFZ = abs(FZ - FZ_target) < FZ_tol;
        maskP  = abs(P - P_target) < P_tol;

        SA_f = SA(maskFZ & maskP);
        IA_f = IA(maskFZ & maskP);
        MX_f = MX(maskFZ & maskP);

        if numel(SA_f) < 50
            IA_vs_FZ(t) = NaN;
            continue
        end

        F_Mx_local = scatteredInterpolant(SA_f, IA_f, MX_f, ...
            'natural', 'none');

        IA_candidates = linspace(min(IA_f), max(IA_f), 400);

        Mx_vals = F_Mx_local( ...
            SA_target * ones(size(IA_candidates)), ...
            IA_candidates);

        valid = ~isnan(Mx_vals);
        IA_valid = IA_candidates(valid);
        Mx_valid = Mx_vals(valid);

        if isempty(Mx_valid)
            IA_vs_FZ(t) = NaN;
            continue
        end

        [~, idx] = min(abs(Mx_valid));
        IA_vs_FZ(t) = IA_valid(idx);
    end

    % Remove NaNs before interpolating
    valid = ~isnan(IA_vs_FZ);
    num_valid = sum(valid);
    
    if num_valid >= 2
        
        IA_required(i) = interp1( ...
            FZ_TTC(valid), ...
            IA_vs_FZ(valid), ...
            FZ_sim(i), ...
            'pchip', ...
            'extrap');
        
    elseif num_valid == 1
        
        % Only one TTC load had valid solution
        IA_required(i) = IA_vs_FZ(valid);
        
    else
        
        % No valid solutions
        IA_required(i) = NaN;
        
    end

end

fprintf("\nRequired Camber to Minimize Mx at %.2fg:\n", target_g)
fprintf("  Front Outside : %.3f deg\n", IA_required(1))
fprintf("  Rear Outside  : %.3f deg\n", IA_required(2))




%% =============================================================
% Visualize 3D Mx Surface for 150lb
%% =============================================================

% ---- Pick sample load ----
FZ_debug = -150;

maskFZ = abs(FZ - FZ_debug) < FZ_tol;
maskP = abs(P - P_target) < P_tol;        % tire pressure filter


SA_f = SA(maskFZ & maskP);
IA_f = IA(maskFZ & maskP);
MX_f = MX(maskFZ & maskP);

if numel(SA_f) < 50
    error("Not enough data for debug surface.")
end

% Build interpolant
F_Mx_debug = scatteredInterpolant(SA_f, IA_f, MX_f, ...
    'natural', 'none');

% Create grid
SA_grid = linspace(min(SA_f), max(SA_f), 60);
IA_grid = linspace(min(IA_f), max(IA_f), 60);

[SA_mesh, IA_mesh] = meshgrid(SA_grid, IA_grid);

MX_mesh = F_Mx_debug(SA_mesh, IA_mesh);

%% ---- Plot surface ----
figure;
surf(SA_mesh, IA_mesh, MX_mesh, ...
    'EdgeColor', 'none');

hold on;

% Overlay raw data points
scatter3(SA_f, IA_f, MX_f, ...
    10, 'w', 'filled');

xlabel('Slip Angle (deg)')
ylabel('Camber IA (deg)')
zlabel('Mx (lb-in)')
title(sprintf('Mx Surface @ FZ = %.1f lb', abs(FZ_debug)))
colorbar
grid on
view(135, 30)


%% =============================================================
% Roll Angle Data
%% =============================================================
roll_gradients = [0.41, 0.49, 0.54, 0.60, 0.64, 0.67, 0.70];
settings = {'Short-MR2', 'Med-MR2', 'Long-MR2', 'Short-MR1', 'Med-MR1', 'Long-MR1', 'Off'};

% Calculate Roll Angles (phi = Gradient * ay (in g))
roll_angles = roll_gradients .* target_g;

% Display Results
fprintf('\n\nChassis Roll Angle at %.2fg:\n', target_g);
fprintf('------------------------------------\n');
for i = 1:length(settings)
    fprintf('%-12s : %.4f degrees\n', settings{i}, roll_angles(i));
end


