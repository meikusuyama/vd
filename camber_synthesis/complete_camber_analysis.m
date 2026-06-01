%% =============================================================
% End-to-end Camber Synthesis Pipeline
% Input: target lateral acceleration in g
% Output:
%   - Common velocity at that g
%   - 4 tire Fz loads
%   - Slip angle (RH, LH, averaged) for each tire
%   - Final plot: IA vs FZ curve (Mx≈0) for each tire
%   - Printed table of FZ and SA for each tire
%
% Requirements in workspace:
%   car.comp.autocross.lat_accel
%   car.comp.autocross.long_vel
%   Tire data MAT file with SA, FY, FZ, MX, IA, P
%
%% =============================================================


clear; clc;

% loads carCell from SteadyState lapsim into workspace
load('DesignBinderFinalDriveSweep2.mat')
car = carCell{1,1};    % get the first car


%% ---------------- USER INPUT ----------------
g = 9.81;

target_g = 1.25;      % <-- change as needed
P_target = 12;        % tire pressure filter
%P_target = car.tire.p_i;   FROM Lapsim
P_tol    = 0.5;

g_tol = 0.05;          % tolerance around target g
FZ_tol = 10;           % lb
SA_tol = 0.3;          % deg
IA_tol = 0.2;          % deg

% Load tire dataset
load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

%% =============================================================
% STEP 1 — Find common velocity at target lateral acceleration
%% =============================================================

lat_g = car.comp.autocross.lat_accel / g;
vel   = car.comp.autocross.long_vel;

% Filter at target lateral g
idx = abs(lat_g - target_g) < g_tol;
vel_selected = vel(idx);

%% -------------------------------------------------
% STEP 1A — Bin velocities in 0.5 m/s increments
%% -------------------------------------------------

bin_width = 0.5;

v_min = floor(min(vel_selected));
v_max = ceil(max(vel_selected));

edges = v_min : bin_width : v_max;

[counts, edges] = histcounts(vel_selected, edges);

% Find most populated bin
[~, maxBinIdx] = max(counts);

bin_lower = edges(maxBinIdx);
bin_upper = edges(maxBinIdx + 1);

% Extract velocities within most populated bin
in_bin = vel_selected >= bin_lower & vel_selected < bin_upper;
vel_bin = vel_selected(in_bin);

%% -------------------------------------------------
% STEP 1B — Round within bin and find mode
%% -------------------------------------------------

vel_bin_rounded = round(vel_bin, 1);
V_common = mode(vel_bin_rounded);

a_y = target_g * g;

fprintf("Target g: %.2f g\n", target_g)
fprintf("Most populated velocity bin: %.2f–%.2f m/s\n", bin_lower, bin_upper)
fprintf("Common velocity (mode within bin): %.2f m/s\n", V_common)
fprintf("Target pressure: %.2f psi\n", P_target)


%% =============================================================
% STEP 2 — Compute FZ at each tire
%% =============================================================

[carCell, carParams, aeroParams] = carConfig();
car = carCell{1,1};
Fz_struct = computeFzLoads(carParams, aeroParams, a_y, V_common, car);

tireNames = fieldnames(Fz_struct);
FZ_tires = struct2array(Fz_struct);   % [front_out front_in rear_out rear_in]

fprintf("\nComputed tire loads (N):\n")
for i = 1:4
    fprintf("  %-14s : %.1f\n", tireNames{i}, FZ_tires(i))
end

% convert to lb
N_to_lb = 0.224809;

FZ_tires = FZ_tires * N_to_lb;

fprintf("\nComputed tire loads (lb):\n")
for i = 1:4
    fprintf("  %-14s : %.1f\n", tireNames{i}, FZ_tires(i))
end


%% =============================================================
% STEP 3 — Filter tire data by pressure
%% =============================================================

maskP = abs(P - P_target) < P_tol;
SA = SA(maskP);
FY = FY(maskP);
FZ = FZ(maskP);
MX = MX(maskP);
IA = IA(maskP);

%% =============================================================
% STEP 4 — For each tire, find slip angle that maximizes |FY|
%% =============================================================

SA_results = struct();

figure; hold on;

for t = 1:4

    FZ_target = -FZ_tires(t);   % dataset uses negative FZ

    maskFZ = abs(FZ - FZ_target) < FZ_tol;
    SA_f = SA(maskFZ);
    FY_f = FY(maskFZ);

    % RH and LH turns
    neg = SA_f < 0;
    pos = SA_f > 0;

    SA_RH = NaN; SA_LH = NaN;

    if any(neg)
        neg_SA = SA_f(neg);
        neg_FY = FY_f(neg);
        [~,iR] = max(abs(neg_FY));
        SA_RH = abs(neg_SA(iR));
        FY_RH = neg_FY(iR);
    end
    
    if any(pos)
        pos_SA = SA_f(pos);
        pos_FY = FY_f(pos);
        [~,iL] = max(abs(pos_FY));
        SA_LH = pos_SA(iL);
        FY_LH = pos_FY(iL);
    end


    SA_avg = mean([SA_RH SA_LH],'omitnan');

    SA_results.(tireNames{t}).RH = SA_RH;
    SA_results.(tireNames{t}).LH = SA_LH;
    SA_results.(tireNames{t}).avg = SA_avg;

    fprintf("\n%s:\n", tireNames{t});
    fprintf("  RH SA = %.3f deg\n", SA_RH);
    fprintf("  LH SA = %.3f deg\n", SA_LH);
    fprintf("  Avg SA = %.3f deg\n", SA_avg);

    % Plot SA vs FY
    scatter(SA_f, FY_f, 8, [0.7 0.7 0.7]);
    plot(-SA_RH, FY_RH, 'ro', 'MarkerSize', 8, 'LineWidth', 2)
    plot(SA_LH,  FY_LH, 'bo', 'MarkerSize', 8, 'LineWidth', 2)

end

xlabel('Slip Angle (deg)')
ylabel('FY (lb)')
title(sprintf('SA vs FY at Target g = %.2f', target_g))
grid on;



%% =============================================================
% STEP 5 — Cobb-style camber curves (many SA, 4 Fz each)
%% =============================================================

% Build full interpolant
F_Mx = scatteredInterpolant(SA, IA, FZ, MX, 'natural', 'none');

% Slip angle range
SA_peaks = structfun(@(s) s.avg, SA_results);
SA_max = max(SA_peaks);
SA_set = 0 : 1 : SA_max;

figure; hold on;
cmap = turbo(length(SA_set));

for i = 1:length(SA_set)

    SA_target = SA_set(i);

    IA_zero_list = [];
    FZ_list = [];

    FZ_set = [50 100 150 200 250];   % lb, positive magnitude

    for f = 1:length(FZ_set)
        FZ_target = -FZ_set(f);   % dataset uses negative

        IA_candidates = linspace(min(IA), max(IA), 400);

        Mx_vals = F_Mx( ...
            SA_target * ones(size(IA_candidates)), ...
            IA_candidates, ...
            FZ_target * ones(size(IA_candidates)) );

        valid = ~isnan(Mx_vals);
        IA_candidates = IA_candidates(valid);
        Mx_vals = Mx_vals(valid);

        if isempty(Mx_vals)
            continue
        end

        % find camber angle that minimizes Mx (close to 0)
        [~, idx] = min(abs(Mx_vals));
        IA_opt = IA_candidates(idx);

        IA_zero_list(end+1) = IA_opt;
        FZ_list(end+1) = FZ_target;
    end

    if numel(FZ_list) >= 2
        [FZ_sorted, idx] = sort(FZ_list);
        IA_sorted = IA_zero_list(idx);
        
        plot(FZ_sorted, IA_sorted, '-o', ...
            'Color', cmap(i,:), ...
            'LineWidth', 1.2);
    end
end

xlabel('Vertical Load FZ (lb)')
ylabel('Optimal Camber IA (deg)')
title(sprintf('Camber Curves via Mx Zero-Crossing (%.2f psi)', P_target))
grid on;

cb = colorbar;
cb.Label.String = 'Slip Angle (deg)';
colormap(turbo);
caxis([min(SA_set) max(SA_set)]);



%% =============================================================
% LOAD FUNCTION
%% =============================================================

function Fz = computeFzLoads(carParams, aeroParams, a_y, V, car)

    g = car.g;

    m_total = carParams.mass + carParams.driver_weight;
    W_total = m_total * g;

    W_rear  = W_total * carParams.weight_dist;
    W_front = W_total * (1 - carParams.weight_dist);

    rho = car.aero.rho;
    D_total = 0.5 * rho * V^2 * aeroParams.cla;
    D_front = D_total * aeroParams.distribution;
    D_rear  = D_total * (1 - aeroParams.distribution);

    W_front = W_front + D_front;
    W_rear  = W_rear  + D_rear;

    h = carParams.cg_height;
    t = carParams.track_width;
    R_sf = carParams.R_sf;

    dFz_total = (a_y / g) * (h / t) * W_total;
    dFz_front = dFz_total * R_sf;
    dFz_rear  = dFz_total * (1 - R_sf);

    Fz.front_outside = W_front/2 + dFz_front/2;
    Fz.front_inside  = W_front/2 - dFz_front/2;
    Fz.rear_outside  = W_rear/2  + dFz_rear/2;
    Fz.rear_inside   = W_rear/2  - dFz_rear/2;
end

