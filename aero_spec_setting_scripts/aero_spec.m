%% ========================================================================
% FSAE AERO SPECIFICATION - FAST TIRE-AWARE AERO TARGET MODEL (v3)
%
% CHANGES FROM v2 (search "NEW v3:" to find every addition):
%
%   1) REAL POWERTRAIN FOR ACCEL EVENT
%      Replaces the Pmax/V placeholder with your actual Powertrain class
%      + KTM450 torque curve: real gear ratios, shift time, drivetrain
%      efficiency. This was the single biggest source of error (accel
%      was beating the WORLD RECORD by 0.37s in v2).
%
%   2) FRICTION-ELLIPSE (COMBINED SLIP) COUPLING
%      Tires can no longer deliver 100% lateral grip while simultaneously
%      fighting drag/accelerating/braking. This is what was making
%      skidpad faster than the best-ever recorded run. Approximated as
%      sqrt(1-(F_used/F_max)^2) per axle, looked up from the same Fz-based
%      tables you already had -- NOT full Magic Formula combined slip,
%      but a large step up from fully decoupled Fx/Fy.
%
%   3) CALIBRATION LAYER
%      Four constants (ACTUAL_*) hold your team's real times. The model
%      computes a calibration ratio = actual/model at the CURRENT
%      baseline aero, then applies it to every sweep result. Physics
%      still drives the CLA/CdA/CoP *trends*; calibration fixes the
%      *absolute* numbers so they're comparable to your comp results.
%      UPDATE THESE FOUR NUMBERS EVERY SEASON.
%
% STILL OMITTED (judged not worth it -- see chat): yaw transients,
% ackermann, camber compliance, full Magic Formula combined slip,
% fmincon-based g-g optimization. These are second-order next to the
% three fixes above.
%
%% ========================================================================

clc;
clear;
close all;

%% ========================================================================
% 1. CALIBRATION DATA -- DISABLED FOR NOW
%
% Set ENABLE_CALIBRATION = true and fill in ACTUAL_* below once you've
% decided on a consistent way to pick representative comp times (year-
% to-year and driver/condition variance make this a judgment call, not
% just "use most recent"). Until then this section is a no-op: every
% CAL_* factor is fixed at 1.0, so every number below is raw physics.
% ========================================================================

ENABLE_CALIBRATION = false;

ACTUAL_AUTOX_TIME       = 46.189;       %B24
ACTUAL_ENDURANCE_LAP    = 1418.241/10;  %B24
ACTUAL_SKIDPAD_TIME     = 5.045;        %B26
ACTUAL_ACCEL_TIME       = 5.236;        %B26 (B24 = 4.99, B25 = 4.862)

%% ========================================================================
% 2. BASELINE AERO + MASS
%% ========================================================================

BASE_CLA = 2.8;       % m^2 -- baseline package (auto/skid/endurance)
BASE_CDA = 1.35;       % m^2

BASE_COP_FRONT = 0.47;
BASE_COP_REAR  = 1 - BASE_COP_FRONT;

BASE_CLA_ACCEL = 2.10;      % m^2, accel package starting point
BASE_CDA_ACCEL = 1.01;      % m^2
BASE_COP_FRONT_ACCEL = 0.63;

BASE_CAR_MASS      = 162.8;   % kg, car only (no driver)
BASE_DRIVER_MASS   = 64;      % kg
ACCEL_DRIVER_MASS  = 59;      % kg

fprintf('\n');
fprintf('===============================================================\n');
fprintf('           FSAE FAST AERO SPECIFICATION MODEL (v3)\n');
fprintf('===============================================================\n');

fprintf('\nBaseline package (auto/skid/endurance):\n');
fprintf('  CLA       = %.2f m^2\n',BASE_CLA);
fprintf('  CdA       = %.2f m^2\n',BASE_CDA);
fprintf('  CoP       = %.1f%% front / %.1f%% rear\n', ...
    100*BASE_COP_FRONT,100*BASE_COP_REAR);

fprintf('\nAccel package (starting point, to be swept):\n');
fprintf('  CLA_accel = %.2f m^2\n',BASE_CLA_ACCEL);
fprintf('  CdA_accel = %.2f m^2\n',BASE_CDA_ACCEL);
fprintf('  CoP_accel = %.1f%% front\n',100*BASE_COP_FRONT_ACCEL);

%% ========================================================================
% 3. VEHICLE PARAMETERS
%% ========================================================================

rho = 1.225;
g = 9.81;

M       = BASE_CAR_MASS + BASE_DRIVER_MASS;    % kg
M_ACCEL = BASE_CAR_MASS + ACCEL_DRIVER_MASS;   % kg

Wb = 62*0.0254;             % m
track = 47*0.0254;          % m

weight_dist_rear = 0.521;

lf = Wb*weight_dist_rear;
lr = Wb*(1-weight_dist_rear);

hg = 0.281;

hrf = 0.0876;
hrr = 0.0914;
hrc = (hrf+hrr)/2;

Rsf = 0.34;

wheel_radius = 0.127;   % NEW v3: needed for engine_rpm <-> wheel speed

%% ========================================================================
% 4. MODEL SETTINGS
%% ========================================================================

N_AUTO = 1500;
N_END  = 1500;

N_ITER = 3;

V_CORNER = linspace(5,35,81);

CURVATURE_CUTOFF = 0.008;

TIRE_UTILIZATION = 0.95;

ACCEL_ROLLOUT_M = 2.19;
ACCEL_TIMED_M   = 75;
ACCEL_DS        = 0.05;

SKIDPAD_WIN_TIME   = 4.878; %2026
ACCEL_WIN_TIME     = 4.126; %2026
AUTOX_WIN_TIME     = 46.911; %2024
ENDURANCE_WIN_TIME = 1395.891; %2024

%% ========================================================================
% 5. TIRE MODEL
%% ========================================================================

fprintf('\nLoading tire model...\n');

load('Fx_combined_parameters_run38_30.mat');
Fx_parameters = cell2mat(Xbestcell);

load('Lapsim_Fy_combined_parameters_1965run15.mat');
Fy_parameters = cell2mat(Xbestcell);

tire_pressure = 12;
friction_scaling_factor = 1.05*0.55;

tire = Tire2( ...
    tire_pressure, ...
    Fx_parameters, ...
    Fy_parameters, ...
    friction_scaling_factor);

%% ========================================================================
% 6. NEW v3: REAL POWERTRAIN (matches carConfig.m exactly)
% ========================================================================

fprintf('Building powertrain model...\n');

eParams = struct();
eParams.redline = 11500;
eParams.shift_point = 10000;
eParams.gears = [32/16 30/18 28/20 26/22 24/24];
eParams.primary_reduction = 76/32;
eParams.shift_time = 0.050;

DTparams = struct();
DTparams.final_drive = 33/11;
DTparams.drivetrain_efficiency = 0.87;
DTparams.G_d1 = 0;
DTparams.G_d2_overrun = 0;
TBR = 1;
DTparams.G_d2_driving = (TBR-1)/(2+2*TBR);

Bparams = struct();
Bparams.brake_distribution = 0.75;
Bparams.max_braking_torque = 840;

torque_fn = KTM450();   % NOTE: this function also pops open a plot window

powertrain = Powertrain( ...
    eParams.redline,eParams.shift_point,eParams.gears,eParams.primary_reduction, ...
    torque_fn,eParams.shift_time,DTparams.final_drive,wheel_radius, ...
    DTparams.drivetrain_efficiency,DTparams.G_d1,DTparams.G_d2_overrun, ...
    DTparams.G_d2_driving,Bparams.brake_distribution,Bparams.max_braking_torque);

fprintf('  Max speed (top gear, redline) = %.1f km/h\n', ...
    (eParams.redline/eParams.gears(end)/DTparams.final_drive/eParams.primary_reduction*pi/30*wheel_radius)*3.6);

%% ========================================================================
% 7. LOAD / RESAMPLE TRACKS
%% ========================================================================

fprintf('Loading tracks...\n');

load('michigantrack.mat');   % 2024 autocross track 

auto_s_raw = arclength(:)';
auto_k_raw = curvature(:)';

load('2024endurancetrack.mat');

end_s_raw = arclength(:)';
end_k_raw = curvature(:)';

auto_s = linspace(auto_s_raw(1),auto_s_raw(end),N_AUTO);
auto_k = interp1(auto_s_raw,auto_k_raw,auto_s,'linear');

end_s = linspace(end_s_raw(1),end_s_raw(end),N_END);
end_k = interp1(end_s_raw,end_k_raw,end_s,'linear');

%% ========================================================================
% 8. BUILD TIRE LOOKUP TABLES
% ========================================================================

fprintf('Building tire lookup tables...\n');

Fz_grid = linspace(50,900,60);

alpha_grid = linspace(0,15,151);
kappa_grid = linspace(0,0.25,126);

Fy_max_table = zeros(size(Fz_grid));
Fx_max_table = zeros(size(Fz_grid));

for i = 1:numel(Fz_grid)

    Fz_i = Fz_grid(i);

    Fy_vals = zeros(size(alpha_grid));
    Fx_vals = zeros(size(kappa_grid));

    for j = 1:numel(alpha_grid)
        Fy_vals(j) = tire.F_y(alpha_grid(j),0,Fz_i,0);
    end

    for j = 1:numel(kappa_grid)
        Fx_vals(j) = tire.F_x(0,kappa_grid(j),Fz_i,0);
    end

    Fy_max_table(i) = max(abs(Fy_vals));
    Fx_max_table(i) = max(abs(Fx_vals));

end

fprintf('Tire lookup complete.\n');

%% ========================================================================
% 9. BASELINE (auto / skid / endurance / accel) + CALIBRATION
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    BASELINE MODEL (RAW PHYSICS)\n');
fprintf('===============================================================\n');

t_start = tic;

baseline = evaluate_package( ...
    BASE_CLA,BASE_CDA,BASE_COP_FRONT,M, ...
    rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    auto_s,auto_k,end_s,end_k, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

baseline_accel = evaluate_accel_package( ...
    BASE_CLA_ACCEL,BASE_CDA_ACCEL,BASE_COP_FRONT_ACCEL,M_ACCEL, ...
    rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
    ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

fprintf('Autocross       = %.3f s (raw)\n',baseline.auto_time);
fprintf('Skidpad         = %.3f s (raw)\n',baseline.skidpad_time);
fprintf('Endurance/lap   = %.3f s (raw)\n',baseline.endurance_lap_time);
fprintf('Accel           = %.3f s (raw)\n',baseline_accel.accel_time);
fprintf('Runtime         = %.2f s\n',toc(t_start));

% Calibration ratios = actual / raw-model. Disabled for now (all = 1.0),
% per ENABLE_CALIBRATION above -- everything downstream is raw physics.
if ENABLE_CALIBRATION
    CAL_AUTOX      = ACTUAL_AUTOX_TIME    / baseline.auto_time;
    CAL_ENDURANCE  = ACTUAL_ENDURANCE_LAP / baseline.endurance_lap_time;
    CAL_SKIDPAD    = ACTUAL_SKIDPAD_TIME  / baseline.skidpad_time;
    CAL_ACCEL      = ACTUAL_ACCEL_TIME    / baseline_accel.accel_time;
else
    CAL_AUTOX     = 1;
    CAL_ENDURANCE = 1;
    CAL_SKIDPAD   = 1;
    CAL_ACCEL     = 1;
end

fprintf('\nCalibration: %s\n', mat2str(ENABLE_CALIBRATION));
if ENABLE_CALIBRATION
    fprintf('  Autocross  x%.3f\n',CAL_AUTOX);
    fprintf('  Endurance  x%.3f\n',CAL_ENDURANCE);
    fprintf('  Skidpad    x%.3f\n',CAL_SKIDPAD);
    fprintf('  Accel      x%.3f\n',CAL_ACCEL);
else
    fprintf('  (disabled -- all numbers below are raw physics, not comp-calibrated)\n');
end

baseline_cal.auto_time      = baseline.auto_time*CAL_AUTOX;
baseline_cal.skidpad_time   = baseline.skidpad_time*CAL_SKIDPAD;
baseline_cal.endurance_lap  = baseline.endurance_lap_time*CAL_ENDURANCE;
baseline_cal.endurance_10lap= baseline.endurance_10lap_time*CAL_ENDURANCE;
baseline_cal.accel_time     = baseline_accel.accel_time*CAL_ACCEL;

baseline_points.autox   = fsae_points_linear(baseline_cal.auto_time,AUTOX_WIN_TIME,1.45,118.5,6.5);
baseline_points.skidpad = fsae_points_skidpad(baseline_cal.skidpad_time,SKIDPAD_WIN_TIME);
baseline_points.enduro  = fsae_points_linear(baseline_cal.endurance_10lap,ENDURANCE_WIN_TIME,1.45,250,25);
baseline_points.accel   = fsae_points_linear(baseline_cal.accel_time,ACCEL_WIN_TIME,1.5,95.5,4.5);
baseline_points.total   = baseline_points.autox+baseline_points.skidpad+ ...
                           baseline_points.enduro+baseline_points.accel;

fprintf('\nBaseline times (raw physics; calibration disabled):\n');
fprintf('  Autocross      = %.3f s (points %.1f)\n',baseline_cal.auto_time,baseline_points.autox);
fprintf('  Skidpad        = %.3f s (points %.1f)\n',baseline_cal.skidpad_time,baseline_points.skidpad);
fprintf('  Endurance/lap  = %.3f s (points %.1f)\n',baseline_cal.endurance_lap,baseline_points.enduro);
fprintf('  Accel          = %.3f s (points %.1f)\n',baseline_cal.accel_time,baseline_points.accel);
fprintf('  TOTAL POINTS   = %.1f\n',baseline_points.total);

%% ========================================================================
% 10. CLA x CDA SWEEP -- BASELINE PACKAGE
% ========================================================================

CLA_values = 2.70:0.10:3.80;
CdA_values = 1.15:0.05:1.55;

nCLA = numel(CLA_values);
nCdA = numel(CdA_values);

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              BASELINE CLA x CdA SWEEP\n');
fprintf('===============================================================\n');
fprintf('Total cases = %d\n',nCLA*nCdA);

sweep = struct();
sweep.CLA = CLA_values;
sweep.CdA = CdA_values;

sweep.auto_time = NaN(nCLA,nCdA);
sweep.skidpad_time = NaN(nCLA,nCdA);
sweep.endurance_time = NaN(nCLA,nCdA);
sweep.endurance_10lap_time = NaN(nCLA,nCdA);

% calibrated + points (primary metrics for target selection)
sweep.points_autox = NaN(nCLA,nCdA);
sweep.points_skid  = NaN(nCLA,nCdA);
sweep.points_end   = NaN(nCLA,nCdA);
sweep.points_baseline_total = NaN(nCLA,nCdA);

counter = 0;
tic;

for i = 1:nCLA
    for j = 1:nCdA

        counter = counter + 1;

        r = evaluate_package( ...
            CLA_values(i),CdA_values(j),BASE_COP_FRONT,M, ...
            rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
            powertrain,wheel_radius, ...
            Fz_grid,Fy_max_table,Fx_max_table, ...
            auto_s,auto_k,end_s,end_k, ...
            N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

        sweep.auto_time(i,j) = r.auto_time;
        sweep.skidpad_time(i,j) = r.skidpad_time;
        sweep.endurance_time(i,j) = r.endurance_lap_time;
        sweep.endurance_10lap_time(i,j) = r.endurance_10lap_time;

        auto_cal = r.auto_time*CAL_AUTOX;
        skid_cal = r.skidpad_time*CAL_SKIDPAD;
        end10_cal = r.endurance_10lap_time*CAL_ENDURANCE;

        sweep.points_autox(i,j) = fsae_points_linear(auto_cal,AUTOX_WIN_TIME,1.45,118.5,6.5);
        sweep.points_skid(i,j)  = fsae_points_skidpad(skid_cal,SKIDPAD_WIN_TIME);
        sweep.points_end(i,j)   = fsae_points_linear(end10_cal,ENDURANCE_WIN_TIME,1.45,250,25);

        sweep.points_baseline_total(i,j) = sweep.points_autox(i,j) + ...
            sweep.points_skid(i,j) + sweep.points_end(i,j);

    end

    fprintf('CLA %.2f complete: %.1f %% | %.1f s\n', ...
        CLA_values(i),100*counter/(nCLA*nCdA),toc);

end

%% ========================================================================
% 11. BEST PACKAGE IN BASELINE SWEEP (by points (raw physics))
% ========================================================================

[~,best_idx] = max(sweep.points_baseline_total(:));
[best_i,best_j] = ind2sub(size(sweep.points_baseline_total),best_idx);

best_CLA = CLA_values(best_i);
best_CdA = CdA_values(best_j);

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    SWEEP RESULT (BASELINE)\n');
fprintf('===============================================================\n');
fprintf('Best CLA = %.2f m^2, Best CdA = %.2f m^2\n',best_CLA,best_CdA);
fprintf('Best 3-event points (raw physics) = %.1f\n',sweep.points_baseline_total(best_i,best_j));

%% ========================================================================
% 12. BREAK-EVEN TRADEOFF (endurance-time criterion)
% ========================================================================

endurance_time_cal = sweep.endurance_time*CAL_ENDURANCE;
baseline_endurance_cal = baseline.endurance_lap_time*CAL_ENDURANCE;

break_even_CdA = NaN(size(CLA_values));

for i = 1:nCLA

    delta_time = endurance_time_cal(i,:) - baseline_endurance_cal;

    for j = 1:nCdA-1

        y1 = delta_time(j);
        y2 = delta_time(j+1);

        if y1 == 0
            break_even_CdA(i) = CdA_values(j);
            break;
        elseif y1*y2 < 0
            break_even_CdA(i) = CdA_values(j) + ...
                (-y1)*(CdA_values(j+1)-CdA_values(j))/(y2-y1);
            break;
        end

    end
end

%% ========================================================================
% 13. PRACTICAL TARGET -- BASELINE PACKAGE
% ========================================================================

candidate_mask = false(nCLA,nCdA);

for i = 1:nCLA
    for j = 1:nCdA
        if CLA_values(i) <= 3.50 && CdA_values(j) <= 1.45 && ...
           endurance_time_cal(i,j) <= baseline_endurance_cal
            candidate_mask(i,j) = true;
        end
    end
end

if any(candidate_mask(:))

    candidate_idx = find(candidate_mask);
    best_target_idx = candidate_idx(1);
    best_target_score = -Inf;

    for q = 1:numel(candidate_idx)
        idx = candidate_idx(q);
        [ii,jj] = ind2sub(size(candidate_mask),idx);
        score = 100*CLA_values(ii) - CdA_values(jj);
        if score > best_target_score
            best_target_score = score;
            best_target_idx = idx;
        end
    end

    [target_i,target_j] = ind2sub(size(candidate_mask),best_target_idx);
    target_CLA = CLA_values(target_i);
    target_CdA = CdA_values(target_j);

else
    target_i = find(CLA_values==BASE_CLA,1);
    target_j = find(CdA_values==BASE_CDA,1);
    target_CLA = BASE_CLA;
    target_CdA = BASE_CDA;
end

target = evaluate_package( ...
    target_CLA,target_CdA,BASE_COP_FRONT,M, ...
    rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    auto_s,auto_k,end_s,end_k, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

%% ========================================================================
% 14. CLA_accel x CdA_accel SWEEP -- ACCEL PACKAGE (now powertrain-aware)
% ========================================================================

CLA_accel_values = 0:0.25:3.10;
CdA_accel_values = 0.85:0.05:1.35;

nCLAa = numel(CLA_accel_values);
nCdAa = numel(CdA_accel_values);

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              ACCEL CLA x CdA SWEEP\n');
fprintf('===============================================================\n');

accel_sweep = struct();
accel_sweep.CLA_accel = CLA_accel_values;
accel_sweep.CdA_accel = CdA_accel_values;
accel_sweep.accel_time = NaN(nCLAa,nCdAa);
accel_sweep.points_accel = NaN(nCLAa,nCdAa);

tic;

for i = 1:nCLAa
    for j = 1:nCdAa

        ra = evaluate_accel_package( ...
            CLA_accel_values(i),CdA_accel_values(j),BASE_COP_FRONT_ACCEL,M_ACCEL, ...
            rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
            powertrain,wheel_radius, ...
            Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
            ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

        accel_time_cal = ra.accel_time*CAL_ACCEL;

        accel_sweep.accel_time(i,j) = ra.accel_time;
        accel_sweep.points_accel(i,j) = fsae_points_linear( ...
            accel_time_cal,ACCEL_WIN_TIME,1.5,95.5,4.5);

    end
end

fprintf('Accel sweep complete: %.1f s\n',toc);

[~,best_accel_idx] = max(accel_sweep.points_accel(:));
[best_accel_i,best_accel_j] = ind2sub(size(accel_sweep.points_accel),best_accel_idx);

target_CLA_accel = CLA_accel_values(best_accel_i);
target_CdA_accel = CdA_accel_values(best_accel_j);

target_accel = evaluate_accel_package( ...
    target_CLA_accel,target_CdA_accel,BASE_COP_FRONT_ACCEL,M_ACCEL, ...
    rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
    ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

fprintf('\nFastest accel package found in sweep:\n');
fprintf('  CLA_accel = %.2f m^2, CdA_accel = %.2f m^2\n',target_CLA_accel,target_CdA_accel);
fprintf('  Accel time (raw physics) = %.3f s\n',target_accel.accel_time*CAL_ACCEL);

%% ========================================================================
% 15. FINAL TARGET SUMMARY
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    PRACTICAL TARGETS\n');
fprintf('===============================================================\n');

fprintf('\n--- BASELINE PACKAGE (autocross / skidpad / endurance) ---\n');
fprintf('Current: CLA=%.2f, CdA=%.2f, CoP=%.1f%% front\n', ...
    BASE_CLA,BASE_CDA,100*BASE_COP_FRONT);
fprintf('Target:  CLA=%.2f, CdA=%.2f (Delta CLA=%+0.2f, Delta CdA=%+0.2f)\n', ...
    target_CLA,target_CdA,target_CLA-BASE_CLA,target_CdA-BASE_CDA);
fprintf('  Autocross (raw)      = %.3f s\n',target.auto_time*CAL_AUTOX);
fprintf('  Skidpad (raw)        = %.3f s\n',target.skidpad_time*CAL_SKIDPAD);
fprintf('  Endurance/lap (raw)  = %.3f s\n',target.endurance_lap_time*CAL_ENDURANCE);

fprintf('\n--- ACCEL PACKAGE (only meaningful if aero is adjustable) ---\n');
fprintf('Starting: CLA_accel=%.2f, CdA_accel=%.2f\n',BASE_CLA_ACCEL,BASE_CDA_ACCEL);
fprintf('Target:   CLA_accel=%.2f, CdA_accel=%.2f\n',target_CLA_accel,target_CdA_accel);
fprintf('  Accel time (raw) = %.3f s (vs %.3f s at starting point)\n', ...
    target_accel.accel_time*CAL_ACCEL,baseline_accel.accel_time*CAL_ACCEL);

total_points_target = sweep.points_baseline_total(best_i,best_j) + ...
    accel_sweep.points_accel(best_accel_i,best_accel_j);

fprintf('\nCombined 4-event points (raw physics) at BEST baseline + BEST accel:\n');
fprintf('  TOTAL = %.1f  (vs %.1f at current baseline package)\n', ...
    total_points_target,baseline_points.total);

%% ========================================================================
% 16. CoP SWEEPS (baseline + accel)
% ========================================================================

CoP_values = 0.30:0.02:0.55;

cop_results.auto_time = zeros(size(CoP_values));
cop_results.skidpad_time = zeros(size(CoP_values));
cop_results.endurance_time = zeros(size(CoP_values));

for i = 1:numel(CoP_values)

    r = evaluate_package( ...
        BASE_CLA,BASE_CDA,CoP_values(i),M, ...
        rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fy_max_table,Fx_max_table, ...
        auto_s,auto_k,end_s,end_k, ...
        N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

    cop_results.auto_time(i) = r.auto_time;
    cop_results.skidpad_time(i) = r.skidpad_time;
    cop_results.endurance_time(i) = r.endurance_lap_time;

end

CoP_accel_values = 0.00:0.05:0.50;
accel_cop_results.accel_time = zeros(size(CoP_accel_values));

for i = 1:numel(CoP_accel_values)

    ra = evaluate_accel_package( ...
        target_CLA_accel,target_CdA_accel,CoP_accel_values(i),M_ACCEL, ...
        rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
        ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

    accel_cop_results.accel_time(i) = ra.accel_time;

end

%% ========================================================================
% 17. PLOTS
% ========================================================================

[CdA_mesh,CLA_mesh] = meshgrid(CdA_values,CLA_values);

figure('Name','CLA CdA Points Surface - Baseline','Position',[100 100 1100 750]);
surf(CdA_mesh,CLA_mesh,sweep.points_baseline_total,'EdgeColor','k','FaceAlpha',0.85);
hold on;

baseline_z = fsae_points_linear(baseline.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5) + ...
    fsae_points_skidpad(baseline.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME) + ...
    fsae_points_linear(baseline.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);

plot3(BASE_CDA,BASE_CLA,baseline_z,'ko','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');
plot3(target_CdA,target_CLA,sweep.points_baseline_total(target_i,target_j), ...
    'ks','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');
plot3(best_CdA,best_CLA,sweep.points_baseline_total(best_i,best_j), ...
    'k^','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');

xlabel('CdA (m^2)'); ylabel('CLA (m^2)'); zlabel('3-event points (raw physics)');
title('Baseline CLA / CdA vs. Competition Points (raw physics)');
legend('Points surface','Current baseline','Practical target','Best in sweep','Location','best');
grid on; view(-40,28); colorbar;

[CdAa_mesh,CLAa_mesh] = meshgrid(CdA_accel_values,CLA_accel_values);

figure('Name','Accel CLA CdA Time Surface','Position',[130 130 1100 750]);
surf(CdAa_mesh,CLAa_mesh,accel_sweep.accel_time*CAL_ACCEL,'EdgeColor','k','FaceAlpha',0.85);
hold on;
plot3(BASE_CDA_ACCEL,BASE_CLA_ACCEL,baseline_accel.accel_time*CAL_ACCEL, ...
    'ko','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');
plot3(target_CdA_accel,target_CLA_accel,target_accel.accel_time*CAL_ACCEL, ...
    'ks','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');
xlabel('CdA_{accel} (m^2)'); ylabel('CLA_{accel} (m^2)'); zlabel('Accel time (s)');
title('Accel CLA / CdA vs. Accel Time');
legend('Accel time surface','Current (=baseline)','Fastest in sweep','Location','best');
grid on; view(-40,28); colorbar;

figure('Name','CoP Sensitivity - Baseline','Position',[150 150 1000 800]);

subplot(2,2,1);
plot(100*CoP_values,cop_results.auto_time*CAL_AUTOX,'o-','LineWidth',2); hold on;
xline(100*BASE_COP_FRONT,'--');
xlabel('Front CoP (%)'); ylabel('Autocross time (s)'); title('Autocross'); grid on;

subplot(2,2,2);
plot(100*CoP_values,cop_results.skidpad_time*CAL_SKIDPAD,'o-','LineWidth',2); hold on;
xline(100*BASE_COP_FRONT,'--');
xlabel('Front CoP (%)'); ylabel('Skidpad time (s)'); title('Skidpad'); grid on;

subplot(2,2,3);
plot(100*CoP_values,cop_results.endurance_time*CAL_ENDURANCE,'o-','LineWidth',2); hold on;
xline(100*BASE_COP_FRONT,'--');
xlabel('Front CoP (%)'); ylabel('Endurance lap time (s)'); title('Endurance'); grid on;

subplot(2,2,4);
plot(100*CoP_values, ...
    0.30*(baseline.auto_time./cop_results.auto_time) + ...
    0.70*(baseline.endurance_lap_time./cop_results.endurance_time), ...
    'o-','LineWidth',2); hold on;
xline(100*BASE_COP_FRONT,'--');
xlabel('Front CoP (%)'); ylabel('Relative performance'); title('Combined (auto+endurance)'); grid on;

figure('Name','CoP Sensitivity - Accel','Position',[180 180 700 500]);
plot(100*CoP_accel_values,accel_cop_results.accel_time*CAL_ACCEL,'o-','LineWidth',2); hold on;
xline(100*BASE_COP_FRONT_ACCEL,'--','Baseline CoP');
xlabel('Front CoP (%)'); ylabel('Accel time (s)');
title({'Accel Time vs. Front/Rear Downforce Split','(0% front = all downforce on driven rear axle)'});
grid on;

figure('Name','CLA CdA Tradeoff Boundary','Position',[200 200 1000 700]);
valid_break = isfinite(break_even_CdA);
plot(CLA_values(valid_break),break_even_CdA(valid_break),'o-','LineWidth',2.5,'MarkerSize',7);
hold on;
yline(BASE_CDA,'--','Current CdA'); xline(BASE_CLA,'--','Current CLA');
plot(target_CLA,target_CdA,'ks','MarkerSize',11,'LineWidth',2,'MarkerFaceColor','w');
xlabel('CLA (m^2)'); ylabel('Maximum CdA for no endurance-time penalty (m^2)');
title('Aero Tradeoff: Additional Downforce vs Allowable Drag (Baseline)');
grid on;

figure('Name','Endurance Time Surface','Position',[250 250 1000 700]);
surf(CdA_mesh,CLA_mesh,sweep.endurance_time*CAL_ENDURANCE,'EdgeColor','none');
hold on;
plot3(BASE_CDA,BASE_CLA,baseline.endurance_lap_time*CAL_ENDURANCE, ...
    'ko','MarkerSize',10,'LineWidth',3,'MarkerFaceColor','w');
xlabel('CdA (m^2)'); ylabel('CLA (m^2)'); zlabel('Endurance lap time (s)');
title('Endurance Performance Surface (Baseline)');
grid on; view(-40,28); colorbar;

%% ========================================================================
% 18. SAVE
% ========================================================================

aero_spec_results = struct();
aero_spec_results.baseline = baseline;
aero_spec_results.baseline_accel = baseline_accel;
aero_spec_results.baseline_cal = baseline_cal;
aero_spec_results.baseline_points = baseline_points;
aero_spec_results.calibration = struct('CAL_AUTOX',CAL_AUTOX,'CAL_ENDURANCE',CAL_ENDURANCE, ...
    'CAL_SKIDPAD',CAL_SKIDPAD,'CAL_ACCEL',CAL_ACCEL);
aero_spec_results.target = target;
aero_spec_results.target_accel = target_accel;
aero_spec_results.sweep = sweep;
aero_spec_results.accel_sweep = accel_sweep;
aero_spec_results.targets.baseline_CLA = target_CLA;
aero_spec_results.targets.baseline_CdA = target_CdA;
aero_spec_results.targets.accel_CLA = target_CLA_accel;
aero_spec_results.targets.accel_CdA = target_CdA_accel;

assignin('base','aero_spec_results',aero_spec_results);
save('aero_spec_results_v3.mat','aero_spec_results');

fprintf('\n===============================================================\n');
fprintf('                         COMPLETE\n');
fprintf('===============================================================\n');

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function out = evaluate_package( ...
    CLA,CdA,CoP_front,M, ...
    rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    auto_s,auto_k,end_s,end_k, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION)

    CoP_rear = 1-CoP_front;

    auto = simulate_track( ...
        auto_s,auto_k,CLA,CdA,CoP_front,CoP_rear, ...
        rho,g,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf,powertrain,wheel_radius, ...
        Fz_grid,Fy_max_table,Fx_max_table, ...
        N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

    enduro = simulate_track( ...
        end_s,end_k,CLA,CdA,CoP_front,CoP_rear, ...
        rho,g,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf,powertrain,wheel_radius, ...
        Fz_grid,Fy_max_table,Fx_max_table, ...
        N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

    skid = solve_skidpad( ...
        CLA,CdA,CoP_front,CoP_rear,rho,g,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        Fz_grid,Fy_max_table,Fx_max_table,TIRE_UTILIZATION);

    out.auto_time = auto.total_time*1.08;
    out.endurance_lap_time = enduro.total_time*1.085;
    out.endurance_10lap_time = enduro.total_time*10*1.085;
    out.skidpad_time = skid.time;
    out.max_speed = max(enduro.speed_profile);
    out.mean_endurance_speed = mean(enduro.speed_profile);

end


function out = evaluate_accel_package( ...
    CLA_accel,CdA_accel,CoP_front_accel,M_ACCEL, ...
    rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
    rollout_m,timed_m,ds)

    % NEW v3: powertrain-aware straight-line sim (was Pmax/V in v2)
    CoP_rear_accel = 1 - CoP_front_accel;

    [V_after_rollout,~,~] = integrate_straight_powertrain( ...
        0,rollout_m,ds,CLA_accel,CdA_accel,CoP_front_accel,CoP_rear_accel,M_ACCEL, ...
        rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf,powertrain,wheel_radius, ...
        Fz_grid,Fx_max_table,TIRE_UTILIZATION);

    [V_final,t_timed,num_shifts] = integrate_straight_powertrain( ...
        V_after_rollout,timed_m,ds,CLA_accel,CdA_accel,CoP_front_accel,CoP_rear_accel,M_ACCEL, ...
        rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf,powertrain,wheel_radius, ...
        Fz_grid,Fx_max_table,TIRE_UTILIZATION);

    out.accel_time = t_timed;
    out.final_speed = V_final;
    out.num_shifts = num_shifts;

end


function idx = find_gear(V,switch_gear_velocities)
    idx = find(V < switch_gear_velocities,1);
    if isempty(idx)
        idx = numel(switch_gear_velocities);
    end
end


% NEW v3: replaces the Pmax/V accel integrator with a real gear/torque
% curve model, reusing your Powertrain class and KTM450 torque data.
function [Vend,t,num_shifts] = integrate_straight_powertrain( ...
    V0,distance,ds, ...
    CLA,CdA,CoP_front,CoP_rear,M, ...
    rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fx_max_table,TIRE_UTILIZATION)

    V = max(V0,0.05);
    d = 0;
    t = 0;

    current_gear = find_gear(V,powertrain.switch_gear_velocities);
    shifting = false;
    shift_timer = 0;
    num_shifts = 0;

    rpm_table = powertrain.torque_fn(1,:);
    tq_table  = powertrain.torque_fn(2,:);

    while d < distance

        step = min(ds,distance-d);

        if ~shifting && current_gear < numel(powertrain.gears) && ...
                V > powertrain.switch_gear_velocities(current_gear)
            shifting = true;
            shift_timer = 0;
        end

        Fdrag = 0.5*rho*V^2*CdA;

        if shifting

            ax = -Fdrag/M;

            shift_timer = shift_timer + step/max(V,0.5);

            if shift_timer >= powertrain.shift_time
                shifting = false;
                shift_timer = 0;
                current_gear = current_gear + 1;
                num_shifts = num_shifts + 1;
            end

        else

            omega = V/wheel_radius;
            gear_ratio = powertrain.drivetrain_reduction(current_gear);
            engine_rpm = omega*gear_ratio*30/pi;
            engine_rpm_c = min(max(engine_rpm,min(rpm_table)),max(rpm_table));

            torque_engine_ftlb = interp1(rpm_table,tq_table,engine_rpm_c,'linear');
            torque_engine_nm = torque_engine_ftlb*1.35581795*powertrain.drivetrain_efficiency;

            F_engine = torque_engine_nm*gear_ratio/wheel_radius;

            Fz = wheel_loads(V,0,0,CLA,CoP_front,CoP_rear, ...
                rho,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf);

            Fx_rear_capacity = tire_longitudinal_capacity(Fz(3:4),Fz_grid,Fx_max_table);
            F_traction = TIRE_UTILIZATION*sum(Fx_rear_capacity);

            F_drive = min(F_engine,F_traction);

            ax = (F_drive-Fdrag)/M;

        end

        ax = max(ax,-9.81);

        V2 = sqrt(max(V^2+2*ax*step,0));
        t = t + 2*step/(V+V2);
        V = V2;
        d = d + step;

    end

    Vend = V;

end


function result = simulate_track( ...
    s,k,CLA,CdA,CoP_front,CoP_rear, ...
    rho,g,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf,powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION)

    s = s(:)'; k = k(:)';
    valid = isfinite(s) & isfinite(k);
    s = s(valid); k = k(valid);
    ds = diff(s);

    if numel(ds) < 2
        error('Track has insufficient points.');
    end

    V = 10*ones(size(s));

    for iter = 1:N_ITER

        V_corner = 100*ones(size(V));
        corner_idx = find(abs(k)>=CURVATURE_CUTOFF);

        for n = 1:numel(corner_idx)
            i = corner_idx(n);
            R = 1/abs(k(i));
            V_corner(i) = solve_corner_speed( ...
                R,CLA,CdA,CoP_front,CoP_rear,rho,g,M,Wb,track,lf,lr,hg, ...
                hrf,hrr,hrc,Rsf,Fz_grid,Fy_max_table,Fx_max_table, ...
                V_CORNER,TIRE_UTILIZATION);
        end

        Vfwd = zeros(size(V));
        Vfwd(1) = max(V(1),3);

        for i = 1:length(V)-1

            V0 = Vfwd(i);
            ay = V0^2*abs(k(i));

            ax = available_acceleration( ...
                V0,ay,CLA,CdA,CoP_front,CoP_rear, ...
                rho,M,Wb,track,lf,lr,hg,powertrain,wheel_radius, ...
                Fz_grid,Fy_max_table,Fx_max_table);

            V2 = V0^2 + 2*max(ax,-5)*ds(i);
            Vfwd(i+1) = sqrt(max(V2,0));
            Vfwd(i+1) = min(Vfwd(i+1),V_corner(i+1));

        end

        Vback = Vfwd;
        Vback(end) = min(Vback(end),V_corner(end));

        for i = length(V)-1:-1:1

            Vf = Vback(i+1);

            a_brake = braking_acceleration( ...
                Vf,CLA,CdA,CoP_front,CoP_rear, ...
                rho,M,Wb,track,lf,lr,hg, ...
                Fz_grid,Fy_max_table,Fx_max_table);

            Vpossible = sqrt(max(Vf^2 + 2*abs(a_brake)*ds(i),0));
            Vback(i) = min(Vfwd(i),Vpossible);
            Vback(i) = min(Vback(i),V_corner(i));

        end

        Vnew = Vback;
        err = max(abs(Vnew-V));
        V = 0.5*V + 0.5*Vnew;

        if err < 0.02
            break;
        end

    end

    Vsafe = max(V,0.5);
    dt = ds ./ max(0.5*(Vsafe(1:end-1)+Vsafe(2:end)),0.5);

    result.total_time = sum(dt);
    result.speed_profile = V;

end


% NEW v3: corner speed search now applies friction-ellipse coupling --
% rear tires must spend some capacity fighting drag even at "steady"
% cornering speed, which reduces the lateral grip actually available.
function Vmax = solve_corner_speed( ...
    R,CLA,CdA,CoP_front,CoP_rear,rho,g,M,Wb,track,lf,lr,hg, ...
    hrf,hrr,hrc,Rsf,Fz_grid,Fy_max_table,Fx_max_table, ...
    V_candidates,TIRE_UTILIZATION)

    valid = false(size(V_candidates));

    for i = 1:numel(V_candidates)

        V = V_candidates(i);
        ay = V^2/R;

        if ay > 2.5*9.81
            continue;
        end

        Fz = wheel_loads(V,ay,0,CLA,CoP_front,CoP_rear, ...
            rho,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf);

        Fdrag = 0.5*rho*V^2*CdA;

        % RWD: only rear tires supply the thrust needed to hold steady speed
        Fx_demand = [0,0,Fdrag/2,Fdrag/2];

        Fy = tire_lateral_capacity_combined(Fz,Fx_demand,Fz_grid,Fy_max_table,Fx_max_table);

        Fy_total = TIRE_UTILIZATION*sum(Fy);

        if Fy_total >= M*ay
            valid(i) = true;
        end

    end

    if any(valid)
        Vmax = max(V_candidates(valid));
    else
        Vmax = 5;
    end

end


function ax = available_acceleration( ...
    V,ay,CLA,CdA,CoP_front,CoP_rear, ...
    rho,M,Wb,track,lf,lr,hg,powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table)

    q = 0.5*rho*V^2;
    Fdrag = q*CdA;

    Fz = wheel_loads(V,ay,0,CLA,CoP_front,CoP_rear, ...
        rho,M,Wb,track,lf,lr,hg,0.0876,0.0914,0.0895,0.34);

    % NEW v3: lateral demand per wheel, proportional to normal load share
    Fy_demand_total = M*ay;
    Fy_demand = Fy_demand_total * (Fz/sum(Fz));

    Fx_rear = tire_longitudinal_capacity_combined( ...
        Fz(3:4),Fy_demand(3:4),Fz_grid,Fx_max_table,Fy_max_table);

    Fx_tire = sum(Fx_rear);

    % engine-limited force, using current gear at this velocity
    current_gear = find_gear(V,powertrain.switch_gear_velocities);
    rpm_table = powertrain.torque_fn(1,:);
    tq_table  = powertrain.torque_fn(2,:);
    omega = V/wheel_radius;
    gear_ratio = powertrain.drivetrain_reduction(current_gear);
    engine_rpm = omega*gear_ratio*30/pi;
    engine_rpm_c = min(max(engine_rpm,min(rpm_table)),max(rpm_table));
    torque_engine_ftlb = interp1(rpm_table,tq_table,engine_rpm_c,'linear');
    torque_engine_nm = torque_engine_ftlb*1.35581795*powertrain.drivetrain_efficiency;
    Fx_power = torque_engine_nm*gear_ratio/wheel_radius;

    Fx_total = min(Fx_tire,Fx_power);

    ax = (Fx_total-Fdrag)/M;

end


function ax = braking_acceleration( ...
    V,CLA,CdA,CoP_front,CoP_rear, ...
    rho,M,Wb,track,lf,lr,hg, ...
    Fz_grid,Fy_max_table,Fx_max_table)

    ax_guess = -1.2*9.81;

    Fz = wheel_loads(V,0,ax_guess,CLA,CoP_front,CoP_rear, ...
        rho,M,Wb,track,lf,lr,hg,0.0876,0.0914,0.0895,0.34);

    % NEW v3: braking uses all 4 wheels; assume near-zero lateral demand
    % here since this feeds the pure-braking backward pass (ay~0 at the
    % instant of max braking effort in this simplified model)
    Fy_demand = zeros(size(Fz));

    Fx = tire_longitudinal_capacity_combined(Fz,Fy_demand,Fz_grid,Fx_max_table,Fy_max_table);

    ax = -sum(Fx)/M;
    ax = max(ax,-2*9.81);

end


function result = solve_skidpad( ...
    CLA,CdA,CoP_front,CoP_rear,rho,g,M,Wb,track,lf,lr,hg, ...
    hrf,hrr,hrc,Rsf,Fz_grid,Fy_max_table,Fx_max_table,TIRE_UTILIZATION)

    radius = 8.5;
    V_candidates = linspace(5,15,321);
    valid = false(size(V_candidates));

    for i = 1:numel(V_candidates)

        V = V_candidates(i);
        ay = V^2/radius;

        if ay > 2.5*9.81
            continue;
        end

        Fz = wheel_loads(V,ay,0,CLA,CoP_front,CoP_rear, ...
            rho,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf);

        Fdrag = 0.5*rho*V^2*CdA;
        Fx_demand = [0,0,Fdrag/2,Fdrag/2];   % NEW v3: RWD thrust demand

        Fy = tire_lateral_capacity_combined(Fz,Fx_demand,Fz_grid,Fy_max_table,Fx_max_table);

        if TIRE_UTILIZATION*sum(Fy) >= M*ay
            valid(i) = true;
        end

    end

    if any(valid)
        Vmax = max(V_candidates(valid));
    else
        Vmax = 5;
    end

    result.speed = Vmax;
    result.time = 2*pi*radius/Vmax;

end


function Fz = wheel_loads( ...
    V,ay,ax,CLA,CoP_front,CoP_rear, ...
    rho,M,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf)

    q = 0.5*rho*V^2;
    Fdown = q*CLA;
    Fdown_front = Fdown*CoP_front;
    Fdown_rear = Fdown*CoP_rear;

    Fz_front = M*9.81*lr/Wb + Fdown_front;
    Fz_rear = M*9.81*lf/Wb + Fdown_rear;

    dF_long = M*ax*hg/Wb;

    dF_lat_front = M*ay/track * ((lr*hrf/Wb) + Rsf*(hg-hrc));
    dF_lat_rear  = M*ay/track * ((lr*hrr/Wb) + (1-Rsf)*(hg-hrc));

    Fzv = zeros(1,4);
    Fzv(1) = 0.5*Fz_front - 0.5*dF_long + dF_lat_front;
    Fzv(2) = 0.5*Fz_front - 0.5*dF_long - dF_lat_front;
    Fzv(3) = 0.5*Fz_rear + 0.5*dF_long + dF_lat_rear;
    Fzv(4) = 0.5*Fz_rear + 0.5*dF_long - dF_lat_rear;

    epsilon = 10;
    Fz = (Fzv + sqrt(Fzv.^2+epsilon))/2;

end


function Fy = tire_lateral_capacity(Fz,Fz_grid,Fy_table)
    Fy = zeros(size(Fz));
    for i = 1:numel(Fz)
        Fz_i = min(max(Fz(i),Fz_grid(1)),Fz_grid(end));
        Fy(i) = interp1(Fz_grid,Fy_table,Fz_i,'linear');
    end
end


function Fx = tire_longitudinal_capacity(Fz,Fz_grid,Fx_table)
    Fx = zeros(size(Fz));
    for i = 1:numel(Fz)
        Fz_i = min(max(Fz(i),Fz_grid(1)),Fz_grid(end));
        Fx(i) = interp1(Fz_grid,Fx_table,Fz_i,'linear');
    end
end


% NEW v3: friction-ellipse helpers. Approximation of combined slip:
% available force in one direction shrinks as the tire spends more of
% its budget in the orthogonal direction. sqrt(1-(used/max)^2) is the
% standard friction-circle approximation -- not full Magic Formula
% combined slip, but far better than treating Fx/Fy as independent.
function Fy = tire_lateral_capacity_combined(Fz,Fx_demand,Fz_grid,Fy_table,Fx_table)
    Fy = zeros(size(Fz));
    for i = 1:numel(Fz)
        Fz_i = min(max(Fz(i),Fz_grid(1)),Fz_grid(end));
        Fy_max_i = interp1(Fz_grid,Fy_table,Fz_i,'linear');
        Fx_max_i = interp1(Fz_grid,Fx_table,Fz_i,'linear');
        if Fx_max_i > 1e-6
            frac = min(abs(Fx_demand(i))/Fx_max_i,1);
        else
            frac = 1;
        end
        Fy(i) = Fy_max_i*sqrt(max(1-frac^2,0));
    end
end


function Fx = tire_longitudinal_capacity_combined(Fz,Fy_demand,Fz_grid,Fx_table,Fy_table)
    Fx = zeros(size(Fz));
    for i = 1:numel(Fz)
        Fz_i = min(max(Fz(i),Fz_grid(1)),Fz_grid(end));
        Fx_max_i = interp1(Fz_grid,Fx_table,Fz_i,'linear');
        Fy_max_i = interp1(Fz_grid,Fy_table,Fz_i,'linear');
        if Fy_max_i > 1e-6
            frac = min(abs(Fy_demand(i))/Fy_max_i,1);
        else
            frac = 1;
        end
        Fx(i) = Fx_max_i*sqrt(max(1-frac^2,0));
    end
end


function points = fsae_points_linear(t_your,t_winning,ratio_max,points_range,points_min)
    t_min = min(t_winning,t_your);
    t_max = ratio_max*t_min;
    if t_your > t_max
        points = points_min;
    else
        points = points_range*((t_max/t_your)-1)/((t_max/t_min)-1) + points_min;
    end
end


function points = fsae_points_skidpad(t_your,t_winning)
    t_min = min(t_winning,t_your);
    t_max = 1.25*t_min;
    if t_your > t_max
        points = 3.5;
    else
        points = 71.5*((t_max/t_your)^2-1)/((t_max/t_min)^2-1) + 3.5;
    end
end