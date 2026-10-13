%% ========================================================================
% FSAE AERO SPECIFICATION - FAST TIRE-AWARE AERO TARGET MODEL (v3.1)
%
% Built directly off the pasted v3 baseline -- NONE of the induced-drag-
% curve, apex-detection, or other later changes are included here. This
% is v3 plus exactly the fixes discussed and logged one at a time:
%
%   FIX 1: Low-drag accel package is now DERIVED from whichever baseline
%          target is chosen, via fixed ratios computed once from your
%          starting accel measurement:
%              CLA_ACCEL_RATIO = BASE_CLA_ACCEL / BASE_CLA
%              CDA_ACCEL_RATIO = BASE_CDA_ACCEL / BASE_CDA
%          The old independent CLA_accel x CdA_accel 2D grid is gone --
%          there's nothing left to range-set there (see FIX 6).
%          NOTE: BASE_COP_FRONT_ACCEL is now DERIVED (see FIX 9) using
%          the same physical reasoning -- no longer a flat unconfirmed
%          constant.
%
%   FIX 2: CdA floor at chassis-only drag (CDA0_BARE_CHASSIS = 0.95 m^2,
%          from coastdown testing). Nothing tested or recommended can go
%          below this -- it's not physically achievable with zero wings.
%
%   FIX 3: The break-even ("cap") boundary now requires BOTH autocross
%          AND endurance to not regress, not just endurance. At each
%          CLA, the actual cap is whichever event regresses FIRST as
%          CdA increases (the tighter of the two).
%
%   FIX 4: CdA_values sweep range -> 1.00:0.05:1.50 (was 1.15-1.55).
%
%   FIX 5: CoP_values (baseline) AND CoP_accel_values (accel) both ->
%          0.30:0.02:0.75 front (was 30-55% and 0-50% respectively).
%
%   FIX 6: Accel sweep ranges removed -- superseded by FIX 1.
%
%   FIX 7: Target quantification -- "best in sweep" (unconstrained
%          argmax) always just sits at whatever edge you tested, so it's
%          kept only as a labeled reference point, NOT the recommended
%          target. Method 1 (marginal return per unit CdA spent) is
%          used instead: maximize (points_gained) / (CdA_spent_above_
%          baseline). Finds a genuine interior peak because the RATIO
%          turns over even when raw points keeps climbing.
%
%% ========================================================================

clc;
clear;
close all;

%% ========================================================================
% 1. CALIBRATION DATA -- DISABLED FOR NOW
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

% FIX 8: aero reports CoP as position along the wheelbase, % measured
% FROM THE FRONT AXLE (e.g. "47% front" = CoP sits 47% of the way along
% the wheelbase from the front axle). Our physics needs a FORCE
% FRACTION -- the actual share of downforce reacted at the front axle
% -- which is the COMPLEMENT of aero's number: 1-0.47 = 0.53. This is
% the single source of truth for the physics; every display below
% converts back to aero's convention for human-facing output.
BASE_COP_FRONT = 0.53;       % df front ditribution = 1 - 0.47 (aero's reported wheelbase position)
BASE_COP_REAR  = 1 - BASE_COP_FRONT;

BASE_CLA_ACCEL = 2.10;      % m^2, accel package starting point (measured, flap flat)
BASE_CDA_ACCEL = 1.01;      % m^2, accel package starting point (measured, flap flat)

% FIX 9: BASE_COP_FRONT_ACCEL is now DERIVED, not hardcoded. Physical
% reasoning: only the rear flap changes between full-aero and low-drag
% configs, so the FRONT wing's absolute downforce contribution stays
% fixed. That fixed front force, divided by the smaller low-drag total
% CLA, gives the new front force fraction:
%     CLA_front_fixed   = BASE_CLA * BASE_COP_FRONT   (front's absolute downforce)
%     BASE_COP_FRONT_ACCEL = CLA_front_fixed / BASE_CLA_ACCEL
% This was previously hardcoded as 0.63, computed using the WRONG
% BASE_COP_FRONT (0.47, aero's un-converted wheelbase-position number).
% Now that BASE_COP_FRONT is corrected to 0.53, this recomputes to
% ~0.71 automatically -- and will stay correct if BASE_COP_FRONT,
% BASE_CLA, or BASE_CLA_ACCEL ever change again.
BASE_COP_FRONT_ACCEL = (BASE_CLA * BASE_COP_FRONT) / BASE_CLA_ACCEL;

BASE_CAR_MASS      = 162.8;   % kg, car only (no driver)
BASE_DRIVER_MASS   = 64;      % kg
ACCEL_DRIVER_MASS  = 59;      % kg

% NEW FIX 2: chassis-only drag floor, from coastdown testing
CDA0_BARE_CHASSIS = 0.95;   % m^2 -- CdA cannot physically go below this

% NEW FIX 1: fixed ratios, computed once, used later to derive the
% low-drag accel package from whatever baseline target is chosen
CLA_ACCEL_RATIO = BASE_CLA_ACCEL/BASE_CLA;
CDA_ACCEL_RATIO = BASE_CDA_ACCEL/BASE_CDA;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('           FSAE FAST AERO SPECIFICATION MODEL (v3.1)\n');
fprintf('===============================================================\n');

fprintf('\nBaseline package (auto/skid/endurance):\n');
fprintf('  CLA       = %.2f m^2\n',BASE_CLA);
fprintf('  CdA       = %.2f m^2\n',BASE_CDA);
fprintf('  CoP       = %.1f%% (aero convention: %% wheelbase from front axle)\n', ...
    100*(1-BASE_COP_FRONT));
fprintf('              [internal: %.1f%% of downforce at front axle]\n',100*BASE_COP_FRONT);

fprintf('\nAccel package (measured starting point, flap flat):\n');
fprintf('  CLA_accel = %.2f m^2\n',BASE_CLA_ACCEL);
fprintf('  CdA_accel = %.2f m^2\n',BASE_CDA_ACCEL);
fprintf('  CoP_accel = %.1f%% (aero convention) [internal: %.1f%% of downforce at front]\n', ...
    100*(1-BASE_COP_FRONT_ACCEL),100*BASE_COP_FRONT_ACCEL);
fprintf('              (derived from front-wing-fixed assumption, FIX 9)\n');
fprintf('  Ratios (flap-flat / deployed): CLA=%.4f, CdA=%.4f\n',CLA_ACCEL_RATIO,CDA_ACCEL_RATIO);

fprintf('\nChassis-only drag floor (from coastdown): CdA0 = %.2f m^2\n',CDA0_BARE_CHASSIS);

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

wheel_radius = 0.127;

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
% 6. REAL POWERTRAIN (matches carConfig.m exactly)
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
baseline_runtime_est = toc(t_start);
fprintf('Runtime         = %.2f s\n',baseline_runtime_est);

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
if ~ENABLE_CALIBRATION
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

baseline_3event_points = baseline_points.autox+baseline_points.skidpad+baseline_points.enduro;

fprintf('\nBaseline times (raw physics; calibration disabled):\n');
fprintf('  Autocross      = %.3f s (points %.1f)\n',baseline_cal.auto_time,baseline_points.autox);
fprintf('  Skidpad        = %.3f s (points %.1f)\n',baseline_cal.skidpad_time,baseline_points.skidpad);
fprintf('  Endurance/lap  = %.3f s (points %.1f)\n',baseline_cal.endurance_lap,baseline_points.enduro);
fprintf('  Accel          = %.3f s (points %.1f)\n',baseline_cal.accel_time,baseline_points.accel);
fprintf('  3-EVENT POINTS = %.1f\n',baseline_3event_points);
fprintf('  TOTAL POINTS   = %.1f\n',baseline_points.total);

%% ========================================================================
% 10. CLA x CDA SWEEP -- BASELINE PACKAGE
% FIX 4: CdA_values widened/shifted to 1.00-1.50
% FIX 2: floor CdA_values at the chassis-only drag measurement
% ========================================================================

CLA_values = 2.70:0.10:3.80;
CdA_values = max(1.00:0.05:1.50, CDA0_BARE_CHASSIS);   % FIX 2 + FIX 4

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
% 11. UNCONSTRAINED BEST IN SWEEP -- REFERENCE ONLY, NOT THE TARGET
%
% This will basically always sit at whatever edge of the grid you
% tested, since raw points has no built-in cost for spending more CdA.
% Shown for context/diagnostics only -- see section 13 for the actual
% recommended targets (Methods 1 and 2).
% ========================================================================

[~,best_idx] = max(sweep.points_baseline_total(:));
[best_i,best_j] = ind2sub(size(sweep.points_baseline_total),best_idx);

best_CLA = CLA_values(best_i);
best_CdA = CdA_values(best_j);

fprintf('\n');
fprintf('===============================================================\n');
fprintf('        UNCONSTRAINED BEST IN SWEEP (reference only)\n');
fprintf('===============================================================\n');
fprintf('Best CLA = %.2f m^2, Best CdA = %.2f m^2\n',best_CLA,best_CdA);
fprintf('Best 3-event points (raw physics) = %.1f\n',sweep.points_baseline_total(best_i,best_j));
if best_i==1 || best_i==nCLA || best_j==1 || best_j==nCdA
    fprintf('  (sits at grid edge, as expected -- NOT a recommended target)\n');
end

%% ========================================================================
% 12. NEW FIX 3: BREAK-EVEN BOUNDARY -- BOTH AUTOCROSS AND ENDURANCE
%
% At each CLA, find the CdA where EACH event individually first regresses
% past baseline, then take the tighter (lower) of the two -- that's the
% real cap, since either event regressing is a real loss.
% ========================================================================

auto_time_cal = sweep.auto_time*CAL_AUTOX;
endurance_time_cal = sweep.endurance_time*CAL_ENDURANCE;

baseline_auto_cal = baseline.auto_time*CAL_AUTOX;
baseline_endurance_cal = baseline.endurance_lap_time*CAL_ENDURANCE;

break_even_CdA_auto = NaN(size(CLA_values));
break_even_CdA_end  = NaN(size(CLA_values));

for i = 1:nCLA

    dt_auto = auto_time_cal(i,:) - baseline_auto_cal;
    for j = 1:nCdA-1
        y1 = dt_auto(j); y2 = dt_auto(j+1);
        if y1 == 0
            break_even_CdA_auto(i) = CdA_values(j); break;
        elseif y1*y2 < 0
            break_even_CdA_auto(i) = CdA_values(j) + ...
                (-y1)*(CdA_values(j+1)-CdA_values(j))/(y2-y1);
            break;
        end
    end

    dt_end = endurance_time_cal(i,:) - baseline_endurance_cal;
    for j = 1:nCdA-1
        y1 = dt_end(j); y2 = dt_end(j+1);
        if y1 == 0
            break_even_CdA_end(i) = CdA_values(j); break;
        elseif y1*y2 < 0
            break_even_CdA_end(i) = CdA_values(j) + ...
                (-y1)*(CdA_values(j+1)-CdA_values(j))/(y2-y1);
            break;
        end
    end

end

% tighter of the two (MATLAB's min() naturally treats a NaN as "no
% constraint from that event" and returns the other, finite value)
break_even_CdA_combined = min(break_even_CdA_auto,break_even_CdA_end);

fprintf('\n');
fprintf('===============================================================\n');
fprintf('     BREAK-EVEN BOUNDARY (tighter of autocross vs endurance)\n');
fprintf('===============================================================\n');
fprintf('\nCLA     Auto break-even   Endurance break-even   COMBINED (tighter)\n');
fprintf('---------------------------------------------------------------------\n');
for i = 1:nCLA
    fprintf('%.2f     ', CLA_values(i));
    if isfinite(break_even_CdA_auto(i)), fprintf('%14.3f   ',break_even_CdA_auto(i));
    else, fprintf('%14s   ','outside'); end
    if isfinite(break_even_CdA_end(i)), fprintf('%18.3f   ',break_even_CdA_end(i));
    else, fprintf('%18s   ','outside'); end
    if isfinite(break_even_CdA_combined(i)), fprintf('%15.3f\n',break_even_CdA_combined(i));
    else, fprintf('%15s\n','outside'); end
end

%% ========================================================================
% 13. NEW FIX 7: TARGET QUANTIFICATION -- METHOD 1 AND METHOD 2
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              TARGET QUANTIFICATION (Methods 1 and 2)\n');
fprintf('===============================================================\n');

% -------------------------------------------------------------------
% METHOD 1: marginal return per unit CdA spent above baseline
% -------------------------------------------------------------------

marginal_return = NaN(nCLA,nCdA);

for i = 1:nCLA
    for j = 1:nCdA
        dCdA = CdA_values(j) - BASE_CDA;
        if dCdA > 1e-6
            marginal_return(i,j) = (sweep.points_baseline_total(i,j) - baseline_3event_points) / dCdA;
        end
    end
end

if all(isnan(marginal_return(:)))
    warning('METHOD 1: no CdA values above baseline in the sweep -- widen CdA_values.');
    m1_i = find(abs(CLA_values-BASE_CLA)<1e-9,1); m1_j = find(abs(CdA_values-BASE_CDA)<1e-9,1);
    if isempty(m1_i), [~,m1_i] = min(abs(CLA_values-BASE_CLA)); end
    if isempty(m1_j), [~,m1_j] = min(abs(CdA_values-BASE_CDA)); end
else
    [~,m1_idx] = max(marginal_return(:));
    [m1_i,m1_j] = ind2sub(size(marginal_return),m1_idx);
end

target_CLA_M1 = CLA_values(m1_i);
target_CdA_M1 = CdA_values(m1_j);

target_M1 = evaluate_package( ...
    target_CLA_M1,target_CdA_M1,BASE_COP_FRONT,M, ...
    rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    auto_s,auto_k,end_s,end_k, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

points_M1_3event = sweep.points_baseline_total(m1_i,m1_j);

fprintf('\n--- METHOD 1: marginal return per unit CdA spent ---\n');
fprintf('Target: CLA=%.2f m^2, CdA=%.2f m^2\n',target_CLA_M1,target_CdA_M1);
fprintf('  Delta CLA=%+0.2f, Delta CdA=%+0.2f\n',target_CLA_M1-BASE_CLA,target_CdA_M1-BASE_CDA);
fprintf('  Marginal return at this point = %.2f points per m^2 CdA\n',marginal_return(m1_i,m1_j));
fprintf('  3-event points = %.1f (vs %.1f at baseline)\n',points_M1_3event,baseline_3event_points);
if m1_i==1 || m1_i==nCLA || m1_j==1 || m1_j==nCdA
    fprintf('  NOTE: sits at grid edge -- widen CLA_values/CdA_values and rerun.\n');
end

% -------------------------------------------------------------------
% METHOD 2: slope cutoff along the FIX-3 tightened boundary
% -------------------------------------------------------------------

%% ========================================================================
% 14. NEW FIX 1: LOW-DRAG ACCEL PACKAGE, DERIVED FROM BASELINE TARGETS
%
% No independent accel sweep anymore (FIX 6). The accel package is
% whatever the measured flap-flat ratios imply, applied to EACH baseline
% target found above.
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('       LOW-DRAG ACCEL PACKAGE (derived via fixed ratios)\n');
fprintf('===============================================================\n');

accel_CLA_M1 = target_CLA_M1*CLA_ACCEL_RATIO;
accel_CdA_M1 = target_CdA_M1*CDA_ACCEL_RATIO;
accel_M1 = evaluate_accel_package( ...
    accel_CLA_M1,accel_CdA_M1,BASE_COP_FRONT_ACCEL,M_ACCEL, ...
    rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
    ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);
points_accel_M1 = fsae_points_linear(accel_M1.accel_time*CAL_ACCEL,ACCEL_WIN_TIME,1.5,95.5,4.5);

fprintf('\nMethod 1 baseline target -> accel package (uses baseline CoP):\n');
fprintf('  CLA_accel=%.2f, CdA_accel=%.2f -> accel time = %.3f s (points %.1f)\n', ...
    accel_CLA_M1,accel_CdA_M1,accel_M1.accel_time*CAL_ACCEL,points_accel_M1);

fprintf('\nCurrent (measured) accel package for reference:\n');
fprintf('  CLA_accel=%.2f, CdA_accel=%.2f -> accel time = %.3f s (points %.1f)\n', ...
    BASE_CLA_ACCEL,BASE_CDA_ACCEL,baseline_accel.accel_time*CAL_ACCEL,baseline_points.accel);

%% ========================================================================
% 14b. NEW FIX 10: CoP TARGET -- ALL 4 EVENTS, AT THE RECOMMENDED AERO
%
% Evaluated at BASELINE CLA/CdA (BASE_CLA, BASE_CDA) -- treated as a
% separate sweep/target from the CLA/CdA recommendation above, since
% sweeping CoP at the recommended (more aggressive) CLA/CdA was
% producing unreasonable CoP results. CoP doesn't need CLA/CdA's
% marginal-return trick: it's a fixed pool of downforce being
% REDISTRIBUTED, not a quantity you keep spending more of, so each
% event's time-vs-CoP curve should already have a genuine interior
% optimum -- a plain points-argmax is appropriate here.
%
% Accel's CoP is NOT independently swept -- per FIX 9, only the rear
% flap changes for low-drag mode, so accel's front force fraction is
% FORCED by whatever the deployed-wing CoP is:
%     accel_CoP(CoP_test) = CoP_test / CLA_ACCEL_RATIO
% applied to the BASELINE accel package (BASE_CLA_ACCEL/BASE_CDA_ACCEL),
% consistent with sweeping baseline CLA/CdA above, not the target's
% derived accel package. All 4 events are scored with their REAL
% competition points formulas (125/75/275/100 max) and summed.
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('     CoP TARGET (all 4 events, at baseline CLA/CdA)\n');
fprintf('===============================================================\n');

CoP_target_values = 0.10:0.01:0.75;   % widened downward (edge hit at 0.30) + finer step (was 0.02)
nCoPtarget = numel(CoP_target_values);

cop_target_sweep = struct();
cop_target_sweep.CoP_front = CoP_target_values;
cop_target_sweep.auto_time = NaN(1,nCoPtarget);
cop_target_sweep.skidpad_time = NaN(1,nCoPtarget);
cop_target_sweep.endurance_time = NaN(1,nCoPtarget);
cop_target_sweep.endurance_10lap_time = NaN(1,nCoPtarget);
cop_target_sweep.accel_time = NaN(1,nCoPtarget);
cop_target_sweep.points_autox = NaN(1,nCoPtarget);
cop_target_sweep.points_skid = NaN(1,nCoPtarget);
cop_target_sweep.points_end = NaN(1,nCoPtarget);
cop_target_sweep.points_accel = NaN(1,nCoPtarget);
cop_target_sweep.points_total_4event = NaN(1,nCoPtarget);

n_clipped = 0;

for i = 1:nCoPtarget

    CoP_test = CoP_target_values(i);

    r = evaluate_package( ...
        BASE_CLA,BASE_CDA,CoP_test,M, ...   % using baseline CLA/CdA here, not target_CLA_M1/CdA_M1
        rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fy_max_table,Fx_max_table, ...
        auto_s,auto_k,end_s,end_k, ...
        N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

    CoP_accel_test = CoP_test / CLA_ACCEL_RATIO;
    CoP_accel_test_clipped = min(max(CoP_accel_test,0),1);
    if abs(CoP_accel_test_clipped-CoP_accel_test) > 1e-9
        n_clipped = n_clipped+1;
    end

    ra = evaluate_accel_package( ...
        BASE_CLA_ACCEL,BASE_CDA_ACCEL,CoP_accel_test_clipped,M_ACCEL, ...   % baseline accel package, not accel_CLA_M1/CdA_M1
        rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
        ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

    cop_target_sweep.auto_time(i) = r.auto_time;
    cop_target_sweep.skidpad_time(i) = r.skidpad_time;
    cop_target_sweep.endurance_time(i) = r.endurance_lap_time;
    cop_target_sweep.endurance_10lap_time(i) = r.endurance_10lap_time;
    cop_target_sweep.accel_time(i) = ra.accel_time;

    pa = fsae_points_linear(r.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5);
    ps = fsae_points_skidpad(r.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME);
    pe = fsae_points_linear(r.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);
    pacc = fsae_points_linear(ra.accel_time*CAL_ACCEL,ACCEL_WIN_TIME,1.5,95.5,4.5);

    % NEW: save per-event points, not just the combined total -- lets you
    % see directly which event(s) are driving the combined curve's shape,
    % rather than trusting the sum alone.
    cop_target_sweep.points_autox(i) = pa;
    cop_target_sweep.points_skid(i) = ps;
    cop_target_sweep.points_end(i) = pe;
    cop_target_sweep.points_accel(i) = pacc;
    cop_target_sweep.points_total_4event(i) = pa+ps+pe+pacc;

end

if n_clipped > 0
    fprintf('\nNOTE: %d of %d tested CoP values implied an out-of-range accel CoP\n',n_clipped,nCoPtarget);
    fprintf('(clipped to [0,1] -- treat those points with caution).\n');
end

[~,cop_target_idx] = max(cop_target_sweep.points_total_4event);
target_COP = CoP_target_values(cop_target_idx);

fprintf('\nBest CoP (all 4 events) = %.1f%% (aero convention) [internal: %.1f%% front]\n', ...
    100*(1-target_COP),100*target_COP);
fprintf('4-event points at this CoP = %.1f\n',cop_target_sweep.points_total_4event(cop_target_idx));
if cop_target_idx==1 || cop_target_idx==nCoPtarget
    fprintf('  NOTE: sits at grid edge -- widen CoP_target_values and rerun.\n');
end

%% ========================================================================
% 14c. CoP TARGET PACKAGE -- baseline CLA/CdA + the CoP found above
%
% This is a SEPARATE finding, not merged with target_M1/accel_M1 above.
% We're treating the CLA/CdA sweep and the CoP sweep as two independent
% analyses; the Final Target Summary below reports both side by side
% rather than combining them into one package.
% ========================================================================

accel_CoP_final = min(max(target_COP / CLA_ACCEL_RATIO,0),1);

cop_target_package = evaluate_package( ...
    BASE_CLA,BASE_CDA,target_COP,M, ...
    rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    auto_s,auto_k,end_s,end_k, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

points_cop_target_3event = fsae_points_linear(cop_target_package.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5) + ...
    fsae_points_skidpad(cop_target_package.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME) + ...
    fsae_points_linear(cop_target_package.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);

accel_cop_target = evaluate_accel_package( ...
    BASE_CLA_ACCEL,BASE_CDA_ACCEL,accel_CoP_final,M_ACCEL, ...
    rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
    ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);
points_accel_cop_target = fsae_points_linear(accel_cop_target.accel_time*CAL_ACCEL,ACCEL_WIN_TIME,1.5,95.5,4.5);

fprintf('\nCoP target package (baseline CLA/CdA + target CoP):\n');
fprintf('  Autocross      = %.3f s\n',cop_target_package.auto_time*CAL_AUTOX);
fprintf('  Skidpad        = %.3f s\n',cop_target_package.skidpad_time*CAL_SKIDPAD);
fprintf('  Endurance/lap  = %.3f s\n',cop_target_package.endurance_lap_time*CAL_ENDURANCE);
fprintf('  Accel_CoP = %.1f%% (aero convention) [internal: %.1f%% front]\n', ...
    100*(1-accel_CoP_final),100*accel_CoP_final);
fprintf('  Accel time     = %.3f s\n',accel_cop_target.accel_time*CAL_ACCEL);

%% ========================================================================
% 15. FINAL TARGET SUMMARY
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    FINAL TARGET SUMMARY\n');
fprintf('===============================================================\n');

total_M1 = points_M1_3event + points_accel_M1;
total_cop_target = points_cop_target_3event + points_accel_cop_target;

fprintf('\n--- FINDING 1: CLA/CdA target (Method 1), at baseline CoP ---\n');
fprintf('Baseline: CLA=%.2f, CdA=%.2f, CoP=%.1f%% (aero convention)\n',target_CLA_M1,target_CdA_M1,100*(1-BASE_COP_FRONT));
fprintf('Accel:    CLA_accel=%.2f, CdA_accel=%.2f, CoP_accel=%.1f%% (aero convention)\n', ...
    accel_CLA_M1,accel_CdA_M1,100*(1-BASE_COP_FRONT_ACCEL));
fprintf('  Autocross (raw)      = %.3f s\n',target_M1.auto_time*CAL_AUTOX);
fprintf('  Skidpad (raw)        = %.3f s\n',target_M1.skidpad_time*CAL_SKIDPAD);
fprintf('  Endurance/lap (raw)  = %.3f s\n',target_M1.endurance_lap_time*CAL_ENDURANCE);
fprintf('  Accel (raw)          = %.3f s\n',accel_M1.accel_time*CAL_ACCEL);
fprintf('  COMBINED 4-EVENT POINTS = %.1f (vs %.1f at current baseline)\n',total_M1,baseline_points.total);

fprintf('\n--- FINDING 2: CoP target, at baseline CLA/CdA ---\n');
fprintf('Baseline: CLA=%.2f, CdA=%.2f, CoP=%.1f%% (aero convention)\n',BASE_CLA,BASE_CDA,100*(1-target_COP));
fprintf('Accel:    CLA_accel=%.2f, CdA_accel=%.2f, CoP_accel=%.1f%% (aero convention)\n', ...
    BASE_CLA_ACCEL,BASE_CDA_ACCEL,100*(1-accel_CoP_final));
fprintf('  Autocross (raw)      = %.3f s\n',cop_target_package.auto_time*CAL_AUTOX);
fprintf('  Skidpad (raw)        = %.3f s\n',cop_target_package.skidpad_time*CAL_SKIDPAD);
fprintf('  Endurance/lap (raw)  = %.3f s\n',cop_target_package.endurance_lap_time*CAL_ENDURANCE);
fprintf('  Accel (raw)          = %.3f s\n',accel_cop_target.accel_time*CAL_ACCEL);
fprintf('  COMBINED 4-EVENT POINTS = %.1f (vs %.1f at current baseline)\n',total_cop_target,baseline_points.total);

fprintf('\nThese are two SEPARATE findings (independent sweeps), not a combined\n');
fprintf('package -- combining a new CLA/CdA target with a new CoP target\n');
fprintf('simultaneously has not been evaluated together here.\n');

%% ========================================================================
% NEW: MASS SENSITIVITY SWEEP -- INDEPENDENT OF CLA/CdA/CoP
%
% Per team discussion: the new wing is expected to be roughly mass-
% neutral vs B26 (smaller endplates offsetting other additions), but the
% new undertray's mass impact isn't pinned down yet. Rather than guess a
% number and bake it into the CLA/CdA sweep, this sweeps MASS ALONE --
% holding CLA/CdA/CoP fixed at baseline -- to show directly how
% sensitive each event is to +/- mass, independent of any aero decision.
% ========================================================================

MASS_DELTA_KG = -3:0.5:3;   % EDIT: kg, added to/subtracted from current baseline mass
nMass = numel(MASS_DELTA_KG);

fprintf('\n');
fprintf('===============================================================\n');
fprintf('     MASS SENSITIVITY SWEEP (CLA/CdA/DF split held at baseline)\n');
fprintf('===============================================================\n');
fprintf('Baseline mass (car+driver) = %.1f kg (%.1f lb)\n',M,M/0.45359237);

mass_sweep = struct();
mass_sweep.mass_delta_kg = MASS_DELTA_KG;
mass_sweep.mass_total_kg = M + MASS_DELTA_KG;
mass_sweep.auto_time = NaN(1,nMass);
mass_sweep.skidpad_time = NaN(1,nMass);
mass_sweep.endurance_time = NaN(1,nMass);
mass_sweep.points_total = NaN(1,nMass);

for i = 1:nMass

    M_i = M + MASS_DELTA_KG(i);

    r = evaluate_package( ...
        BASE_CLA,BASE_CDA,BASE_COP_FRONT,M_i, ...
        rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fy_max_table,Fx_max_table, ...
        auto_s,auto_k,end_s,end_k, ...
        N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

    mass_sweep.auto_time(i) = r.auto_time;
    mass_sweep.skidpad_time(i) = r.skidpad_time;
    mass_sweep.endurance_time(i) = r.endurance_lap_time;

    pa = fsae_points_linear(r.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5);
    ps = fsae_points_skidpad(r.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME);
    pe = fsae_points_linear(r.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);
    mass_sweep.points_total(i) = pa+ps+pe;

    fprintf('  Mass %+.1f kg (%.1f kg total, %.1f lb): auto=%.3fs  skid=%.3fs  end=%.3fs  points=%.1f\n', ...
        MASS_DELTA_KG(i),M_i,M_i/0.45359237,r.auto_time,r.skidpad_time,r.endurance_lap_time,mass_sweep.points_total(i));

end

% simple linear sensitivity, for a quick "X points per kg" reference
mass_slope_points = polyfit(MASS_DELTA_KG,mass_sweep.points_total,1);
mass_slope_endurance = polyfit(MASS_DELTA_KG,mass_sweep.endurance_time,1);

fprintf('\nApprox sensitivity (linear fit across the swept range):\n');
fprintf('  %.3f points per kg\n',mass_slope_points(1));
fprintf('  %.4f s per kg (endurance lap time)\n',mass_slope_endurance(1));

figure('Name','Mass Sensitivity - Points','Position',[300 150 1000 700]);
plot(MASS_DELTA_KG,mass_sweep.points_total,'o-','LineWidth',2); hold on;
xline(0,'--','Current baseline mass');
xlabel('Mass delta from current baseline (kg)'); ylabel('3-event points (raw physics)');
title('Mass Sensitivity (CLA/CdA/DF split held at baseline)');
grid on;

figure('Name','Mass Sensitivity - Event Times','Position',[350 150 1000 800]);
subplot(3,1,1);
plot(MASS_DELTA_KG,mass_sweep.auto_time,'o-','LineWidth',2); hold on;
xline(0,'--');
xlabel('Mass delta (kg)'); ylabel('Autocross time (s)'); title('Autocross'); grid on;
subplot(3,1,2);
plot(MASS_DELTA_KG,mass_sweep.skidpad_time,'o-','LineWidth',2); hold on;
xline(0,'--');
xlabel('Mass delta (kg)'); ylabel('Skidpad time (s)'); title('Skidpad'); grid on;
subplot(3,1,3);
plot(MASS_DELTA_KG,mass_sweep.endurance_time,'o-','LineWidth',2); hold on;
xline(0,'--');
xlabel('Mass delta (kg)'); ylabel('Endurance lap time (s)'); title('Endurance'); grid on;

%% ========================================================================
% NEW: MARGINAL SCREENING RATIO (self-service, for future CFD iterations)
%
% Local sensitivity of 3-event points to CLA and CdA, evaluated AT THE
% CURRENT BASELINE via central differences on the already-computed
% sweep grid (CoP held at baseline for this rate -- does not separately
% account for a simultaneous CoP shift; see the CoP sensitivity plots
% for that effect on its own).
%
% Gives a BREAK-EVEN ratio: how much CLA gain (m^2) is needed per m^2
% of CdA cost to stay net points-neutral, for SMALL changes near the
% current baseline. This is a LOCAL LINEAR approximation -- always
% double-check anything close to this ratio, or any large change, with
% a real lapsim run rather than trusting the rule alone.
% ========================================================================

base_i = find(abs(CLA_values-BASE_CLA)<1e-9,1);
base_j = find(abs(CdA_values-BASE_CDA)<1e-9,1);

if isempty(base_i) || isempty(base_j) || base_i==1 || base_i==nCLA || base_j==1 || base_j==nCdA
    warning('Baseline CLA/CdA not centered on the sweep grid -- cannot compute local screening ratio this way.');
else

    dCLA_step = CLA_values(base_i+1)-CLA_values(base_i-1);
    dPdCLA = (sweep.points_baseline_total(base_i+1,base_j) - sweep.points_baseline_total(base_i-1,base_j)) / dCLA_step;

    dCdA_step = CdA_values(base_j+1)-CdA_values(base_j-1);
    dPdCdA = (sweep.points_baseline_total(base_i,base_j+1) - sweep.points_baseline_total(base_i,base_j-1)) / dCdA_step;

    breakeven_ratio = -dPdCdA/dPdCLA;

    fprintf('\n');
    fprintf('===============================================================\n');
    fprintf('        MARGINAL SCREENING RATIO (for future CFD iterations)\n');
    fprintf('===============================================================\n');
    fprintf('\nLocal sensitivity at current baseline (CLA=%.2f, CdA=%.2f):\n',BASE_CLA,BASE_CDA);
    fprintf('  dPoints/dCLA = %+.2f points per m^2 CLA\n',dPdCLA);
    fprintf('  dPoints/dCdA = %+.2f points per m^2 CdA\n',dPdCdA);
    fprintf('\nBREAK-EVEN RATIO: need Delta_CLA / Delta_CdA > %.2f to be points-positive\n',breakeven_ratio);
    fprintf('  (i.e., every +0.01 m^2 of CdA needs at least +%.4f m^2 of CLA to break even)\n',breakeven_ratio*0.01);
    fprintf('\nUse this to screen candidate CFD numbers yourself (see aero_candidate_tester.m\n');
    fprintf('for actually running a specific candidate through the full physics model).\n');

end

%% ========================================================================
% 16. CoP SENSITIVITY DIAGNOSTIC (at TODAY'S baseline, not the target)
%
% NOTE: this is a diagnostic only -- "if we kept today's CLA/CdA, how
% sensitive is each event to CoP." It does NOT set the CoP target; that
% happens in section 14b, evaluated at the RECOMMENDED CLA/CdA instead.
% FIX 5: both ranges -> 0.30:0.02:0.75 front
% ========================================================================

CoP_values = 0.30:0.02:0.75;

cop_results.auto_time = zeros(size(CoP_values));
cop_results.skidpad_time = zeros(size(CoP_values));
cop_results.endurance_time = zeros(size(CoP_values));
cop_results.points_3event = zeros(size(CoP_values));

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

    pa = fsae_points_linear(r.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5);
    ps = fsae_points_skidpad(r.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME);
    pe = fsae_points_linear(r.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);
    cop_results.points_3event(i) = pa+ps+pe;

end

CoP_accel_values = 0.30:0.02:0.75;   % FIX 5

accel_cop_results.accel_time = zeros(size(CoP_accel_values));

for i = 1:numel(CoP_accel_values)

    ra = evaluate_accel_package( ...
        accel_CLA_M1,accel_CdA_M1,CoP_accel_values(i),M_ACCEL, ...
        rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
        ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

    accel_cop_results.accel_time(i) = ra.accel_time;

end

%% ========================================================================
% 17. PLOTS
% ========================================================================

figure('Name','CoP Target Sweep (4-event)','Position',[80 80 1000 700]);
plot(100*(1-CoP_target_values),cop_target_sweep.points_total_4event,'o-','LineWidth',2);
hold on;
plot(100*(1-target_COP),cop_target_sweep.points_total_4event(cop_target_idx), ...
    'ks','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','w');
xlabel('CoP (% wheelbase from front axle)'); ylabel('4-event points (raw physics)');
title('CoP Target: Combined 4-Event Points vs. CoP (at recommended CLA/CdA)');
legend('4-event points','Target CoP','Location','best');
grid on;

% NEW: per-event decomposition -- shows directly which event(s) are
% driving the combined curve's shape/direction, since the FSAE points
% formulas have very different steepness per event (a small time change
% in a ~4s accel run can swing points far more than the same absolute
% time change in a ~147s endurance lap).
figure('Name','CoP Target Sweep - Per-Event Decomposition','Position',[100 100 1100 750]);
plot(100*(1-CoP_target_values),cop_target_sweep.points_autox,'o-','LineWidth',2,'DisplayName','Autocross'); hold on;
plot(100*(1-CoP_target_values),cop_target_sweep.points_skid,'o-','LineWidth',2,'DisplayName','Skidpad');
plot(100*(1-CoP_target_values),cop_target_sweep.points_end,'o-','LineWidth',2,'DisplayName','Endurance');
plot(100*(1-CoP_target_values),cop_target_sweep.points_accel,'o-','LineWidth',2,'DisplayName','Accel');
xline(100*(1-target_COP),'k--','Target CoP');
xlabel('CoP (% wheelbase from front axle)'); ylabel('Points, this event only');
title('Which Event(s) Are Driving the Combined CoP Target?');
legend('Location','best');
grid on;

[CdA_mesh,CLA_mesh] = meshgrid(CdA_values,CLA_values);

figure('Name','CLA CdA Points Surface - Baseline','Position',[100 100 1100 750]);
surf(CdA_mesh,CLA_mesh,sweep.points_baseline_total,'EdgeColor','k','FaceAlpha',0.85);
hold on;
plot3(BASE_CDA,BASE_CLA,baseline_3event_points,'ko','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');
plot3(target_CdA_M1,target_CLA_M1,points_M1_3event,'ks','MarkerSize',11,'LineWidth',3,'MarkerFaceColor','w');
plot3(best_CdA,best_CLA,sweep.points_baseline_total(best_i,best_j),'k^','MarkerSize',9,'LineWidth',2,'MarkerFaceColor',[0.6 0.6 0.6]);
xlabel('CdA (m^2)'); ylabel('CLA (m^2)'); zlabel('3-event points (raw physics)');
title('Baseline CLA / CdA vs. Competition Points');
legend('Points surface','Current baseline','Method 1 target', ...
    'Unconstrained best (reference)','Location','best');
grid on; view(-40,28); colorbar;

figure('Name','CLA CdA Tradeoff Boundary (Fix 3)','Position',[200 200 1000 700]);
plot(CLA_values,break_even_CdA_auto,'o--','LineWidth',1.5,'Color',[0.6 0.6 0.9],'DisplayName','Autocross break-even');
hold on;
plot(CLA_values,break_even_CdA_end,'o--','LineWidth',1.5,'Color',[0.9 0.6 0.6],'DisplayName','Endurance break-even');
plot(CLA_values,break_even_CdA_combined,'ko-','LineWidth',2.5,'MarkerSize',6,'DisplayName','Combined (tighter)');
yline(BASE_CDA,'--','Current CdA');
xline(BASE_CLA,'--','Current CLA');
plot(target_CLA_M1,target_CdA_M1,'ks','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','w','DisplayName','Method 1 target');
xlabel('CLA (m^2)'); ylabel('Maximum CdA before regression (m^2)');
title('Aero Tradeoff Boundary: Autocross AND Endurance Must Not Regress');
legend('Location','best');
grid on;

figure('Name','Marginal Return per Unit CdA (Method 1)','Position',[250 200 1000 700]);
surf(CdA_mesh,CLA_mesh,marginal_return,'EdgeColor','k','FaceAlpha',0.85);
hold on;
plot3(target_CdA_M1,target_CLA_M1,marginal_return(m1_i,m1_j),'ks','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','w');
xlabel('CdA (m^2)'); ylabel('CLA (m^2)'); zlabel('Points gained per unit CdA spent');
title('Method 1: Marginal Return per Unit CdA Spent Above Baseline');
grid on; view(-40,28); colorbar;

figure('Name','CoP Sensitivity - Baseline (aero convention)','Position',[150 150 1000 800]);

subplot(2,2,1);
plot(100*(1-CoP_values),cop_results.auto_time*CAL_AUTOX,'o-','LineWidth',2); hold on;
xlabel('CoP (% wheelbase from front axle)'); ylabel('Autocross time (s)'); title('Autocross'); grid on;

subplot(2,2,2);
plot(100*(1-CoP_values),cop_results.skidpad_time*CAL_SKIDPAD,'o-','LineWidth',2); hold on;
xlabel('CoP (% wheelbase from front axle)'); ylabel('Skidpad time (s)'); title('Skidpad'); grid on;

subplot(2,2,3);
plot(100*(1-CoP_values),cop_results.endurance_time*CAL_ENDURANCE,'o-','LineWidth',2); hold on;
xlabel('CoP (% wheelbase from front axle)'); ylabel('Endurance lap time (s)'); title('Endurance'); grid on;

subplot(2,2,4);
plot(100*(1-CoP_values),cop_results.points_3event,'o-','LineWidth',2); hold on;
xlabel('CoP (% wheelbase from front axle)'); ylabel('3-event points (raw physics)'); title('Combined (auto+skid+endurance)'); grid on;

figure('Name','CoP Sensitivity - Accel (aero convention, at Method 1 target)','Position',[180 180 700 500]);
plot(100*(1-CoP_accel_values),accel_cop_results.accel_time*CAL_ACCEL,'o-','LineWidth',2); hold on;
xlabel('CoP (% wheelbase from front axle)'); ylabel('Accel time (s)');
title({'Accel Time vs. CoP Position (at Method 1 CLA_accel/CdA_accel)', ...
    '(0% = CoP at front axle, 100% = CoP at rear axle)'});
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
aero_spec_results.sweep = sweep;
aero_spec_results.CDA0_BARE_CHASSIS = CDA0_BARE_CHASSIS;
aero_spec_results.break_even = struct('auto',break_even_CdA_auto,'endurance',break_even_CdA_end, ...
    'combined',break_even_CdA_combined);
aero_spec_results.method1 = struct('CLA',target_CLA_M1,'CdA',target_CdA_M1,'target',target_M1, ...
    'accel_CLA',accel_CLA_M1,'accel_CdA',accel_CdA_M1,'accel',accel_M1,'total_points',total_M1, ...
    'CoP',BASE_COP_FRONT,'accel_CoP',BASE_COP_FRONT_ACCEL);
aero_spec_results.cop_target = struct('CoP',target_COP,'accel_CoP',accel_CoP_final, ...
    'package',cop_target_package,'accel',accel_cop_target,'total_points',total_cop_target);
aero_spec_results.cop_target_sweep = cop_target_sweep;
aero_spec_results.mass_sweep = mass_sweep;
aero_spec_results.CLA_ACCEL_RATIO = CLA_ACCEL_RATIO;
aero_spec_results.CDA_ACCEL_RATIO = CDA_ACCEL_RATIO;

assignin('base','aero_spec_results',aero_spec_results);
save('aero_spec_results_v3_1.mat','aero_spec_results');

fprintf('\n===============================================================\n');
fprintf('                         COMPLETE\n');
fprintf('===============================================================\n');

%% ========================================================================
% LOCAL FUNCTIONS (unchanged from v3)
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

    Fy_demand_total = M*ay;
    Fy_demand = Fy_demand_total * (Fz/sum(Fz));

    Fx_rear = tire_longitudinal_capacity_combined( ...
        Fz(3:4),Fy_demand(3:4),Fz_grid,Fx_max_table,Fy_max_table);

    Fx_tire = sum(Fx_rear);

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
        Fx_demand = [0,0,Fdrag/2,Fdrag/2];

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



%{

%==================================
%Finer cop setting
%% ========================================================================
% CoP TARGET QUICK COMPARE (fast, standalone)
%
% PURPOSE: run ONLY the CoP-target sweep (all 4 events, real points
% formulas, accel CoP forced via the front-wing-fixed relationship) at
% TWO named CLA/CdA operating points, for a direct side-by-side. Skips
% the expensive CLA x CdA grid sweep entirely -- you supply both
% configs' CLA/CdA directly below, since you already know them.
%
% Same physics engine as aero_spec_fast_model, verified identical.
%% ========================================================================

clc;
clear;
close all;

%% ========================================================================
% 1. USER INPUT -- EDIT THIS SECTION
%% ========================================================================

configs = struct('name',{},'CLA',{},'CdA',{});
configs(1) = struct('name','Current Baseline', 'CLA',2.80,'CdA',1.35);

CoP_target_values = 0.47:0.001:0.52;   % zoomed in around previous ~0.50 result, much finer step

%% ========================================================================
% 2. FIXED CONSTANTS (measured accel package + ratios -- do not change
% per config; these are fixed regardless of which CLA/CdA you're
% testing CoP for, since the ratio itself is what's measured)
%% ========================================================================

BASE_CLA = 2.8;          % current measured baseline -- used ONLY to derive the ratios below
BASE_CDA = 1.35;
BASE_CLA_ACCEL = 2.10;   % measured, flap flat
BASE_CDA_ACCEL = 1.01;   % measured, flap flat

CLA_ACCEL_RATIO = BASE_CLA_ACCEL/BASE_CLA;
CDA_ACCEL_RATIO = BASE_CDA_ACCEL/BASE_CDA;

BASE_CAR_MASS      = 162.8;
BASE_DRIVER_MASS   = 64;

ENABLE_CALIBRATION = false;
CAL_AUTOX = 1; CAL_ENDURANCE = 1; CAL_SKIDPAD = 1; CAL_ACCEL = 1;

SKIDPAD_WIN_TIME   = 4.878;
ACCEL_WIN_TIME     = 4.126;
AUTOX_WIN_TIME     = 46.911;
ENDURANCE_WIN_TIME = 1395.891;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              CoP TARGET QUICK COMPARE (fast)\n');
fprintf('===============================================================\n');
fprintf('\nFixed ratios (from measured flap-flat accel package):\n');
fprintf('  CLA_ACCEL_RATIO = %.4f\n',CLA_ACCEL_RATIO);
fprintf('  CDA_ACCEL_RATIO = %.4f\n',CDA_ACCEL_RATIO);

%% ========================================================================
% 3. VEHICLE PARAMETERS
%% ========================================================================

rho = 1.225;
g = 9.81;

M       = BASE_CAR_MASS + BASE_DRIVER_MASS;
M_ACCEL = M;   % accel package uses same total mass here (driver mass not split out in this quick script)

Wb = 62*0.0254;
track = 47*0.0254;

weight_dist_rear = 0.521;
lf = Wb*weight_dist_rear;
lr = Wb*(1-weight_dist_rear);

hg = 0.281;
hrf = 0.0876;
hrr = 0.0914;
hrc = (hrf+hrr)/2;
Rsf = 0.34;

wheel_radius = 0.127;

N_ITER = 3;
V_CORNER = linspace(5,35,81);
CURVATURE_CUTOFF = 0.008;
TIRE_UTILIZATION = 0.95;

ACCEL_ROLLOUT_M = 2.19;
ACCEL_TIMED_M   = 75;
ACCEL_DS        = 0.05;

N_AUTO = 1500;
N_END  = 1500;

%% ========================================================================
% 4. TIRE MODEL, POWERTRAIN, TRACKS, TIRE LOOKUP TABLES (setup only)
%% ========================================================================

fprintf('\nLoading tire model...\n');

load('Fx_combined_parameters_run38_30.mat');
Fx_parameters = cell2mat(Xbestcell);

load('Lapsim_Fy_combined_parameters_1965run15.mat');
Fy_parameters = cell2mat(Xbestcell);

tire_pressure = 12;
friction_scaling_factor = 1.05*0.55;

tire = Tire2(tire_pressure,Fx_parameters,Fy_parameters,friction_scaling_factor);

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

torque_fn = KTM450();

powertrain = Powertrain( ...
    eParams.redline,eParams.shift_point,eParams.gears,eParams.primary_reduction, ...
    torque_fn,eParams.shift_time,DTparams.final_drive,wheel_radius, ...
    DTparams.drivetrain_efficiency,DTparams.G_d1,DTparams.G_d2_overrun, ...
    DTparams.G_d2_driving,Bparams.brake_distribution,Bparams.max_braking_torque);

fprintf('Loading tracks...\n');

load('michigantrack.mat');
auto_s_raw = arclength(:)';
auto_k_raw = curvature(:)';

load('2024endurancetrack.mat');
end_s_raw = arclength(:)';
end_k_raw = curvature(:)';

auto_s = linspace(auto_s_raw(1),auto_s_raw(end),N_AUTO);
auto_k = interp1(auto_s_raw,auto_k_raw,auto_s,'linear');

end_s = linspace(end_s_raw(1),end_s_raw(end),N_END);
end_k = interp1(end_s_raw,end_k_raw,end_s,'linear');

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

fprintf('Setup complete.\n');

%% ========================================================================
% 5. RUN THE CoP TARGET SWEEP FOR EACH CONFIG
%% ========================================================================

nConfigs = numel(configs);
nCoP = numel(CoP_target_values);

results = struct();

for c = 1:nConfigs

    fprintf('\n');
    fprintf('===============================================================\n');
    fprintf('  %s (CLA=%.2f, CdA=%.2f)\n',configs(c).name,configs(c).CLA,configs(c).CdA);
    fprintf('===============================================================\n');

    accel_CLA = configs(c).CLA * CLA_ACCEL_RATIO;
    accel_CdA = configs(c).CdA * CDA_ACCEL_RATIO;
    fprintf('  Derived accel package: CLA_accel=%.3f, CdA_accel=%.3f\n',accel_CLA,accel_CdA);

    pts_autox = NaN(1,nCoP);
    pts_skid  = NaN(1,nCoP);
    pts_end   = NaN(1,nCoP);
    pts_accel = NaN(1,nCoP);
    pts_total = NaN(1,nCoP);

    n_clipped = 0;

    for i = 1:nCoP

        CoP_test = CoP_target_values(i);

        r = evaluate_package( ...
            configs(c).CLA,configs(c).CdA,CoP_test,M, ...
            rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
            powertrain,wheel_radius, ...
            Fz_grid,Fy_max_table,Fx_max_table, ...
            auto_s,auto_k,end_s,end_k, ...
            N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

        CoP_accel_test = CoP_test / CLA_ACCEL_RATIO;
        CoP_accel_test_clipped = min(max(CoP_accel_test,0),1);
        if abs(CoP_accel_test_clipped-CoP_accel_test) > 1e-9
            n_clipped = n_clipped+1;
        end

        ra = evaluate_accel_package( ...
            accel_CLA,accel_CdA,CoP_accel_test_clipped,M_ACCEL, ...
            rho,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
            powertrain,wheel_radius, ...
            Fz_grid,Fx_max_table,TIRE_UTILIZATION, ...
            ACCEL_ROLLOUT_M,ACCEL_TIMED_M,ACCEL_DS);

        pts_autox(i) = fsae_points_linear(r.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5);
        pts_skid(i)  = fsae_points_skidpad(r.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME);
        pts_end(i)   = fsae_points_linear(r.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);
        pts_accel(i) = fsae_points_linear(ra.accel_time*CAL_ACCEL,ACCEL_WIN_TIME,1.5,95.5,4.5);
        pts_total(i) = pts_autox(i)+pts_skid(i)+pts_end(i)+pts_accel(i);

    end

    [~,best_idx] = max(pts_total);
    best_CoP = CoP_target_values(best_idx);

    fprintf('\n  Best CoP = %.1f%% (aero convention) [internal: %.1f%% front]\n', ...
        100*(1-best_CoP),100*best_CoP);
    fprintf('  4-event points at this CoP = %.1f\n',pts_total(best_idx));
    if best_idx==1 || best_idx==nCoP
        fprintf('  NOTE: sits at grid edge -- widen CoP_target_values and rerun.\n');
    end
    if n_clipped > 0
        fprintf('  NOTE: %d of %d tested CoP values implied an out-of-range accel CoP (clipped).\n',n_clipped,nCoP);
    end

    results(c).name = configs(c).name;
    results(c).CLA = configs(c).CLA;
    results(c).CdA = configs(c).CdA;
    results(c).accel_CLA = accel_CLA;
    results(c).accel_CdA = accel_CdA;
    results(c).CoP_target_values = CoP_target_values;
    results(c).pts_autox = pts_autox;
    results(c).pts_skid = pts_skid;
    results(c).pts_end = pts_end;
    results(c).pts_accel = pts_accel;
    results(c).pts_total = pts_total;
    results(c).best_CoP = best_CoP;
    results(c).best_points = pts_total(best_idx);

end

%% ========================================================================
% 6. SIDE-BY-SIDE COMPARISON
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    SIDE-BY-SIDE COMPARISON\n');
fprintf('===============================================================\n');
fprintf('\n%-20s %10s %10s %14s %12s\n','Config','CLA','CdA','Best CoP(%)','Points');
fprintf('---------------------------------------------------------------------\n');
for c = 1:nConfigs
    fprintf('%-20s %10.2f %10.2f %14.1f %12.1f\n', ...
        results(c).name,results(c).CLA,results(c).CdA, ...
        100*(1-results(c).best_CoP),results(c).best_points);
end

if nConfigs == 2
    dCoP_aero = 100*(1-results(2).best_CoP) - 100*(1-results(1).best_CoP);
    dPoints = results(2).best_points - results(1).best_points;
    fprintf('\nDelta (Config 2 - Config 1):\n');
    fprintf('  Best CoP shift  = %+.1f percentage points (aero convention)\n',dCoP_aero);
    fprintf('  Points          = %+.1f\n',dPoints);
end

%% ========================================================================
% 7. PLOT -- BOTH CONFIGS OVERLAID
%% ========================================================================

figure('Name','CoP Target - Current Baseline','Position',[100 100 1000 700]);
hold on;
colors = lines(nConfigs);
for c = 1:nConfigs
    plot(100*(1-results(c).CoP_target_values),results(c).pts_total,'o-', ...
        'LineWidth',2,'Color',colors(c,:),'DisplayName',results(c).name);
    plot(100*(1-results(c).best_CoP),results(c).best_points,'s','MarkerSize',12, ...
        'LineWidth',2,'Color',colors(c,:),'MarkerFaceColor','w','HandleVisibility','off');
end
xlabel('CoP (% wheelbase from front axle)'); ylabel('4-event points (raw physics)');
title('CoP Target: Combined 4-Event Points vs. CoP');
legend('Location','best');
grid on;

%% ========================================================================
% 7b. PER-EVENT DECOMPOSITION -- ONE FIGURE PER CONFIG
%
% Shows directly whether one event's points curve is still climbing
% toward the found optimum while the others have already peaked and
% turned back down -- i.e. whether the combined result is being
% dominated by a single event rather than genuinely balancing all 4.
% ========================================================================

for c = 1:nConfigs
    figure('Name',sprintf('Per-Event Decomposition - %s',results(c).name),'Position',[150+50*c 150 1100 750]);
    plot(100*(1-results(c).CoP_target_values),results(c).pts_autox,'o-','LineWidth',2,'DisplayName','Autocross'); hold on;
    plot(100*(1-results(c).CoP_target_values),results(c).pts_skid,'o-','LineWidth',2,'DisplayName','Skidpad');
    plot(100*(1-results(c).CoP_target_values),results(c).pts_end,'o-','LineWidth',2,'DisplayName','Endurance');
    plot(100*(1-results(c).CoP_target_values),results(c).pts_accel,'o-','LineWidth',2,'DisplayName','Accel');
    xline(100*(1-results(c).best_CoP),'k--','Best CoP');
    xlabel('CoP (% wheelbase from front axle)'); ylabel('Points, this event only');
    title(sprintf('Which Event(s) Are Driving the CoP Result? -- %s',results(c).name),'Interpreter','none');
    legend('Location','best');
    grid on;
end

%% ========================================================================
% 8. SAVE
%% ========================================================================

assignin('base','cop_target_compare_results',results);
save('cop_target_compare_results.mat','results');

fprintf('\nSaved as: cop_target_compare_results (workspace), cop_target_compare_results.mat\n');
fprintf('\n===============================================================\n');
fprintf('                         COMPLETE\n');
fprintf('===============================================================\n');


%% ========================================================================
% LOCAL FUNCTIONS (identical physics engine to aero_spec_fast_model)
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

    Fy_demand_total = M*ay;
    Fy_demand = Fy_demand_total * (Fz/sum(Fz));

    Fx_rear = tire_longitudinal_capacity_combined( ...
        Fz(3:4),Fy_demand(3:4),Fz_grid,Fx_max_table,Fy_max_table);

    Fx_tire = sum(Fx_rear);

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
        Fx_demand = [0,0,Fdrag/2,Fdrag/2];

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


function Fx = tire_longitudinal_capacity(Fz,Fz_grid,Fx_table)
    Fx = zeros(size(Fz));
    for i = 1:numel(Fz)
        Fz_i = min(max(Fz(i),Fz_grid(1)),Fz_grid(end));
        Fx(i) = interp1(Fz_grid,Fx_table,Fz_i,'linear');
    end
end


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

%}