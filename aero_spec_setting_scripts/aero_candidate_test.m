%% ========================================================================
% FSAE AERO CANDIDATE TESTER (fast, standalone)
%
% PURPOSE: quickly test a handful of specific CLA/CdA/CoP candidates --
% e.g. real CFD numbers from aero -- WITHOUT running the full target-
% setting sweep (aero_spec_fast_model). That script's CLA x CdA grid,
% break-even boundary, and mass sweep are the expensive parts; this
% script skips all of that and just evaluates the candidates you give
% it directly. Same physics engine, verified identical to the target-
% setting script -- just no big sweep loop.
%
% TWO MODES (set CANDIDATE_MODE below):
%   'literal'      -- use each candidate's CLA/CdA exactly as given.
%                     Reasonable if the numbers you have are already
%                     whole-car totals on a scale comparable to
%                     BASE_CLA/BASE_CDA below (e.g. consistent CFD
%                     methodology, not an isolated-component study).
%   'delta_mapped' -- treat candidate #1 as the reference point, apply
%                     every other candidate's DELTA from it on top of
%                     BASE_CLA/BASE_CDA instead. Safer if you don't
%                     trust the candidates' absolute scale matches the
%                     lapsim's baseline scale, at the cost of assuming
%                     candidate #1 IS equivalent to the current baseline.
%
% Neither mode is more "correct" in general -- it depends on where your
% numbers came from. Run both if you're not sure; if they agree, the
% distinction didn't matter much for this data.
%
%% ========================================================================

clc;
clear;
close all;

%% ========================================================================
% 1. USER INPUT -- EDIT THIS SECTION
%% ========================================================================

CANDIDATE_MODE = 'literal';   % 'literal' or 'delta_mapped'

% Each row: name, CLA (m^2), CdA (m^2), CoP_front (0-1, force fraction --
% (fraction of DF in the front) -- NOT aero's wheelbase-position convention; convert first if needed).
% Use NaN for CoP_front to hold it at BASE_COP_FRONT (defined below).
%
% In 'delta_mapped' mode, candidate #1 is the reference -- every other
% row's CLA/CdA is treated as an absolute value to compute a DELTA from
% row 1, which then gets added onto BASE_CLA/BASE_CDA. Row 1 itself
% always maps exactly onto the baseline in that mode.

%{
candidates = struct('name',{},'CLA',{},'CdA',{},'CoP_front',{});
%}

%candidates(1) = struct('name','v1 (B26 RW, new mesh)',              'CLA',3.59,'CdA',1.36,'CoP_front',1-0.48);
%candidates(2) = struct('name','v2 (v1 + outboard MP twist +3deg)',  'CLA',3.72,'CdA',1.44,'CoP_front',1-0.51);
%candidates(3) = struct('name', 'Orig EPs + no Gurney',              'CLA',2.92,'CdA',1.10,'CoP_front',NaN);
candidates(1) = struct('name','B26',                                'CLA',3.55,'CdA',1.40,'CoP_front',1-0.4896);
candidates(2) = struct('name','v9 (no louvres)',                    'CLA',3.7,'CdA',1.45,'CoP_front',1-0.502);




% ADD MORE CANDIDATES HERE, e.g.:
% candidates(3) = struct('name','My New Config','CLA',3.65,'CdA',1.40,'CoP_front',NaN);

%% ========================================================================
% 2. BASELINE AERO + MASS (must match aero_spec_fast_model's values)
%% ========================================================================

BASE_CLA = 2.8;
BASE_CDA = 1.35;

BASE_COP_FRONT = 0.53;   % corrected force fraction (BASE_COP_FRONT = 0.53 --> 53% distributed in the front, 47% from front axle)
BASE_COP_REAR  = 1 - BASE_COP_FRONT;

BASE_CAR_MASS      = 162.8;
BASE_DRIVER_MASS   = 64;

ENABLE_CALIBRATION = false;
CAL_AUTOX = 1; CAL_ENDURANCE = 1; CAL_SKIDPAD = 1;

SKIDPAD_WIN_TIME   = 4.878;
AUTOX_WIN_TIME     = 46.911;
ENDURANCE_WIN_TIME = 1395.891;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              FSAE AERO CANDIDATE TESTER (fast)\n');
fprintf('===============================================================\n');
fprintf('\nMode: %s\n',CANDIDATE_MODE);
fprintf('Baseline: CLA=%.2f, CdA=%.2f, CoP_front=%.2f (force fraction)\n', ...
    BASE_CLA,BASE_CDA,BASE_COP_FRONT);

%% ========================================================================
% 3. VEHICLE PARAMETERS
%% ========================================================================

rho = 1.225;
g = 9.81;

M = BASE_CAR_MASS + BASE_DRIVER_MASS;

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

N_AUTO = 1500;
N_END  = 1500;

%% ========================================================================
% 4. TIRE MODEL, POWERTRAIN, TRACKS, TIRE LOOKUP TABLES
% (setup only -- fast, no big sweep)
%% ========================================================================

fprintf('\nLoading tire model...\n');

load('Fx_combined_parameters_run38_30.mat');
Fx_parameters = cell2mat(Xbestcell);

load('Lapsim_Fy_combined_parameters_1965run15.mat');
Fy_parameters = cell2mat(Xbestcell);

tire_pressure = 12;
friction_scaling_factor = 1.05*0.55;

tire = Tire2(tire_pressure,Fx_parameters,Fy_parameters,friction_scaling_factor);

fprintf('Building powertrain model (not used for baseline package, kept for interface compatibility)...\n');

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
% 5. BASELINE EVALUATION (for comparison)
%% ========================================================================

t_start = tic;

baseline = evaluate_package( ...
    BASE_CLA,BASE_CDA,BASE_COP_FRONT,M, ...
    rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
    powertrain,wheel_radius, ...
    Fz_grid,Fy_max_table,Fx_max_table, ...
    auto_s,auto_k,end_s,end_k, ...
    N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

baseline_points_autox   = fsae_points_linear(baseline.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5);
baseline_points_skidpad = fsae_points_skidpad(baseline.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME);
baseline_points_enduro  = fsae_points_linear(baseline.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);
baseline_3event_points  = baseline_points_autox+baseline_points_skidpad+baseline_points_enduro;

fprintf('\nBaseline (raw physics):\n');
fprintf('  Autocross      = %.3f s (points %.1f)\n',baseline.auto_time,baseline_points_autox);
fprintf('  Skidpad        = %.3f s (points %.1f)\n',baseline.skidpad_time,baseline_points_skidpad);
fprintf('  Endurance/lap  = %.3f s (points %.1f)\n',baseline.endurance_lap_time,baseline_points_enduro);
fprintf('  3-EVENT POINTS = %.1f\n',baseline_3event_points);
fprintf('  Setup+baseline runtime = %.1f s\n',toc(t_start));

%% ========================================================================
% 6. RESOLVE CANDIDATES (literal or delta-mapped) AND EVALUATE
%% ========================================================================

nCandidates = numel(candidates);

resolved_CLA = NaN(1,nCandidates);
resolved_CdA = NaN(1,nCandidates);
resolved_CoP = NaN(1,nCandidates);

if strcmpi(CANDIDATE_MODE,'delta_mapped')
    CLA_ref = candidates(1).CLA;
    CdA_ref = candidates(1).CdA;
    for c = 1:nCandidates
        resolved_CLA(c) = BASE_CLA + (candidates(c).CLA - CLA_ref);
        resolved_CdA(c) = BASE_CDA + (candidates(c).CdA - CdA_ref);
    end
else % literal
    for c = 1:nCandidates
        resolved_CLA(c) = candidates(c).CLA;
        resolved_CdA(c) = candidates(c).CdA;
    end
end

for c = 1:nCandidates
    if isnan(candidates(c).CoP_front)
        resolved_CoP(c) = BASE_COP_FRONT;
    else
        resolved_CoP(c) = candidates(c).CoP_front;
    end
end

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    CANDIDATE RESULTS\n');
fprintf('===============================================================\n');
if strcmpi(CANDIDATE_MODE,'delta_mapped')
    fprintf('(delta-mapped onto baseline; candidate #1 = reference)\n\n');
else
    fprintf('(literal values, used exactly as given)\n\n');
end

results = struct();
results.names = {candidates.name};
results.CLA_input = [candidates.CLA];
results.CdA_input = [candidates.CdA];
results.CLA_resolved = resolved_CLA;
results.CdA_resolved = resolved_CdA;
results.CoP_resolved = resolved_CoP;
results.auto_time = NaN(1,nCandidates);
results.skidpad_time = NaN(1,nCandidates);
results.endurance_time = NaN(1,nCandidates);
results.points_total = NaN(1,nCandidates);

for c = 1:nCandidates

    r = evaluate_package( ...
        resolved_CLA(c),resolved_CdA(c),resolved_CoP(c),M, ...
        rho,g,Wb,track,lf,lr,hg,hrf,hrr,hrc,Rsf, ...
        powertrain,wheel_radius, ...
        Fz_grid,Fy_max_table,Fx_max_table, ...
        auto_s,auto_k,end_s,end_k, ...
        N_ITER,V_CORNER,CURVATURE_CUTOFF,TIRE_UTILIZATION);

    results.auto_time(c) = r.auto_time;
    results.skidpad_time(c) = r.skidpad_time;
    results.endurance_time(c) = r.endurance_lap_time;

    pa = fsae_points_linear(r.auto_time*CAL_AUTOX,AUTOX_WIN_TIME,1.45,118.5,6.5);
    ps = fsae_points_skidpad(r.skidpad_time*CAL_SKIDPAD,SKIDPAD_WIN_TIME);
    pe = fsae_points_linear(r.endurance_10lap_time*CAL_ENDURANCE,ENDURANCE_WIN_TIME,1.45,250,25);
    results.points_total(c) = pa+ps+pe;

    %%% CoP in aero-convention
    fprintf('%-32s CLA=%.2f CdA=%.2f CoP=%.4f | auto=%.3fs skid=%.3fs end=%.3fs | points=%.1f (%+.1f vs baseline)\n', ...
        candidates(c).name,resolved_CLA(c),resolved_CdA(c),1-resolved_CoP(c), ...
        r.auto_time,r.skidpad_time,r.endurance_lap_time,results.points_total(c), ...
        results.points_total(c)-baseline_3event_points);

end

[~,best_idx] = max(results.points_total);
fprintf('\nBest candidate: %s (points=%.1f, %+.1f vs baseline)\n', ...
    candidates(best_idx).name,results.points_total(best_idx),results.points_total(best_idx)-baseline_3event_points);

%% ========================================================================
% 7. PLOT
%% ========================================================================

figure('Name','Candidate Comparison','Position',[200 200 900 600]);
bar(results.points_total - baseline_3event_points);
hold on;
yline(0,'k--','Baseline');
set(gca,'XTickLabel',results.names,'XTickLabelRotation',20,'TickLabelInterpreter','none');
ylabel('3-event points vs. baseline');
title(sprintf('Candidate Comparison (%s mode)',CANDIDATE_MODE));
grid on;

%% ========================================================================
% 8. SAVE
%% ========================================================================

assignin('base','candidate_test_results',results);
save('aero_candidate_test_results.mat','results');

fprintf('\nSaved as: candidate_test_results (workspace), aero_candidate_test_results.mat\n');
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


function idx = find_gear(V,switch_gear_velocities)
    idx = find(V < switch_gear_velocities,1);
    if isempty(idx)
        idx = numel(switch_gear_velocities);
    end
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