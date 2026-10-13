%% ========================================================================
%  MOCK ENDURANCE: CORNER-BY-CORNER AERO OPPORTUNITY ANALYSIS
%
%  CHANGES FROM PREVIOUS VERSION:
%    - CORNER DETECTION REWORKED: this track is continuously twisty --
%      lateral-G stays above threshold for ~80% of an entire lap, with
%      radius swinging between ~3m and ~450m sample to sample. The old
%      method (merge any stretch of high-lateral-G separated by less
%      than a 1.0s gap into one "corner") saw almost the whole lap as a
%      SINGLE corner, since there's rarely a full-second lull between
%      turns on this layout. That gave 38 corners total across the
%      entire 36-lap session -- about 1 per lap, when the track map
%      clearly shows 6+ distinct turns per lap.
%
%      NEW METHOD: apex detection. Within any region already satisfying
%      the cornering mask (lateral-G + radius bounds), find local PEAKS
%      IN CURVATURE (= local minima in radius = apexes) using
%      islocalmax with a minimum time separation and minimum prominence
%      to reject GPS-noise-driven spurious peaks. Each apex becomes one
%      corner; its data window spans from the midpoint to its
%      neighboring apexes (or the start/end of the cornering region for
%      the first/last apex in a stretch). This correctly separates
%      chicane/esses-type sequences into individual corners even when
%      lateral-G never drops between them.
%
%      Validated against this session's real data: apex detection finds
%      roughly 14-18 corners/lap (~500-650 total across 36 laps),
%      consistent with the track map. TWO TUNABLE KNOBS control
%      sensitivity -- APEX_MIN_SEPARATION_S and APEX_MIN_PROMINENCE_FRAC
%      -- see section 3. If the printed corner count still looks wrong
%      for your track, these are the first things to adjust.
%
%    - CLA_base / CdA_base use the REAL lapsim baseline (2.80 / 1.35).
%    - Added a TIME-SAVED column per corner (not just km/h).
%    - Added self-consistent drag time-cost (CdA sensitivity) using
%      this session's own data, for Scenario A in
%      aero_direction_comparison.m.
%    - Results saved to a .mat file for aero_direction_comparison.m.
%
%  IMPORTANT: this is a first-order engineering model. It does NOT
%  claim to directly measure tire friction or CLA.
%
%% ========================================================================

clc;
close all;
clear;

%% ========================================================================
%  1. FILES / ALIGNMENT
%% ========================================================================

file_rb = ...
    'testing_data/racebox_data/RaceBox_Track_Sessionon_04-05-2026_05-46.csv';

file_can = ...
    'testing_data/rotortemp_fullendurance_05_04_2026.csv';

time_offset = 69.3;
max_rb_time = 938.0;

%% ========================================================================
%  2. VEHICLE / AERO ASSUMPTIONS
%% ========================================================================

rho = 1.225;

vehicle_mass_kg = 562 * 0.45359237;

CLA_base = 2.80;    %  baseline
CLA_plus10 = 1.10 * CLA_base;
CdA_base = 1.35;    %  baseline 

g = 9.80665;

%% ========================================================================
%  3. ANALYSIS SETTINGS
%% ========================================================================

speed_min = 20;
speed_max = 80;

ay_corner_min = 0.40;       % g
min_corner_radius = 80;     % m
max_corner_radius = 3;      % m

position_smooth_seconds = 0.35;
speed_percentile = 95;

% NEW: apex detection tuning. These replace min_corner_duration and
% min_gap_between_corners from the old time-gap method.
APEX_MIN_SEPARATION_S = 0.8;      % minimum time between two distinct apexes
APEX_MIN_PROMINENCE_FRAC = 0.20;  % min curvature peak prominence, as a
                                    % fraction of that segment's curvature
                                    % std-dev -- raise this if you're
                                    % getting spurious noise-driven apexes,
                                    % lower it if plausible corners are
                                    % being missed.
MIN_STRETCH_GAP_S = 0.3;          % gap (s) used only to split genuinely
                                    % separate cornering stretches (e.g.
                                    % across different laps' same corner);
                                    % NOT used to define individual corners
                                    % anymore -- apexes handle that.

%% ========================================================================
%  4. LOAD DATA
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('          MOCK ENDURANCE CORNER-BY-CORNER ANALYSIS\n');
fprintf('===============================================================\n');

fprintf('\nUsing baseline: CLA=%.2f m^2, CdA=%.2f m^2\n',CLA_base,CdA_base);

opts = detectImportOptions(file_rb);
rb = readtable(file_rb,opts);
can = readtable(file_can);

fprintf('\nRaceBox samples = %d\n',height(rb));
fprintf('CANary samples  = %d\n',height(can));

if any(strcmp(rb.Properties.VariableNames,'Lap'))
    fprintf('RaceBox laps in file = %d (0 to %d)\n',numel(unique(rb.Lap)),max(rb.Lap));
end

%% ========================================================================
%  5. RACEBOX SIGNALS
%% ========================================================================

t_rb = sanitize(rb.Time);
speed_rb_ms = sanitize(rb.Speed);
ax_rb = sanitize(rb.GForceX);
ay_rb = abs(sanitize(rb.GForceY));
lat = sanitize(rb.Latitude);
lon = sanitize(rb.Longitude);

%% ========================================================================
%  6. CANARY
%% ========================================================================

t_can = sanitize(can.Time);
tps = sanitize(can.ThrottlePosition);

%% ========================================================================
%  7. ALIGN CANARY TO RACEBOX
%% ========================================================================

t_can_aligned = t_can - time_offset;
valid_can = t_can_aligned >= 0 & t_can_aligned <= max_rb_time;
t_can_aligned = t_can_aligned(valid_can);
tps = tps(valid_can);

%% ========================================================================
%  8. INTERPOLATE RACEBOX SIGNALS TO CANARY TIME
%% ========================================================================

v_gps_ms = interp1(t_rb,speed_rb_ms,t_can_aligned,'linear','extrap');
v_gps_kmh = v_gps_ms * 3.6;
ax_g = interp1(t_rb,ax_rb,t_can_aligned,'linear','extrap');
ay_g = interp1(t_rb,ay_rb,t_can_aligned,'linear','extrap');
lat_can = interp1(t_rb,lat,t_can_aligned,'linear','extrap');
lon_can = interp1(t_rb,lon,t_can_aligned,'linear','extrap');

%% ========================================================================
%  9. TIME STEP
%% ========================================================================

dt = median(diff(t_can_aligned),'omitnan');

%% ========================================================================
%  10. CONVERT GPS TO LOCAL X/Y
%% ========================================================================

R_earth = 6371000;
lat0 = median(lat_can(isfinite(lat_can)));
lon0 = median(lon_can(isfinite(lon_can)));
x = R_earth * cosd(lat0) .* deg2rad(lon_can-lon0);
y = R_earth .* deg2rad(lat_can-lat0);

%% ========================================================================
%  11. SMOOTH GPS POSITION
%% ========================================================================

smooth_samples = max(5,round(position_smooth_seconds/dt));
x_s = movmean(x,smooth_samples,'omitnan');
y_s = movmean(y,smooth_samples,'omitnan');

%% ========================================================================
%  12. TRACK DERIVATIVES / CURVATURE
%% ========================================================================

dx = gradient(x_s,dt);
dy = gradient(y_s,dt);
ddx = gradient(dx,dt);
ddy = gradient(dy,dt);

track_speed_ms = sqrt(dx.^2+dy.^2);

curvature = abs(dx.*ddy-dy.*ddx) ./ max((dx.^2+dy.^2).^(3/2),1e-8);
radius_m = 1 ./ max(curvature,1e-8);
radius_m(track_speed_ms < 5/3.6) = NaN;

%% ========================================================================
%  13. BASIC VALIDITY + CORNERING MASK
%% ========================================================================

base_mask = ...
    isfinite(v_gps_kmh) & isfinite(ax_g) & isfinite(ay_g) & isfinite(radius_m) & ...
    v_gps_kmh >= speed_min & v_gps_kmh <= speed_max;

broad_mask = ...
    base_mask & ay_g >= ay_corner_min & ...
    radius_m <= min_corner_radius & radius_m >= max_corner_radius;

%% ========================================================================
%  14. NEW: APEX-BASED CORNER DETECTION
%
%  Split the broad cornering mask into contiguous stretches (using a
%  short gap tolerance just to separate genuinely disjoint episodes),
%  then find curvature peaks (apexes) within each stretch. Each apex is
%  one corner; its data window runs from the midpoint to its neighbor
%  apexes (or stretch start/end for the first/last apex).
%% ========================================================================

idx = find(broad_mask);

if isempty(idx)
    error('No cornering regions detected.');
end

gap_tol_samples = round(MIN_STRETCH_GAP_S/dt);
stretch_breaks = find(diff(idx) > gap_tol_samples);
stretch_start_pos = [1; stretch_breaks+1];
stretch_end_pos = [stretch_breaks; length(idx)];

min_sep_samples = max(1,round(APEX_MIN_SEPARATION_S/dt));

apex_global_idx = [];   % index into the full time series for each apex

for si = 1:length(stretch_start_pos)

    s = stretch_start_pos(si);
    e = stretch_end_pos(si);
    seg_idx = idx(s:e);

    if numel(seg_idx) < 3
        continue;
    end

    seg_curv = curvature(seg_idx);
    prom_thresh = APEX_MIN_PROMINENCE_FRAC * std(seg_curv,'omitnan');

    if ~isfinite(prom_thresh) || prom_thresh <= 0
        continue;
    end

    tf = islocalmax(seg_curv,'MinSeparation',min_sep_samples,'MinProminence',prom_thresh);

    apex_global_idx = [apex_global_idx; seg_idx(tf)]; %#ok<AGROW>

end

apex_global_idx = sort(apex_global_idx);

fprintf('\nApex-based corner detection:\n');
fprintf('  Cornering stretches found = %d\n',numel(stretch_start_pos));
fprintf('  Apexes (= corners) found  = %d\n',numel(apex_global_idx));

%% ========================================================================
%  15. BUILD CORNER WINDOWS AROUND EACH APEX
%
%  Each corner's window spans from the midpoint to the previous apex
%  (or its stretch's start, if it's the first apex in that stretch) to
%  the midpoint to the next apex (or its stretch's end).
%% ========================================================================

corners = struct( ...
    'id',{}, ...
    'apex_idx',{}, ...
    'start_idx',{}, ...
    'end_idx',{}, ...
    'duration',{}, ...
    'speed95',{}, ...
    'speed_max',{}, ...
    'ay95',{}, ...
    'radius_median',{}, ...
    'radius10',{}, ...
    'radius90',{}, ...
    'distance_m',{}, ...
    'DF95_N',{}, ...
    'CLA_speed_gain_kmh',{}, ...
    'time_saved_s',{}, ...
    'valid',{});

corner_count = 0;

for si = 1:length(stretch_start_pos)

    s = stretch_start_pos(si);
    e = stretch_end_pos(si);
    seg_idx = idx(s:e);

    apex_in_seg = apex_global_idx(apex_global_idx >= seg_idx(1) & apex_global_idx <= seg_idx(end));

    if isempty(apex_in_seg)
        continue;
    end

    for k = 1:numel(apex_in_seg)

        if k == 1
            i1 = seg_idx(1);
        else
            i1 = round((apex_in_seg(k-1)+apex_in_seg(k))/2);
        end

        if k == numel(apex_in_seg)
            i2 = seg_idx(end);
        else
            i2 = round((apex_in_seg(k)+apex_in_seg(k+1))/2);
        end

        duration = t_can_aligned(i2)-t_can_aligned(i1);

        V = v_gps_kmh(i1:i2);
        AY = ay_g(i1:i2);
        R = radius_m(i1:i2);

        valid = isfinite(V) & isfinite(AY) & isfinite(R);
        V = V(valid); AY = AY(valid); R = R(valid);

        if numel(V) < 3
            continue;
        end

        corner_count = corner_count+1;

        corners(corner_count).id = corner_count;
        corners(corner_count).apex_idx = apex_in_seg(k);
        corners(corner_count).start_idx = i1;
        corners(corner_count).end_idx = i2;
        corners(corner_count).duration = duration;
        corners(corner_count).speed95 = prctile(V,speed_percentile);
        corners(corner_count).speed_max = max(V);
        corners(corner_count).ay95 = prctile(AY,95);
        corners(corner_count).radius_median = median(R,'omitnan');
        corners(corner_count).radius10 = prctile(R,10);
        corners(corner_count).radius90 = prctile(R,90);
        corners(corner_count).distance_m = trapz(t_can_aligned(i1:i2),v_gps_ms(i1:i2));

        Vrep = corners(corner_count).speed95/3.6;
        q = 0.5*rho*Vrep^2;
        corners(corner_count).DF95_N = CLA_base*q;

        ay_obs = corners(corner_count).ay95 * g;
        W_N = vehicle_mass_kg*g;
        DF_current = CLA_base*q;
        DF_plus10 = CLA_plus10*q;

        mu_effective = ay_obs*vehicle_mass_kg / max(W_N+DF_current,1);
        ay_plus10 = mu_effective * (W_N+DF_plus10) / vehicle_mass_kg;

        Rrep = corners(corner_count).radius_median;
        V_possible_current = sqrt(max(ay_obs*Rrep,0));
        V_possible_plus10 = sqrt(max(ay_plus10*Rrep,0));

        corners(corner_count).CLA_speed_gain_kmh = (V_possible_plus10-V_possible_current)*3.6;

        if V_possible_current > 0 && V_possible_plus10 > 0 && corners(corner_count).distance_m > 0
            t_old = corners(corner_count).distance_m / V_possible_current;
            t_new = corners(corner_count).distance_m / V_possible_plus10;
            corners(corner_count).time_saved_s = t_old - t_new;
        else
            corners(corner_count).time_saved_s = NaN;
        end

        corners(corner_count).valid = true;

    end
end

fprintf('  Final corner count (after window building) = %d\n',corner_count);

%% ========================================================================
%  16. PRINT CORNER TABLE (first/last 20 -- full table can be very long)
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                  DETECTED CORNERS (apex-based)\n');
fprintf('===============================================================\n');

fprintf('\n');
fprintf(['ID   Duration   Rmed     R10-R90      V95      Ay95   ' ...
         'DF@V95    +10%%CLA dV   time saved\n']);
fprintf(['     (s)        (m)        (m)       (km/h)     g       ' ...
         '(N)       (km/h)        (s)\n']);
fprintf('----------------------------------------------------------------------------------------\n');

print_ids = unique([1:min(20,corner_count), max(1,corner_count-19):corner_count]);
for i = print_ids
    fprintf( ...
        '%3d   %6.2f    %6.1f   %5.1f-%5.1f   %7.1f   %5.2f   %7.1f    %+6.2f      %+7.3f\n', ...
        corners(i).id,corners(i).duration,corners(i).radius_median, ...
        corners(i).radius10,corners(i).radius90,corners(i).speed95, ...
        corners(i).ay95,corners(i).DF95_N,corners(i).CLA_speed_gain_kmh, ...
        corners(i).time_saved_s);
end
if corner_count > 40
    fprintf('  ... (%d corners total, showing first/last 20) ...\n',corner_count);
end

%% ========================================================================
%  17. HIGH-SPEED CORNER SUBSET
%% ========================================================================

speed_thresholds = [40 50 60];

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              HIGH-SPEED CORNER SUMMARY\n');
fprintf('===============================================================\n');

for Vth = speed_thresholds
    selected = [corners.speed95] >= Vth;
    if any(selected)
        dV = [corners(selected).CLA_speed_gain_kmh];
        dT = [corners(selected).time_saved_s];
        DF = [corners(selected).DF95_N];
        R = [corners(selected).radius_median];
        fprintf('\nCorners with V95 >= %d km/h:\n',Vth);
        fprintf('  Number of corners = %d\n',sum(selected));
        fprintf('  Median radius     = %.1f m\n',median(R,'omitnan'));
        fprintf('  Median DF         = %.1f N\n',median(DF,'omitnan'));
        fprintf('  Median +10%% CLA speed benefit = %.2f km/h\n',median(dV,'omitnan'));
        fprintf('  Total time saved (+10%% CLA)   = %.3f s\n',sum(dT,'omitnan'));
    end
end

%% ========================================================================
%  18. DRAG EXPOSURE
%% ========================================================================

wot_mask = base_mask & tps >= 85;
high_speed_wot = wot_mask & v_gps_kmh >= 60;
high_speed_70_wot = wot_mask & v_gps_kmh >= 70;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    DRAG EXPOSURE\n');
fprintf('===============================================================\n');

total_time = sum(base_mask)*dt;

fprintf('\nTotal valid time = %.1f s\n',total_time);
fprintf('WOT time = %.1f s (%.1f%%)\n',sum(wot_mask)*dt,100*sum(wot_mask)*dt/total_time);
fprintf('WOT >60 km/h = %.1f s (%.1f%%)\n',sum(high_speed_wot)*dt,100*sum(high_speed_wot)*dt/total_time);
fprintf('WOT >70 km/h = %.1f s (%.1f%%)\n',sum(high_speed_70_wot)*dt,100*sum(high_speed_70_wot)*dt/total_time);

V = v_gps_ms;
q = 0.5*rho*V.^2;
Fdrag = CdA_base*q;
Pdrag = Fdrag.*V;

drag_energy_kJ = sum(Pdrag(wot_mask)*dt)/1000;
fprintf('\nEstimated drag energy during WOT = %.2f kJ\n',drag_energy_kJ);

%% ========================================================================
%  19. SELF-CONSISTENT DRAG TIME-COST (this session's own data)
%
%  Uses GForceX (RaceBox) for acceleration, since this session has no
%  CANary ACCELX. Mirrors the same calculation in
%  comp_endurance_analysis.m so Scenario A in aero_direction_comparison
%  compares downforce and drag both measured on THIS track.
%% ========================================================================

ax_ms2 = ax_g * g;

F_engine_est = vehicle_mass_kg*ax_ms2 + Fdrag;
P_engine_est = F_engine_est .* V;

valid_power = wot_mask & isfinite(P_engine_est) & P_engine_est > 0 & ...
    P_engine_est < 60000 & v_gps_kmh > 15;

avg_wot_power_W = mean(P_engine_est(valid_power),'omitnan');
E_drag_total_kJ = sum(Pdrag(base_mask)*dt)/1000;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('     SELF-CONSISTENT DRAG TIME-COST (this session''s own data)\n');
fprintf('===============================================================\n');
fprintf('\nEmpirical average WOT power = %.2f kW (n=%d samples)\n', ...
    avg_wot_power_W/1000,nnz(valid_power));
fprintf('Total drag energy, full valid session (CdA=%.3f) = %.2f kJ\n', ...
    CdA_base,E_drag_total_kJ);

CdA_cases = [CdA_base, 0.95*CdA_base, 0.90*CdA_base, 0.85*CdA_base, 0.80*CdA_base];
time_saved_cases = NaN(size(CdA_cases));

fprintf('\nCdA          Reduction    Energy saved     TIME SAVED\n');
fprintf(' (m^2)                       (kJ, full sess)   (s, this session)\n');
fprintf('-------------------------------------------------------------------\n');

for i = 1:length(CdA_cases)
    cda = CdA_cases(i);
    reduction = 100*(1-cda/CdA_base);
    Fdrag_i = cda*q;
    Pdrag_i = Fdrag_i.*V;
    E_i_kJ = sum(Pdrag_i(base_mask)*dt)/1000;
    dE_kJ = E_drag_total_kJ - E_i_kJ;
    time_saved_cases(i) = (dE_kJ*1000)/avg_wot_power_W;
    fprintf('%7.3f      %6.1f %%       %9.2f          %+8.3f\n', ...
        cda,reduction,dE_kJ,time_saved_cases(i));
end

%% ========================================================================
%  20. TOTAL CORNER OPPORTUNITY
%% ========================================================================

corner_dV = [corners.CLA_speed_gain_kmh];
corner_dT = [corners.time_saved_s];
corner_speed = [corners.speed95];
corner_radius = [corners.radius_median];
corner_DF = [corners.DF95_N];

total_time_saved_plus10 = sum(corner_dT,'omitnan');

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                 AERO OPPORTUNITY SUMMARY\n');
fprintf('===============================================================\n');

fprintf('\nAll detected corners:\n');
if any(strcmp(rb.Properties.VariableNames,'Lap'))
    n_laps_in_window = numel(unique(rb.Lap(rb.Time<=max_rb_time)));
    fprintf('  Number of corners = %d (across %d laps -- ~%.1f corners/lap)\n', ...
        corner_count,n_laps_in_window,corner_count/max(n_laps_in_window,1));
else
    fprintf('  Number of corners = %d\n',corner_count);
end
fprintf('  Median V95 = %.1f km/h\n',median(corner_speed));
fprintf('  Median radius = %.1f m\n',median(corner_radius));
fprintf('  Median downforce at V95 = %.1f N\n',median(corner_DF));
fprintf('  Median estimated +10%%CLA speed benefit = %.2f km/h\n',median(corner_dV));
fprintf('  Maximum estimated +10%%CLA speed benefit = %.2f km/h\n',max(corner_dV));
fprintf('\n  TOTAL SESSION TIME SAVED (+10%% CLA) = %.3f s\n',total_time_saved_plus10);
fprintf('  (over this session''s valid duration of %.1f s)\n',total_time);

%% ========================================================================
%  21-25. PLOTS
%% ========================================================================

figure('Name','Mock Endurance Track Map','Position',[100 100 1000 750]);
plot(x_s,y_s,'LineWidth',1.0);
hold on;
% only mark a subset of corners on the map to keep it readable
mark_ids = round(linspace(1,corner_count,min(corner_count,40)));
for i = mark_ids
    i1 = corners(i).start_idx; i2 = corners(i).end_idx;
    plot(x_s(i1:i2),y_s(i1:i2),'LineWidth',2);
end
axis equal;
xlabel('Track X (m)'); ylabel('Track Y (m)');
title(sprintf('Detected Corner Regions (%d total, %d shown)',corner_count,numel(mark_ids)));
grid on;

figure('Name','Corner Speed vs Radius','Position',[150 150 1000 650]);
scatter(corner_radius,corner_speed,40,corner_dV,'filled');
xlabel('Median corner radius (m)'); ylabel('95th-percentile corner speed (km/h)');
title('Corner Speed vs Radius (apex-based)');
cb = colorbar; cb.Label.String = '+10% CLA predicted speed benefit (km/h)';
grid on;

figure('Name','Estimated Time Benefit of Additional Downforce','Position',[200 200 1000 650]);
bar(1:corner_count,corner_dT);
xlabel('Detected corner (apex #)'); ylabel('Estimated time saved from +10% CLA (s)');
title('First-Order Corner TIME Benefit of +10% CLA');
grid on;

figure('Name','Speed vs Lateral G','Position',[250 250 1000 650]);
scatter(v_gps_kmh(base_mask),ay_g(base_mask),8,'filled');
xlabel('GPS speed (km/h)'); ylabel('Lateral acceleration (g)');
title('Mock Endurance Speed vs Lateral G');
grid on;

%% ========================================================================
%  26. SAVE RESULTS
%% ========================================================================

corner_analysis = struct();
corner_analysis.corners = corners;
corner_analysis.CLA_base = CLA_base;
corner_analysis.CLA_plus10 = CLA_plus10;
corner_analysis.CdA_base = CdA_base;
corner_analysis.total_time = total_time;
corner_analysis.drag_energy_kJ = drag_energy_kJ;
corner_analysis.corner_speed = corner_speed;
corner_analysis.corner_radius = corner_radius;
corner_analysis.corner_dV = corner_dV;
corner_analysis.corner_dT = corner_dT;
corner_analysis.total_time_saved_plus10 = total_time_saved_plus10;
corner_analysis.session_duration_s = total_time;
corner_analysis.avg_wot_power_W = avg_wot_power_W;
corner_analysis.E_drag_total_kJ = E_drag_total_kJ;
corner_analysis.CdA_cases = CdA_cases;
corner_analysis.time_saved_cases = time_saved_cases;
corner_analysis.corner_count = corner_count;

assignin('base','corner_analysis',corner_analysis);
save('mock_endurance_analysis.mat','corner_analysis');

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                     COMPLETE\n');
fprintf('===============================================================\n');
fprintf('\nResults saved to:\n');
fprintf('  corner_analysis (workspace)\n');
fprintf('  mock_endurance_analysis.mat\n');
fprintf('\nThis is intentionally a first-order opportunity analysis rather\n');
fprintf('than a claim of exact lap-time improvement. If the corner count\n');
fprintf('still looks wrong for your track, tune APEX_MIN_SEPARATION_S and\n');
fprintf('APEX_MIN_PROMINENCE_FRAC in section 3.\n');
fprintf('\n===============================================================\n');


%% ========================================================================
%  LOCAL FUNCTION
%% ========================================================================

function x = sanitize(x)
    x = double(x(:));
    x(isinf(x)) = NaN;
    x = fillmissing(x,'linear','EndValues','nearest');
end