%% ========================================================================
%  FSAE COMPETITION DATA - SPEED / LONGITUDINAL / LATERAL AERO ANALYSIS
%
%  CHANGES FROM PREVIOUS VERSION:
%    - Added an EMPIRICAL average-WOT-power estimate, derived directly
%      from this session's wheel-speed acceleration + drag force (no
%      dyno number assumed) -- used to convert the existing CdA
%      sensitivity table from drag FORCE/POWER into actual TIME COST
%      over the full session, which is what actually gets compared
%      against the corner-side time benefit from the mock endurance
%      script.
%    - Results now also saved to a .mat file so aero_direction_
%      comparison.m can load this run's output independently.
%
%  PURPOSE
%  -------
%  Analyze competition CANary logs where:
%    - Front-right wheel speed is available
%    - Front-left wheel speed is NOT used
%    - Throttle is available
%    - ACCELX / ACCELY are available and cleaned
%    - ACCELZ is intentionally NOT used
%    - NO shockpots are available
%
%  It does NOT attempt to calculate absolute downforce, CoP, or actual
%  tire utilization from this dataset.
%
%% ========================================================================

clc;
close all;
clear;

%% ========================================================================
%  1. USER FILE
%% ========================================================================

file_comp = ...
    'testing_data/comp_202_2026-07-15T12-37-00.csv';

max_analysis_time = 2000.0;

%% ========================================================================
%  2. VEHICLE / AERO PARAMETERS
%% ========================================================================

rho = 1.225;

mass_kg = 530 * 0.45359237;

% Baseline values
CLA_base = 2.80;
CdA_base = 1.35;

%% ========================================================================
%  3. SPEED / THROTTLE SETTINGS
%% ========================================================================

speed_min = 5;
speed_max = 100;

wheel_speed_max = 100;

WOT_threshold = 85;

speed_thresholds = [40 50 60 70 75 80];

accel_filter_fc = 1.0;

imu_spike_limit = 3.0;
imu_median_window = 5;
imu_lowpass_fc = 1.0;

%% ========================================================================
%  4. LOAD DATA
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('        FSAE COMPETITION SPEED + IMU ANALYSIS\n');
fprintf('===============================================================\n');

fprintf('\nBaseline: CLA=%.2f m^2, CdA=%.2f m^2\n',CLA_base,CdA_base);
fprintf('\nLoading:\n');
fprintf('  %s\n',file_comp);

df = readtable(file_comp);

fprintf('Samples = %d\n',height(df));

%% ========================================================================
%  5. EXTRACT CANARY CHANNELS
%% ========================================================================

t = sanitize(df.Time);

v_fr = sanitize(df.WheelSpeedFrontRight);

tps = sanitize(df.ThrottlePosition);

accelx_available = false;
accely_available = false;

if any(strcmp(df.Properties.VariableNames,'ACCELX'))
    accelx_raw = sanitize(df.ACCELX);
    accelx_available = true;
    fprintf('\nIMU channel used:\n  ACCELX\n');
else
    accelx_raw = NaN(height(df),1);
    fprintf('\nWARNING: ACCELX not found.\n');
end

if any(strcmp(df.Properties.VariableNames,'ACCELY'))
    accely_raw = sanitize(df.ACCELY);
    accely_available = true;
    fprintf('  ACCELY\n');
else
    accely_raw = NaN(height(df),1);
    fprintf('WARNING: ACCELY not found.\n');
end

session_mask = t >= 0 & t <= max_analysis_time;

t = t(session_mask);
v_fr = v_fr(session_mask);
tps = tps(session_mask);
accelx_raw = accelx_raw(session_mask);
accely_raw = accely_raw(session_mask);

%% ========================================================================
%  6. WHEEL SPEED (Front Right only)
%% ========================================================================

v_wheel_kmh = v_fr;

v_wheel_kmh(v_wheel_kmh < 0 | v_wheel_kmh > wheel_speed_max) = NaN;
v_wheel_kmh = fillmissing(v_wheel_kmh,'linear','EndValues','nearest');

v_wheel_ms = v_wheel_kmh/3.6;

%% ========================================================================
%  7. TIME STEP
%% ========================================================================

dt = median(diff(t),'omitnan');
if ~isfinite(dt) || dt <= 0
    error('Could not determine valid CAN sample time.');
end
fs = 1/dt;

fprintf('\nSampling:\n  dt = %.6f s\n  fs = %.2f Hz\n',dt,fs);

%% ========================================================================
%  8. BASIC VALID MASK
%% ========================================================================

valid = ...
    isfinite(t) & isfinite(v_wheel_kmh) & isfinite(tps) & ...
    v_wheel_kmh >= speed_min & v_wheel_kmh <= speed_max;

fprintf('\nValid speed points = %d\n',nnz(valid));

%% ========================================================================
%  9. SMOOTH SPEED FOR ACCELERATION
%% ========================================================================

[b_speed,a_speed] = butter(2,accel_filter_fc/(fs/2),'low');
v_smooth_ms = filtfilt(b_speed,a_speed,double(v_wheel_ms));
v_smooth_kmh = v_smooth_ms*3.6;

%% ========================================================================
%  10. LONGITUDINAL ACCELERATION FROM WHEEL SPEED
%% ========================================================================

ax_est_ms2 = gradient(v_smooth_ms,dt);
ax_est_g = ax_est_ms2/9.80665;

%% ========================================================================
%  11. CLEAN ACCELX / ACCELY (unchanged)
%% ========================================================================

if accelx_available
    accelx_clean = accelx_raw;
    accelx_clean(abs(accelx_clean) > imu_spike_limit) = NaN;
    accelx_clean = fillmissing(accelx_clean,'linear','EndValues','nearest');
    accelx_clean = medfilt1(accelx_clean,imu_median_window);
    [b_imu,a_imu] = butter(2,imu_lowpass_fc/(fs/2),'low');
    accelx_clean = filtfilt(b_imu,a_imu,double(accelx_clean));
    accelx_clean = -accelx_clean;
else
    accelx_clean = NaN(size(t));
end

if accely_available
    accely_clean = accely_raw;
    accely_clean(abs(accely_clean) > imu_spike_limit) = NaN;
    accely_clean = fillmissing(accely_clean,'linear','EndValues','nearest');
    if ~exist('b_imu','var')
        [b_imu,a_imu] = butter(2,imu_lowpass_fc/(fs/2),'low');
    end
    accely_clean = medfilt1(accely_clean,imu_median_window);
    accely_clean = filtfilt(b_imu,a_imu,double(accely_clean));
else
    accely_clean = NaN(size(t));
end

accely_abs = abs(accely_clean);

%% ========================================================================
%  12. WOT / SPEED MASKS
%% ========================================================================

wot = valid & tps >= WOT_threshold;

high_speed_wot_60 = wot & v_wheel_kmh >= 60;
high_speed_wot_70 = wot & v_wheel_kmh >= 70;

%% ========================================================================
%  13. BASIC EXPOSURE METRICS
%% ========================================================================

t_valid = sum(valid)*dt;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                     SPEED EXPOSURE\n');
fprintf('===============================================================\n');
fprintf('\nValid analyzed time = %.1f s\n',t_valid);
fprintf('\nThreshold    Time above    Percent\n');
fprintf('----------------------------------\n');

for i = 1:length(speed_thresholds)
    Vth = speed_thresholds(i);
    mask = valid & v_wheel_kmh >= Vth;
    t_above = sum(mask)*dt;
    pct = 100*t_above/t_valid;
    fprintf('%6.0f km/h    %8.1f s     %6.2f %%\n',Vth,t_above,pct);
end

fprintf('\nMaximum wheel speed = %.2f km/h\n',max(v_wheel_kmh(valid)));

%% ========================================================================
%  13b. RESTORED: WOT LONGITUDINAL ACCELERATION ROLL-OFF VS SPEED
%
%  (This was in the original version of this script and got dropped in
%  the previous update -- restored here, unchanged logic.)
%% ========================================================================

accel_speed_bins = 10:5:90;
accel_bin_center = accel_speed_bins(1:end-1)+2.5;

ax_wot_mean = NaN(size(accel_bin_center));
ax_wot_median = NaN(size(accel_bin_center));
N_ax = zeros(size(accel_bin_center));

imu_x_wot_mean = NaN(size(accel_bin_center));
imu_x_wot_median = NaN(size(accel_bin_center));
N_imu_x = zeros(size(accel_bin_center));

for i = 1:length(accel_bin_center)

    v1 = accel_speed_bins(i);
    v2 = accel_speed_bins(i+1);

    mask = wot & v_wheel_kmh >= v1 & v_wheel_kmh < v2 & ...
        ax_est_g > -0.5 & ax_est_g < 1.5;
    vals = ax_est_g(mask);
    vals = vals(isfinite(vals));
    N_ax(i) = length(vals);
    if N_ax(i) >= 20
        ax_wot_mean(i) = mean(vals,'omitnan');
        ax_wot_median(i) = median(vals,'omitnan');
    end

    if accelx_available
        mask_x = wot & v_wheel_kmh >= v1 & v_wheel_kmh < v2 & ...
            accelx_clean > -2.0 & accelx_clean < 2.0;
        imu_x_vals = accelx_clean(mask_x);
        imu_x_vals = imu_x_vals(isfinite(imu_x_vals));
        N_imu_x(i) = length(imu_x_vals);
        if N_imu_x(i) >= 20
            imu_x_wot_mean(i) = mean(imu_x_vals,'omitnan');
            imu_x_wot_median(i) = median(imu_x_vals,'omitnan');
        end
    end

end

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              WOT LONGITUDINAL ACCELERATION VS SPEED\n');
fprintf('===============================================================\n');

fprintf('\nWHEEL-SPEED-DERIVED ACCELERATION\n');
fprintf('Speed        N       Mean ax      Median ax\n');
fprintf(' (km/h)                (g)           (g)\n');
fprintf('--------------------------------------------\n');
for i = 1:length(accel_bin_center)
    if N_ax(i) < 20, continue; end
    fprintf('%6.1f      %5d       %+7.3f       %+7.3f\n', ...
        accel_bin_center(i),N_ax(i),ax_wot_mean(i),ax_wot_median(i));
end

if accelx_available
    fprintf('\nCLEANED ACCELX -- SIGN MATCHED TO WHEEL-SPEED CONVENTION\n');
    fprintf('Speed        N       Mean ax      Median ax\n');
    fprintf(' (km/h)                (g)           (g)\n');
    fprintf('--------------------------------------------\n');
    for i = 1:length(accel_bin_center)
        if N_imu_x(i) < 20, continue; end
        fprintf('%6.1f      %5d       %+7.3f       %+7.3f\n', ...
            accel_bin_center(i),N_imu_x(i),imu_x_wot_mean(i),imu_x_wot_median(i));
    end
end

%% ========================================================================
%  13c. RESTORED: LATERAL-G ENVELOPE VS SPEED
%
%  (Also dropped in the previous update -- restored here, unchanged.)
%% ========================================================================

lat_speed_bins = 10:5:90;
lat_bin_center = lat_speed_bins(1:end-1)+2.5;

lat_mean_abs = NaN(size(lat_bin_center));
lat_median_abs = NaN(size(lat_bin_center));
lat_90_abs = NaN(size(lat_bin_center));
lat_95_abs = NaN(size(lat_bin_center));
N_lat = zeros(size(lat_bin_center));

lat_mean_signed = NaN(size(lat_bin_center));
lat_median_signed = NaN(size(lat_bin_center));

for i = 1:length(lat_bin_center)

    v1 = lat_speed_bins(i);
    v2 = lat_speed_bins(i+1);

    if ~accely_available
        continue;
    end

    mask_y = valid & v_wheel_kmh >= v1 & v_wheel_kmh < v2 & ...
        accely_clean > -3.0 & accely_clean < 3.0;
    y_signed = accely_clean(mask_y);
    y_signed = y_signed(isfinite(y_signed));
    N_lat(i) = length(y_signed);

    if N_lat(i) >= 20
        y_abs = abs(y_signed);
        lat_mean_abs(i) = mean(y_abs,'omitnan');
        lat_median_abs(i) = median(y_abs,'omitnan');
        lat_90_abs(i) = prctile(y_abs,90);
        lat_95_abs(i) = prctile(y_abs,95);
        lat_mean_signed(i) = mean(y_signed,'omitnan');
        lat_median_signed(i) = median(y_signed,'omitnan');
    end

end

if accely_available

    fprintf('\n');
    fprintf('===============================================================\n');
    fprintf('                 LATERAL-G VS SPEED\n');
    fprintf('===============================================================\n');
    fprintf('\nACCELY sign is retained in the data, but ABS(ACCELY) is used\n');
    fprintf('for the left/right-independent lateral-G envelope.\n');
    fprintf('\nSpeed       N      Median |Ay|    90th |Ay|    95th |Ay|\n');
    fprintf(' (km/h)                (g)           (g)           (g)\n');
    fprintf('-----------------------------------------------------------\n');

    for i = 1:length(lat_bin_center)
        if N_lat(i) < 20, continue; end
        fprintf('%6.1f     %5d        %7.3f       %7.3f       %7.3f\n', ...
            lat_bin_center(i),N_lat(i),lat_median_abs(i),lat_90_abs(i),lat_95_abs(i));
    end

end

%% ========================================================================
%  14. WOT EXPOSURE
%% ========================================================================

wot_time = sum(wot)*dt;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    WOT EXPOSURE\n');
fprintf('===============================================================\n');
fprintf('\nWOT threshold = %.1f %%\n',WOT_threshold);
fprintf('WOT time = %.1f s (%.1f%% of analyzed time)\n',wot_time,100*wot_time/t_valid);

%% ========================================================================
%  15. AERO FORCE / POWER (real baseline now)
%% ========================================================================

q = 0.5*rho*v_wheel_ms.^2;
Fdrag = CdA_base*q;
Pdrag = Fdrag.*v_wheel_ms;
Fdown = CLA_base*q;

%% ========================================================================
%  16. NEW: EMPIRICAL AVERAGE WOT POWER
%
%  Estimated directly from this session's data rather than assumed:
%    F_engine = m*ax_measured + F_drag(CdA_base)
%    P_engine = F_engine * V
%  averaged over WOT samples with sane, positive power values.
%  This is what converts drag ENERGY into actual TIME COST below.
%% ========================================================================

F_engine_est = mass_kg*ax_est_ms2 + Fdrag;
P_engine_est = F_engine_est .* v_wheel_ms;

valid_power = wot & isfinite(P_engine_est) & P_engine_est > 0 & ...
    P_engine_est < 60000 & v_wheel_kmh > 15;

avg_wot_power_W = mean(P_engine_est(valid_power),'omitnan');

fprintf('\n');
fprintf('===============================================================\n');
fprintf('              EMPIRICAL AVERAGE WOT POWER\n');
fprintf('===============================================================\n');
fprintf('\nEstimated from this session''s accel + drag data (not assumed):\n');
fprintf('  Average available WOT power = %.2f kW\n',avg_wot_power_W/1000);
fprintf('  (based on %d samples)\n',nnz(valid_power));

%% ========================================================================
%  17. DRAG ENERGY -- FULL SESSION (not just WOT) at real CdA_base
%% ========================================================================

E_drag_WOT_J = sum(Pdrag(wot),'omitnan')*dt;
E_drag_WOT_kJ = E_drag_WOT_J/1000;

E_drag_total_kJ = sum(Pdrag(valid),'omitnan')*dt/1000;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                  DRAG EXPOSURE / ENERGY\n');
fprintf('===============================================================\n');
fprintf('\nEstimated drag energy during WOT:\n  %.2f kJ\n',E_drag_WOT_kJ);
fprintf('\nEstimated drag energy over FULL valid session (CdA=%.3f):\n  %.2f kJ\n', ...
    CdA_base,E_drag_total_kJ);

%% ========================================================================
%  18. CdA SENSITIVITY -- NOW WITH TIME COST, using empirical WOT power
%% ========================================================================

CdA_cases = [CdA_base, 0.95*CdA_base, 0.90*CdA_base, 0.85*CdA_base, 0.80*CdA_base];

fprintf('\n');
fprintf('===============================================================\n');
fprintf('           CdA SENSITIVITY -- DRAG ENERGY AND TIME COST\n');
fprintf('===============================================================\n');
fprintf('\nCdA          Reduction    Energy saved     TIME SAVED\n');
fprintf(' (m^2)                       (kJ, full sess)   (s, this session)\n');
fprintf('-------------------------------------------------------------------\n');

time_saved_cases = NaN(size(CdA_cases));

for i = 1:length(CdA_cases)

    cda = CdA_cases(i);
    reduction = 100*(1-cda/CdA_base);

    Fdrag_i = cda*q;
    Pdrag_i = Fdrag_i.*v_wheel_ms;
    E_i_kJ = sum(Pdrag_i(valid),'omitnan')*dt/1000;

    dE_kJ = E_drag_total_kJ - E_i_kJ;   % positive = energy saved

    time_saved_cases(i) = (dE_kJ*1000)/avg_wot_power_W;

    fprintf('%7.3f      %6.1f %%       %9.2f          %+8.3f\n', ...
        cda,reduction,dE_kJ,time_saved_cases(i));

end

fprintf('\n(TIME SAVED is what gets compared against the mock endurance\n');
fprintf(' script''s +CLA time-saved number, in aero_direction_comparison.m)\n');

%% ========================================================================
%  19. CLA SENSITIVITY (unchanged, informational)
%% ========================================================================

test_speeds = [40 50 60 70 80 90];

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                 CLA SENSITIVITY (informational)\n');
fprintf('===============================================================\n');
fprintf('\nSpeed      Current DF     +10%% CLA extra DF\n');
fprintf(' (km/h)       (N)               (N)\n');
fprintf('-----------------------------------------------\n');

for V = test_speeds
    Vms = V/3.6;
    qV = 0.5*rho*Vms^2;
    F_current = CLA_base*qV;
    F_extra = 0.10*CLA_base*qV;
    fprintf('%5.0f        %9.1f          %9.1f\n',V,F_current,F_extra);
end

%% ========================================================================
%  20. PLOTS
%% ========================================================================

figure('Name','Competition Speed Trace','Position',[100 100 1100 650]);
plot(t,v_wheel_kmh,'LineWidth',1.0);
xlabel('CAN time (s)'); ylabel('Front Right wheel speed (km/h)');
title('Competition Speed Trace - Front Right Wheel Speed');
grid on;

figure('Name','Competition Speed Distribution','Position',[150 150 900 600]);
histogram(v_wheel_kmh(valid),35);
hold on;
for i = 1:length(speed_thresholds)
    xline(speed_thresholds(i),'--',sprintf('%d km/h',speed_thresholds(i)));
end
xlabel('Front Right wheel speed (km/h)'); ylabel('Sample count');
title('Competition Speed Distribution');
grid on;

% RESTORED: WOT longitudinal acceleration vs speed
figure('Name','Competition Longitudinal Acceleration','Position',[200 200 1000 650]);
scatter(v_wheel_kmh(wot),ax_est_g(wot),8,'filled','DisplayName','Wheel-speed derived');
hold on;
plot(accel_bin_center,ax_wot_median,'LineWidth',3,'DisplayName','Wheel-speed median');
if accelx_available
    scatter(v_wheel_kmh(wot),accelx_clean(wot),8,'DisplayName','Cleaned ACCELX');
    plot(accel_bin_center,imu_x_wot_median,'LineWidth',3,'DisplayName','ACCELX median');
end
xlabel('Front Right wheel speed (km/h)'); ylabel('Longitudinal acceleration (g)');
title('Competition WOT Longitudinal Acceleration Roll-Off');
grid on; legend('Location','best');

% RESTORED: Lateral acceleration vs speed
if accely_available
    figure('Name','Competition Lateral-G Envelope','Position',[250 250 1000 650]);
    scatter(v_wheel_kmh(valid),accely_clean(valid),6,'DisplayName','Cleaned signed ACCELY');
    hold on;
    plot(lat_bin_center,lat_95_abs,'LineWidth',3,'DisplayName','95th percentile |ACCELY|');
    plot(lat_bin_center,lat_90_abs,'LineWidth',2,'DisplayName','90th percentile |ACCELY|');
    xlabel('Front Right wheel speed (km/h)'); ylabel('Lateral acceleration (g)');
    title('Competition Lateral-G Envelope');
    grid on; legend('Location','best');
end

% RESTORED: Cleaned IMU channels
figure('Name','Cleaned Competition IMU','Position',[300 250 1100 800]);
subplot(2,1,1);
if accelx_available
    plot(t,accelx_raw,'LineWidth',0.7,'DisplayName','Raw ACCELX');
    hold on;
    plot(t,accelx_clean,'LineWidth',1.2,'DisplayName','Cleaned ACCELX (sign matched)');
    ylabel('ACCELX (g)'); title('Longitudinal IMU');
    legend('Location','best');
else
    title('ACCELX unavailable');
end
grid on;

subplot(2,1,2);
if accely_available
    plot(t,accely_raw,'LineWidth',0.7,'DisplayName','Raw ACCELY');
    hold on;
    plot(t,accely_clean,'LineWidth',1.2,'DisplayName','Cleaned ACCELY');
    ylabel('ACCELY (g)'); xlabel('CAN time (s)'); title('Lateral IMU');
    legend('Location','best');
else
    title('ACCELY unavailable');
end
grid on;

% RESTORED: Drag force / power over time
figure('Name','Competition Aero Drag Exposure','Position',[350 300 1000 650]);
yyaxis left;
plot(t,Fdrag,'LineWidth',1.2);
ylabel('Estimated drag force (N)');
yyaxis right;
plot(t,Pdrag/1000,'LineWidth',1.2);
ylabel('Estimated drag power (kW)');
xlabel('CAN time (s)');
title('Estimated Drag Force and Drag Power');
grid on;

figure('Name','CdA Sensitivity - Time Cost','Position',[400 300 900 600]);
bar(time_saved_cases);
xlabel('CdA case'); ylabel('Time saved over full session (s)');
title('Time Cost/Benefit of CdA Reduction (empirical WOT power)');
grid on;
xticklabels(arrayfun(@(c) sprintf('%.3f',c),CdA_cases,'UniformOutput',false));

%% ========================================================================
%  21. FINAL SUMMARY
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                    FINAL SUMMARY\n');
fprintf('===============================================================\n');

fprintf('\nSpeed exposure:\n');
fprintf('  >40 km/h = %.1f%%\n',100*sum(valid & v_wheel_kmh>40)*dt/t_valid);
fprintf('  >50 km/h = %.1f%%\n',100*sum(valid & v_wheel_kmh>50)*dt/t_valid);
fprintf('  >60 km/h = %.1f%%\n',100*sum(valid & v_wheel_kmh>60)*dt/t_valid);
fprintf('  >70 km/h = %.1f%%\n',100*sum(valid & v_wheel_kmh>70)*dt/t_valid);

fprintf('\nAero assumptions used:\n');
fprintf('  CLA = %.3f m^2\n',CLA_base);
fprintf('  CdA = %.3f m^2\n',CdA_base);

fprintf('\nEmpirical average WOT power = %.2f kW\n',avg_wot_power_W/1000);

fprintf('\nIMPORTANT:\n');
fprintf('Competition data contain no shockpots, so this analysis cannot\n');
fprintf('determine absolute downforce from the endurance run.\n');

%% ========================================================================
%  22. SAVE RESULTS
%% ========================================================================

competition_analysis = struct();
competition_analysis.dt = dt;
competition_analysis.speed_kmh = v_wheel_kmh;
competition_analysis.speed_ms = v_wheel_ms;
competition_analysis.accel_g = ax_est_g;
competition_analysis.accelx_available = accelx_available;
competition_analysis.accely_available = accely_available;
competition_analysis.accelx_raw_g = accelx_raw;
competition_analysis.accelx_clean_g = accelx_clean;
competition_analysis.accely_raw_g = accely_raw;
competition_analysis.accely_clean_g = accely_clean;
competition_analysis.accely_abs_g = accely_abs;
competition_analysis.WOT = wot;
competition_analysis.longitudinal_accel_bins_kmh = accel_bin_center;
competition_analysis.wheel_speed_accel_median_g = ax_wot_median;
competition_analysis.wheel_speed_accel_mean_g = ax_wot_mean;
competition_analysis.accelx_wot_median_g = imu_x_wot_median;
competition_analysis.accelx_wot_mean_g = imu_x_wot_mean;
competition_analysis.lateral_accel_bins_kmh = lat_bin_center;
competition_analysis.accely_abs_median_g = lat_median_abs;
competition_analysis.accely_abs_90th_g = lat_90_abs;
competition_analysis.accely_abs_95th_g = lat_95_abs;
competition_analysis.CLA = CLA_base;
competition_analysis.CdA = CdA_base;
competition_analysis.avg_wot_power_W = avg_wot_power_W;
competition_analysis.E_drag_total_kJ = E_drag_total_kJ;
competition_analysis.E_drag_WOT_kJ = E_drag_WOT_kJ;
competition_analysis.CdA_cases = CdA_cases;
competition_analysis.time_saved_cases = time_saved_cases;
competition_analysis.session_duration_s = t_valid;

assignin('base','competition_analysis',competition_analysis);

% NEW: save to .mat so aero_direction_comparison.m can load this run's
% results independently, without needing this workspace.
save('comp_endurance_analysis.mat','competition_analysis');

fprintf('\nResults saved to:\n');
fprintf('  competition_analysis (workspace)\n');
fprintf('  comp_endurance_analysis.mat\n');

fprintf('\n===============================================================\n');


%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function x = sanitize(x)
    x = double(x(:));
    x(isinf(x)) = NaN;
    x = fillmissing(x,'linear','EndValues','nearest');
end