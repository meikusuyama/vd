%% ========================================================================
% COASTDOWN DRAG FIT: Deceleration vs V^2 (GPS-staircase-corrected)
%
% Your phone's GPS speed only updates ~1x/second, but the file timestamps
% every ~10ms -- so each real GPS reading gets repeated across ~100 rows
% before jumping to the next value. Differentiating that oversampled
% staircase directly manufactures fake zero-deceleration points (every
% flat step) and fake banded deceleration values (every jump), which is
% exactly the pattern you saw in the plot -- none of it is real vehicle
% behavior.
%
% FIX: collapse each staircase step down to ONE real sample (its true
% GPS reading, at the average time of that step) before differentiating.
% This recovers the actual ~1 Hz speed signal and removes the artifact.
%
% For each file's time window:
%   1) Collapses the oversampled staircase to true GPS update points
%   2) Computes deceleration (-dv/dt) between REAL consecutive readings
%   3) Plots (-dv/dt) vs v^2 -- physically a straight line:
%           -dv/dt = Crr*g + (rho*CdA0)/(2*M) * v^2
%      slope = rho*CdA0/(2*M), intercept = Crr*g
%   4) Fits a line, overlays it, prints slope/intercept/Crr/CdA0
%% ========================================================================

clc; clear; close all;
warning('off','MATLAB:table:ModifiedAndSavedVarnames');

DATA_FOLDER = 'testing_data';

FILES = {'3_15_26_coastdown_no_aero.csv', '3_15_26_coastdown_no_aero2.csv', ...
         '3_15_26_coastdown_no_aero3.csv', '3_15_26_coastdown_no_aero4.csv', ...
         '3_15_26_coastdown_no_aero5.csv'};

TIME_WINDOWS = containers.Map();
TIME_WINDOWS('3_15_26_coastdown_no_aero.csv')  = [78.6, 98.0];
TIME_WINDOWS('3_15_26_coastdown_no_aero2.csv') = [83.8, 102.0];
TIME_WINDOWS('3_15_26_coastdown_no_aero3.csv') = [77.6, 95.0];
TIME_WINDOWS('3_15_26_coastdown_no_aero4.csv') = [63.39, 82.42];
TIME_WINDOWS('3_15_26_coastdown_no_aero5.csv') = [62, 80];

MASS_KG = 168 - 50*0.453592;   % EDIT: your best current mass estimate
rho = 1.225;                    % air density, kg/m^3

if isempty(FILES)
    d = dir(fullfile(DATA_FOLDER,'*.csv'));
    FILES = {d.name};
end
if isempty(FILES)
    error('No CSV files found in %s -- set DATA_FOLDER/FILES manually.',DATA_FOLDER);
end

fprintf('Assumed mass for this fit: %.2f kg\n\n',MASS_KG);

for f = 1:numel(FILES)

    fpath = fullfile(DATA_FOLDER,FILES{f});
    T = readtable(fpath);
    varnames = T.Properties.VariableNames;

    time_col  = find_col(varnames,{'Time'});
    speed_col = find_col(varnames,{'Speed'});
    date_col  = find_col(varnames,{'Date'});

    if isempty(speed_col)
        warning('%s: no Speed column found -- skipping.',FILES{f});
        continue;
    end

    if ~isempty(time_col)
        t = T.(time_col);
    elseif ~isempty(date_col)
        raw_date = T.(date_col);
        if iscell(raw_date) || isstring(raw_date)
            dt = datetime(raw_date,'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
        else
            dt = raw_date;
        end
        t = seconds(dt - dt(1));
    else
        warning('%s: no Time or Date column found -- skipping.',FILES{f});
        continue;
    end

    v = T.(speed_col);

    valid = isfinite(t) & isfinite(v);
    t = t(valid); v = v(valid);
    [t,order] = sort(t);
    v = v(order);

    % apply per-file time window
    if isKey(TIME_WINDOWS,FILES{f})
        win = TIME_WINDOWS(FILES{f});
        in_window = t >= win(1) & t <= win(2);
        t = t(in_window);
        v = v(in_window);
    else
        warning('%s: no time window defined -- using full range.',FILES{f});
    end

    if numel(t) < 10
        warning('%s: too few valid rows in window -- skipping.',FILES{f});
        continue;
    end

    % --- NEW: collapse the oversampled GPS staircase to true update points ---
    [t_real,v_real,n_blocks,update_interval] = collapse_staircase(t,v);

    fprintf('=== %s ===\n',FILES{f});
    if isKey(TIME_WINDOWS,FILES{f})
        win = TIME_WINDOWS(FILES{f});
        fprintf('  Window: [%.2f - %.2f s]\n',win(1),win(2));
    end
    fprintf('  Raw samples in window: %d  ->  real GPS updates: %d  (~%.2f Hz)\n', ...
        numel(t),n_blocks,1/update_interval);

    if n_blocks < 5
        fprintf('  Too few real updates to fit reliably -- skipping.\n\n');
        continue;
    end

    a_real = gradient(v_real,t_real);

    v2 = v_real.^2;
    decel = -a_real;

    p = polyfit(v2,decel,1);
    slope = p(1);
    intercept = p(2);

    Crr = intercept/9.81;
    CdA0 = slope*2*MASS_KG/rho;

    fprintf('  Slope     = %.6f  (1/m, = rho*CdA0/(2M))\n',slope);
    fprintf('  Intercept = %.5f  (m/s^2, = Crr*g)\n',intercept);
    fprintf('  --> Crr  = %.4f\n',Crr);
    fprintf('  --> CdA0 = %.3f m^2  (at M=%.1f kg)\n',CdA0,MASS_KG);
    fprintf('\n');

    % --- plot: deceleration vs v^2, with fitted line ---
    figure('Name',FILES{f},'Position',[100+30*f 100 800 600]);
    scatter(v2,decel,40,'filled','MarkerFaceAlpha',0.6); hold on;

    v2_line = linspace(0,max(v2),50);
    decel_line = intercept + slope*v2_line;
    plot(v2_line,decel_line,'r-','LineWidth',2);

    xlabel('v^2 (raw speed units^2)');
    ylabel('Deceleration, -dv/dt (raw speed units / s)');
    title(sprintf('%s: Deceleration vs V^2 (GPS-corrected)',FILES{f}),'Interpreter','none');
    legend('Real GPS-update points','Linear fit','Location','best');
    grid on;

    text(0.05,0.92, ...
        sprintf('slope = %.5g\nintercept = %.5g\nCrr = %.4f\nCdA0 = %.3f m^2 (M=%.0f kg)\nn=%d real updates', ...
        slope,intercept,Crr,CdA0,MASS_KG,n_blocks), ...
        'Units','normalized','VerticalAlignment','top', ...
        'BackgroundColor','w','EdgeColor','k');

end

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function col = find_col(varnames,candidates)
    col = '';
    for i = 1:numel(candidates)
        idx = find(strcmpi(varnames,candidates{i}),1);
        if ~isempty(idx)
            col = varnames{idx};
            return;
        end
    end
end

function [t_real,v_real,n_blocks,update_interval] = collapse_staircase(t,v)
    % Collapses runs of identical consecutive speed values (an
    % oversampled GPS staircase) down to one representative point per
    % run: the true speed value, at the mean time of that run.
    t_real = [];
    v_real = [];
    i = 1;
    n = numel(v);
    while i <= n
        j = i;
        while j < n && v(j+1) == v(i)
            j = j+1;
        end
        t_real(end+1,1) = mean(t(i:j)); %#ok<AGROW>
        v_real(end+1,1) = v(i);         %#ok<AGROW>
        i = j+1;
    end
    n_blocks = numel(v_real);
    if n_blocks > 1
        update_interval = mean(diff(t_real));
    else
        update_interval = NaN;
    end
end