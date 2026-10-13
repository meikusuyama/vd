%% ========================================================================
%  AERO DIRECTION COMPARISON: DOWNFORCE vs. DRAG REDUCTION
%
%  TWO SCENARIOS, COMPUTED SIDE BY SIDE:
%
%    SCENARIO A (self-consistent): downforce AND drag both measured on
%    the MOCK ENDURANCE session only. Same track, same conditions, same
%    car state throughout -- no cross-track assumption needed. This is
%    the more methodologically honest number, but Alameda is a limited
%    space and may not represent your real competition course's speed
%    range or corner mix.
%
%    SCENARIO B (cross-track, original approach): downforce from mock
%    endurance (only session with GPS/corner data), drag from the real
%    competition session (comp has no GPS, so it can't do corners --
%    this is the reason this scenario exists at all). Drag number is
%    measured on your ACTUAL competition track, but paired with a
%    downforce number from a different, smaller track.
%
%  Comparing A and B tells you how much the track mismatch in B is
%  actually distorting the answer. If A and B roughly agree, the
%  mismatch isn't a big deal. If they disagree substantially, trust A's
%  drag side less (Alameda's speed range is probably not representative
%  of full competition straights) but trust A's INTERNAL CONSISTENCY
%  more than B's cross-track pairing.
%
%  REQUIRES: run mock_endurance_corner_analysis.m and
%  comp_endurance_analysis.m first so their .mat files exist.
%
%% ========================================================================

clc;
clear;
close all;

REFERENCE_DURATION_S = 1395.891;   % EDIT: match lapsim's ENDURANCE_WIN_TIME if it changes

%% ========================================================================
%  1. LOAD BOTH ANALYSES
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('         AERO DIRECTION COMPARISON (DOWNFORCE vs DRAG)\n');
fprintf('===============================================================\n');

if ~isfile('mock_endurance_analysis.mat')
    error(['mock_endurance_analysis.mat not found -- run ' ...
        'mock_endurance_corner_analysis.m first.']);
end
if ~isfile('comp_endurance_analysis.mat')
    error(['comp_endurance_analysis.mat not found -- run ' ...
        'comp_endurance_analysis.m first.']);
end

load('mock_endurance_analysis.mat','corner_analysis');
load('comp_endurance_analysis.mat','competition_analysis');

fprintf('\nMock endurance: CLA=%.2f, CdA=%.2f, session=%.1fs\n', ...
    corner_analysis.CLA_base,corner_analysis.CdA_base,corner_analysis.session_duration_s);
fprintf('Comp endurance: CdA=%.2f, session=%.1fs (no GPS -- drag only, no corners)\n', ...
    competition_analysis.CdA,competition_analysis.session_duration_s);

if ~isfield(corner_analysis,'time_saved_cases')
    error(['corner_analysis is missing time_saved_cases -- rerun the updated ' ...
        'mock_endurance_corner_analysis.m (with self-consistent drag added) first.']);
end

if abs(corner_analysis.CLA_base - competition_analysis.CLA) > 1e-6
    warning('CLA_base differs between the two analyses -- re-run both with matching baseline.');
end

match_pcts = [5 10 15 20];

%% ========================================================================
%  2. DOWNFORCE SIDE (same for both scenarios -- only mock has corners)
%% ========================================================================

mock_duration = corner_analysis.session_duration_s;

CLA_time_saved_10pct_scaled = ...
    corner_analysis.total_time_saved_plus10 / mock_duration * REFERENCE_DURATION_S;

CLA_scaled_table = CLA_time_saved_10pct_scaled * (match_pcts/10);

%% ========================================================================
%  3. SCENARIO A: SELF-CONSISTENT (drag from mock endurance itself)
%% ========================================================================

CdA_cases_mock = corner_analysis.CdA_cases;
CdA_pct_change_mock = 100*(1 - CdA_cases_mock/corner_analysis.CdA_base);
CdA_time_saved_mock_scaled = ...
    corner_analysis.time_saved_cases / mock_duration * REFERENCE_DURATION_S;

CdA_scaled_table_A = interp1(CdA_pct_change_mock,CdA_time_saved_mock_scaled, ...
    match_pcts,'linear','extrap');

ratio_A = CdA_scaled_table_A ./ CLA_scaled_table;

%% ========================================================================
%  4. SCENARIO B: CROSS-TRACK (drag from competition session)
%% ========================================================================

comp_duration = competition_analysis.session_duration_s;

CdA_cases_comp = competition_analysis.CdA_cases;
CdA_pct_change_comp = 100*(1 - CdA_cases_comp/competition_analysis.CdA);
CdA_time_saved_comp_scaled = ...
    competition_analysis.time_saved_cases / comp_duration * REFERENCE_DURATION_S;

CdA_scaled_table_B = interp1(CdA_pct_change_comp,CdA_time_saved_comp_scaled, ...
    match_pcts,'linear','extrap');

ratio_B = CdA_scaled_table_B ./ CLA_scaled_table;

%% ========================================================================
%  5. PRINT BOTH SCENARIOS SIDE BY SIDE
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('   SCENARIO A: SELF-CONSISTENT (both from mock endurance)\n');
fprintf('===============================================================\n');
fprintf('\n%%change   +CLA benefit (s)   -CdA benefit (s)   ratio (drag/downforce)\n');
fprintf('---------------------------------------------------------------------\n');
for i = 1:numel(match_pcts)
    fprintf('  %3d%%        %8.2f            %8.2f              %6.2fx\n', ...
        match_pcts(i),CLA_scaled_table(i),CdA_scaled_table_A(i),ratio_A(i));
end
fprintf('\nAverage ratio (Scenario A) = %.2fx\n',mean(ratio_A));

fprintf('\n');
fprintf('===============================================================\n');
fprintf('   SCENARIO B: CROSS-TRACK (downforce=mock, drag=competition)\n');
fprintf('===============================================================\n');
fprintf('\n%%change   +CLA benefit (s)   -CdA benefit (s)   ratio (drag/downforce)\n');
fprintf('---------------------------------------------------------------------\n');
for i = 1:numel(match_pcts)
    fprintf('  %3d%%        %8.2f            %8.2f              %6.2fx\n', ...
        match_pcts(i),CLA_scaled_table(i),CdA_scaled_table_B(i),ratio_B(i));
end
fprintf('\nAverage ratio (Scenario B) = %.2fx\n',mean(ratio_B));

%% ========================================================================
%  6. COMPARE THE TWO SCENARIOS
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                  SCENARIO COMPARISON\n');
fprintf('===============================================================\n');

pct_diff = 100*(mean(ratio_B)-mean(ratio_A))/mean(ratio_A);

fprintf('\nScenario A (self-consistent, mock-only) ratio = %.2fx\n',mean(ratio_A));
fprintf('Scenario B (cross-track, mock+comp)      ratio = %.2fx\n',mean(ratio_B));
fprintf('Difference = %+.1f%%\n',pct_diff);

if abs(pct_diff) < 20
    fprintf('\nThe two scenarios roughly agree -- the track mismatch in\n');
    fprintf('Scenario B does not appear to be distorting the answer much.\n');
else
    fprintf('\nThe two scenarios disagree meaningfully -- this IS likely the\n');
    fprintf('track mismatch showing up (Alameda''s speed range/corner mix\n');
    fprintf('differs from real competition). Scenario A is more internally\n');
    fprintf('consistent, but remember its DRAG side is measured on a limited-\n');
    fprintf('space test, which may not reach realistic competition speeds --\n');
    fprintf('so A''s drag number could itself be biased (in either direction)\n');
    fprintf('by Alameda not exposing the car to real straight-line speeds.\n');
end

fprintf('\nScenario A verdict: %s\n',local_verdict(mean(ratio_A)));
fprintf('Scenario B verdict: %s\n',local_verdict(mean(ratio_B)));

fprintf('\nCROSS-CHECK: compare both verdicts against the lapsim''s own\n');
fprintf('"RECOMMENDED DIRECTION" output (aero_spec_fast_model, section 11).\n');

%% ========================================================================
%  NEW: INDUCED-DRAG COUPLED COMPARISON (CLA and CdA move together)
%
%  Everything above treats +CLA and -CdA as independent, isolated
%  levers (holding the other fixed) -- useful for "which single lever
%  is more powerful," but not what physically happens: a real wing
%  change moves CLA and CdA together. This section uses the SAME
%  induced-drag relationship the lapsim (aero_spec_fast_model) uses --
%  CdA(CLA) = CdA0 + K*CLA^2 -- to couple them, then nets the two
%  telemetry-based benefits together to see what telemetry says about
%  the ACTUAL coupled move, directly comparable to the lapsim's own
%  curve-based sweep result.
%
%  K_INDUCED_DRAG is HARDCODED here for now, computed the same way the
%  lapsim computes it (K = (BASE_CDA-CDA0)/BASE_CLA^2, using this run's
%  loaded CLA_base/CdA_base). Update by hand if the lapsim's baseline or
%  CdA0 changes; not yet wired to load directly from the lapsim's saved
%  results.
%% ========================================================================

CDA0_BARE_CHASSIS = 0.95;   % m^2, from coastdown testing (hardcoded, matches lapsim)
K_INDUCED_DRAG = (corner_analysis.CdA_base - CDA0_BARE_CHASSIS) / corner_analysis.CLA_base^2;

fprintf('\n');
fprintf('===============================================================\n');
fprintf('   INDUCED-DRAG COUPLED COMPARISON (CLA and CdA move together)\n');
fprintf('===============================================================\n');
fprintf('\nUsing CdA(CLA) = %.3f + %.5f*CLA^2 (same relationship as the lapsim)\n', ...
    CDA0_BARE_CHASSIS,K_INDUCED_DRAG);
fprintf('This asks: if +CLA%% ALSO drags CdA up by the coupled amount (not\n');
fprintf('held fixed), what does telemetry say the NET time change is?\n');

coupled_CdA_new = NaN(size(match_pcts));
coupled_CdA_pct_change = NaN(size(match_pcts));
coupled_CdA_cost_A = NaN(size(match_pcts));
coupled_CdA_cost_B = NaN(size(match_pcts));
coupled_net_A = NaN(size(match_pcts));
coupled_net_B = NaN(size(match_pcts));

for i = 1:numel(match_pcts)

    pct = match_pcts(i);
    CLA_new = corner_analysis.CLA_base*(1+pct/100);
    CdA_new = CDA0_BARE_CHASSIS + K_INDUCED_DRAG*CLA_new^2;
    coupled_CdA_new(i) = CdA_new;

    % negative = CdA INCREASED relative to baseline (expected direction
    % when CLA goes up)
    coupled_CdA_pct_change(i) = 100*(1 - CdA_new/corner_analysis.CdA_base);

    % interpolate the drag time COST at this %CdA change from each
    % scenario's already-computed sensitivity curve (sign-flipped,
    % since a CdA INCREASE is a time cost, not a time saving)
    coupled_CdA_cost_A(i) = -interp1(CdA_pct_change_mock,CdA_time_saved_mock_scaled, ...
        coupled_CdA_pct_change(i),'linear','extrap');
    coupled_CdA_cost_B(i) = -interp1(CdA_pct_change_comp,CdA_time_saved_comp_scaled, ...
        coupled_CdA_pct_change(i),'linear','extrap');

    coupled_net_A(i) = CLA_scaled_table(i) - coupled_CdA_cost_A(i);
    coupled_net_B(i) = CLA_scaled_table(i) - coupled_CdA_cost_B(i);

end

fprintf('\n%%CLA    CdA (new, m^2)   %%CdA increase   CLA benefit (s)   NET-A (s)   NET-B (s)\n');
fprintf('  (+CLA)                (positive =        (gain from       (+ = time   (+ = time\n');
fprintf('                         more drag)         downforce)       GAINED,     GAINED,\n');
fprintf('                                                             - = time    - = time\n');
fprintf('                                                             LOST)       LOST)\n');
fprintf('----------------------------------------------------------------------------------\n');
for i = 1:numel(match_pcts)
    fprintf('  %3d%%       %.3f            %+6.2f%%           %+7.2f          %+7.2f     %+7.2f\n', ...
        match_pcts(i),coupled_CdA_new(i),-coupled_CdA_pct_change(i), ...
        CLA_scaled_table(i),coupled_net_A(i),coupled_net_B(i));
end

fprintf('\n');
if all(coupled_net_A < 0) && all(coupled_net_B < 0)
    fprintf('VERDICT: telemetry says the COUPLED move the lapsim recommends\n');
    fprintf('(+CLA, accepting the coupled +CdA) is a NET LOSS at every tested\n');
    fprintf('percentage, in both scenarios. This disagrees with the lapsim''s\n');
    fprintf('own sweep result -- likely because the lapsim runs on a\n');
    fprintf('simulated track file, while this is measured on your real\n');
    fprintf('car/course. Weight this real-data result accordingly.\n');
elseif all(coupled_net_A > 0) && all(coupled_net_B > 0)
    fprintf('VERDICT: telemetry AGREES with the lapsim -- the coupled +CLA\n');
    fprintf('move is a net gain in both scenarios. Reasonably strong combined\n');
    fprintf('evidence in favor of the lapsim''s recommended direction.\n');
else
    fprintf('VERDICT: mixed -- scenarios disagree on the sign of the coupled\n');
    fprintf('net effect. Treat the lapsim''s recommendation as unconfirmed\n');
    fprintf('until this is resolved (e.g. with a better-matched track).\n');
end

%% ========================================================================
%  7. PLOT -- BOTH SCENARIOS
%% ========================================================================

figure('Name','Aero Direction Comparison - Both Scenarios','Position',[100 100 1100 650]);

plot(match_pcts,CLA_scaled_table,'o-','LineWidth',2,'Color',[0 0.45 0.74], ...
    'DisplayName','+CLA (downforce, from mock)');
hold on;
plot(match_pcts,CdA_scaled_table_A,'s-','LineWidth',2,'Color',[0.85 0.33 0.10], ...
    'DisplayName','-CdA Scenario A (drag, from mock)');
plot(match_pcts,CdA_scaled_table_B,'s--','LineWidth',2,'Color',[0.49 0.18 0.56], ...
    'DisplayName','-CdA Scenario B (drag, from comp)');

xlabel('% change');
ylabel(sprintf('Time saved over %.0f s reference (s)',REFERENCE_DURATION_S));
title('Downforce vs. Drag Reduction: Self-Consistent vs. Cross-Track');
legend('Location','best');
grid on;

% NEW: coupled net-benefit plot (induced-drag relationship)
figure('Name','Coupled Net Benefit (Induced-Drag)','Position',[150 150 900 650]);
plot(match_pcts,coupled_net_A,'o-','LineWidth',2,'DisplayName','Net (Scenario A, mock drag)');
hold on;
plot(match_pcts,coupled_net_B,'s-','LineWidth',2,'DisplayName','Net (Scenario B, comp drag)');
yline(0,'k--','Break-even');
xlabel('% CLA increase (CdA coupled via induced-drag curve)');
ylabel(sprintf('Net time saved over %.0f s reference (s)',REFERENCE_DURATION_S));
title('Net Effect of Coupled +CLA/+CdA Move (Induced-Drag Relationship)');
legend('Location','best');
grid on;

%% ========================================================================
%  8. SAVE
%% ========================================================================

direction_comparison = struct();
direction_comparison.REFERENCE_DURATION_S = REFERENCE_DURATION_S;
direction_comparison.match_pcts = match_pcts;
direction_comparison.CLA_scaled_table = CLA_scaled_table;
direction_comparison.scenario_A = struct('CdA_scaled_table',CdA_scaled_table_A,'ratio',ratio_A);
direction_comparison.scenario_B = struct('CdA_scaled_table',CdA_scaled_table_B,'ratio',ratio_B);
direction_comparison.coupled = struct('K_INDUCED_DRAG',K_INDUCED_DRAG, ...
    'CDA0_BARE_CHASSIS',CDA0_BARE_CHASSIS,'coupled_CdA_pct_change',coupled_CdA_pct_change, ...
    'net_A',coupled_net_A,'net_B',coupled_net_B);

assignin('base','direction_comparison',direction_comparison);
save('aero_direction_comparison.mat','direction_comparison');

fprintf('\nResults saved to:\n');
fprintf('  direction_comparison (workspace)\n');
fprintf('  aero_direction_comparison.mat\n');

fprintf('\n===============================================================\n');
fprintf('                         COMPLETE\n');
fprintf('===============================================================\n');

%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function verdict = local_verdict(r)
    if r > 1.2
        verdict = sprintf('favor DRAG REDUCTION (%.1fx)',r);
    elseif r < 0.83
        verdict = sprintf('favor ADDED DOWNFORCE (%.1fx)',1/r);
    else
        verdict = sprintf('no strong direction either way (%.2fx)',r);
    end
end