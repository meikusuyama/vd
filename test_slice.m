load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

% FILTER BY TIRE PRESSURE 
%% ---------------------------------------------------------------
P_target = 10;
P_tol    = 0.5;

maskP = abs(P - P_target) < P_tol;

SA = SA(maskP);
FY = FY(maskP);
FZ = FZ(maskP);
MX = MX(maskP);
IA = IA(maskP);

%% ---------------------------------------------------------------
% PARAMETERS
%% ---------------------------------------------------------------
FZ_targets = -[226 188];   % loads you use to determine SA (that generates high FY)
FZ_tol = 10;         % load tolerance
SA_tol = 0.3;        % slip angle tolerance
IA_unique = [0 2 4]; % camber angles tested
IA_tol = 0.15;

% Loads you want to evaluate for Mx = 0
FZ_eval = -[50 100 150 200 250];
FZ_eval_tol = 10;

% Convert IA to degrees if needed
if max(IA) < 1  % probably radians
    IA = IA * 180/pi;
end

%% ---------------------------------------------------------------
% Step 1: For each FZ target, find slip angles that give max |FY|
%% ---------------------------------------------------------------

SA_list = [];   % averaged slip angle list (RH and LH)

for k = 1:length(FZ_targets)
    FZt = FZ_targets(k);

    % Filter data near the target FZ
    maskFZ = abs(FZ - FZt) < FZ_tol;
    if sum(maskFZ) < 1
        warning('No data near FZ = %.1f', FZt);
        continue
    end

    SA_f = SA(maskFZ);
    FY_f = FY(maskFZ);

    % --- Right-hand turn (negative SA usually): find max FY magnitude ---
    [~, idxRH] = max(abs(FY_f(SA_f < 0)));
    if ~isempty(idxRH)
        negSAs = SA_f(SA_f < 0);
        SA_RH = abs(negSAs(idxRH)); % store positive magnitude
    else
        SA_RH = NaN;
    end

    % --- Left-hand turn (positive SA usually): find max FY magnitude ---
    [~, idxLH] = max(abs(FY_f(SA_f > 0)));
    if ~isempty(idxLH)
        posSAs = SA_f(SA_f > 0);
        SA_LH = posSAs(idxLH);
    else
        SA_LH = NaN;
    end

    % --- Compute average of RH and LH slip angles if both exist ---
    if ~isnan(SA_RH) && ~isnan(SA_LH)
        SA_avg = mean([SA_RH, SA_LH]);    % average of magnitudes
    elseif ~isnan(SA_RH)
        SA_avg = SA_RH;
    elseif ~isnan(SA_LH)
        SA_avg = SA_LH;
    else
        SA_avg = NaN;
    end
    
    % append the average of RH and LH turn slip angles
    if ~isnan(SA_avg)
        SA_list = [SA_list; SA_avg]; %#ok<AGROW>
    end

    fprintf("FZ(load): %.4f \t SA: %.4f\n", FZt, SA_avg)
end

% Clean duplicates and keep only positive magnitudes
SA_list = unique(round(abs(SA_list),3));

fprintf('\nSlip angles selected for Mx=0 search:\n');
disp(SA_list');

%% ---------------------------------------------------------------
% Step 2: For each slip angle and each discrete FZ_eval,
%         find IA where Mx ≈ 0 (closest to zero or interpolated)
%% ---------------------------------------------------------------

IA_vs_FZ = {};  % cell: one cell per slip angle

for s = 1:length(SA_list)
    targetSA = SA_list(s);

    % Filter to near this slip angle 
    maskSA = abs(abs(SA) - targetSA) < SA_tol;
    FZ_s   = FZ(maskSA);
    MX_s   = MX(maskSA);
    IA_s   = IA(maskSA);

    combos = [];   % store (FZ_eval, IA)

    for fz_target = FZ_eval
        % Filter near this load
        maskFZ = abs(FZ_s - fz_target) < FZ_eval_tol;
        if sum(maskFZ) < 1
            continue
        end
    
        % Subset
        FZ_filt = FZ_s(maskFZ);
        MX_filt = MX_s(maskFZ);
        IA_filt = IA_s(maskFZ);
    
        % If no valid points, skip
        if isempty(MX_filt)
            continue
        end
    
        % Find IA with smallest |Mx| (closest to zero)
        [~, idxMin] = min(abs(MX_filt));
        IA_zero = IA_filt(idxMin);
    
        combos = [combos; fz_target, IA_zero]; %#ok<AGROW>
    end
    
    IA_vs_FZ{s} = combos;
end

%% ---------------------------------------------------------------
% Step 3: Plot everything on ONE graph
%% ---------------------------------------------------------------

colors = lines(length(SA_list));
markers = {'o','s','^','d','v','>','<','p','h'};  % unique marker per SA

figure; hold on;
for s = 1:length(SA_list)
    combo = IA_vs_FZ{s};
    if isempty(combo), continue; end

    % --- slight horizontal jitter so lines don't overlap ---
    xOffset = (s - ceil(length(SA_list)/2)) * 2;  % shift ± a few lb
    FZ_jittered = combo(:,1) + xOffset;

    % marker for each SA
    mkr = markers{mod(s-1, length(markers)) + 1};

    % plot
    plot(FZ_jittered, combo(:,2), '-o', ...
        'Color', colors(s,:), ...
        'Marker', mkr, ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', colors(s,:), ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.6, ...
        'DisplayName', sprintf('SA = %.2f°', SA_list(s)));
end

xlabel('Vertical Load FZ (lb)');
ylabel('Camber Angle IA (deg)');
title('Camber Synthesis: IA that Gives Mx ≈ 0 at Different FZs for Various SAs');
grid on;
legend('Location','bestoutside');
