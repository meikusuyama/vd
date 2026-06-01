load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

%% ---------------------------------------------------------------
% PARAMETERS
%% ---------------------------------------------------------------
FZ_targets = -[226 188];   % loads you care about for SA selection
FZ_tol = 10;         % load tolerance
SA_tol = 0.3;        % slip angle tolerance
IA_unique = [0 2 4]; % camber angles tested
IA_tol = 0.15;

% Loads you want to evaluate for Mx = 0
FZ_eval = -[50 100 150 200 250];
FZ_eval_tol = 10;

%% ---------------------------------------------------------------
% Step 1: For each FZ target, find slip angles that give max |FY|
%% ---------------------------------------------------------------

SA_list = [];   % combined slip angle list (RH and LH stored positive)

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

    % Append to list if valid
    SA_valid = [SA_RH; SA_LH];
    SA_valid = SA_valid(~isnan(SA_valid));

    SA_list = [SA_list; SA_valid]; %#ok<AGROW>
end

% Clean duplicates and keep only positive magnitudes
SA_list = unique(round(abs(SA_list),3));

fprintf('Slip angles selected for Mx=0 search:\n');
disp(SA_list');

%% ---------------------------------------------------------------
% Step 2: For each slip angle and each discrete FZ_eval,
%         find IA where Mx = 0
%% ---------------------------------------------------------------

IA_vs_FZ = {};  % cell: one cell per slip angle

for s = 1:length(SA_list)
    targetSA = SA_list(s);

    % Filter full dataset to only this slip angle (RH & LH merged)
    maskSA = abs(abs(SA) - targetSA) < SA_tol;
    FZ_s   = FZ(maskSA);
    MX_s   = MX(maskSA);
    IA_s   = IA(maskSA);

    combos = [];   % store (FZ_eval, IA)

    % Loop through the evaluated loads
    for fz_target = FZ_eval
        % Data near this FZ window
        maskFZ = abs(FZ_s - fz_target) < FZ_eval_tol;
        if sum(maskFZ) < 3
            continue
        end

        % Within this FZ range, loop through camber angles
        for ia = IA_unique
            maskIA = abs(IA_s - ia) < IA_tol;
            maskBoth = maskFZ & maskIA;

            if sum(maskBoth) < 1
                continue
            end

            FZ_filt = FZ_s(maskBoth);
            MX_filt = MX_s(maskBoth);

            % Sort by FZ for stable zero detection
            [FZ_sorted, idx] = sort(FZ_filt);
            MX_sorted = MX_filt(idx);

            % Look for Mx sign change in this local range
            FZ_zero = NaN;
            for j = 1:length(MX_sorted)-1
                if MX_sorted(j) == 0
                    FZ_zero = FZ_sorted(j);
                    break
                elseif MX_sorted(j)*MX_sorted(j+1) < 0
                    % Linear interpolation
                    FZ_zero = interp1( ...
                        [MX_sorted(j), MX_sorted(j+1)], ...
                        [FZ_sorted(j), FZ_sorted(j+1)], 0);
                    break
                end
            end

            if ~isnan(FZ_zero)
                combos = [combos; FZ_zero, ia]; %#ok<AGROW>
            end
        end
    end

    IA_vs_FZ{s} = combos;
end

%% ---------------------------------------------------------------
% Step 3: Plot everything on ONE graph
%% ---------------------------------------------------------------

colors = lines(length(SA_list));

figure; hold on;
for s = 1:length(SA_list)
    combo = IA_vs_FZ{s};
    if isempty(combo), continue; end

    plot(combo(:,1), combo(:,2), 'o', 'LineWidth', 1.4, ...
        'Color', colors(s,:), ...
        'DisplayName', sprintf('SA = %.2f°', SA_list(s)));
end

xlabel('Vertical Load FZ (lb)');
ylabel('Camber Angle IA (deg)');
title('Camber Synthesis: IA that Gives Mx ≈ 0 (Discrete FZ points)');
grid on;
legend('Location','bestoutside');
