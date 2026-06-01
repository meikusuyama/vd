load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat");

%% ===============================
% Global filters (tire pressure)
% ===============================
P_target = 10;          % psi
P_tol    = 0.5;

maskP = abs(P - P_target) < P_tol;

SA = SA(maskP);
FY = FY(maskP);
FZ = FZ(maskP);
MX = MX(maskP);
IA = IA(maskP);

%% ==========================================================
% PART 1 — Find SA that produces max FY (RH & LH)
% ==========================================================
target_Fz = -226;   % lb
FZ_tol    = 10;

maskFz = abs(FZ - target_Fz) < FZ_tol;

SA_f = SA(maskFz);
FY_f = FY(maskFz);

[maxFY_RH, idxRH] = max(FY_f);   % RH turn (positive FY)
[minFY_LH, idxLH] = min(FY_f);   % LH turn (negative FY)

SA_RH = SA_f(idxRH);
SA_LH = SA_f(idxLH);

fprintf("RH Max FY = %.2f lb at SA = %.2f deg\n", maxFY_RH, SA_RH);
fprintf("LH Max FY = %.2f lb at SA = %.2f deg\n", minFY_LH, SA_LH);

%% — Plot FY vs SA (still filtered by tire pressure)
figure; hold on;
scatter(SA_f, FY_f, ".", "MarkerEdgeColor", [0.8 0.8 0.8]);
scatter(SA_RH, maxFY_RH, 70, "r", "filled", "DisplayName", "RH Max FY");
scatter(SA_LH, minFY_LH, 70, "b", "filled", "DisplayName", "LH Max FY");

xlabel("Slip Angle (deg)");
ylabel("Lateral Force FY (lb)");
title(sprintf("FY vs SA at FZ = %d (lb) and 10 psi", target_Fz));
legend();
grid on;

%% ==========================================================
% PART 2 — Average the RH/LH slip angles into ONE target SA
% ==========================================================
SA_avg = mean([abs(SA_RH), abs(SA_LH)]);

fprintf("\nAveraged |SA| for FZ vs MX plotting = %.3f deg\n", SA_avg);

%% ==========================================================
% PART 3 — Produce FZ vs MX for the averaged slip angle
% ==========================================================
SA_tol = 0.3;                       % slip angle matching tolerance
IA_list = [0 2 4];                  % camber angles to plot
IA_tol = 0.15;

% Filter data near averaged SA
maskSA = abs(SA - SA_avg) < SA_tol;

FZ_sa = FZ(maskSA);
MX_sa = MX(maskSA);
IA_sa = IA(maskSA);

figure; hold on;

for ia = IA_list
    % filter by camber
    maskIA = abs(IA_sa - ia) < IA_tol;
    
    if sum(maskIA) < 5
        continue;
    end

    FZ_sub = FZ_sa(maskIA);
    MX_sub = MX_sa(maskIA);

    % sort by vertical load
    [FZ_sorted, idx] = sort(FZ_sub);
    MX_sorted = MX_sub(idx);

    % --- smooth best-fit curve using LOWESS ---
    MX_smooth = smooth(FZ_sorted, MX_sorted, 0.5, 'lowess');

    plot(FZ_sorted, MX_smooth, 'LineWidth', 2, ...
        'DisplayName', sprintf("IA = %d°", ia));
end


xlabel("Vertical Load FZ (lb)");
ylabel("Overturning Moment MX (lb-ft)");
title(sprintf("FZ vs MX at SA = %.2f° (10 psi)", SA_avg));
legend('Location','best');
grid on;
