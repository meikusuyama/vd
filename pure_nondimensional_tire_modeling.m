load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

% ---------- STEP 1: CLEAN DATA USING SLIDING WINDOWS ----------
% Remove Break-in (early time) / Spring-rate blocks
win = 200; % checking the last 200 samples

stdSA = movstd(SA, win);
stdFZ = movstd(FZ, win);
stdIA = movstd(IA, win);

isSweep = stdSA > 1.2 ...     % ensure slip angle is actively sweeping
        & stdFZ < 80  ...     % load is relatively stable
        & stdIA < 0.5;        % camber is steady


% remove short glitches
edges = diff([0; isSweep; 0]);
% detect where sweep blocks start and end
starts = find(edges == 1);
ends   = find(edges == -1) - 1;

% Only keep sweep blocks longer than 500 samples.
dur = ends - starts;
good = dur > 500;   % minimum block length

validMask = false(size(SA));
for i = find(good)'
    validMask(starts(i):ends(i)) = true;
end


% apply mask
SA = SA(validMask);
FY = FY(validMask);
FZ = FZ(validMask);
IA = IA(validMask);
SR = SR(validMask);
P  = P(validMask);
ET = ET(validMask);


% ---------- STEP 2: IDENTIFY INDIVIDUAL SWEEPS ----------
dSA = gradient(SA, ET);

% Keep only time points where slip angle is actively moving
threshold = 0.1; % deg/sec
sweepMask = abs(dSA) > threshold;

edges = diff([0; sweepMask; 0]);
startIdx = find(edges == 1);
endIdx   = find(edges == -1) - 1;

% ---------- STEP 3: SPLIT TRIANGLE INTO MONOTONIC RATE SWEEPS ----------
rateSweeps = struct;
k = 1;

for i = 1:length(startIdx)

    block = startIdx(i):endIdx(i);

    % Operating condition checks (whole triangle)
    if mean(abs(SR(block)+1)) > 0.05, continue; end     %SR ~ -1
    if mean(abs(IA(block))) > 0.2, continue; end        %IA ~ 0
    if std(FZ(block)) > 20, continue; end

    SA_blk = SA(block);
    FY_blk = FY(block);
    FZ_blk = FZ(block);
    ET_blk = ET(block);

    % Slip-rate
    dSA = gradient(SA_blk, ET_blk);
    rateSign = sign(dSA);

    % Split where sign changes - into three sections
    edges = diff(rateSign);
    splitPts = [1; find(edges~=0)+1; length(block)];

    for j = 1:length(splitPts)-1
        idx = splitPts(j):splitPts(j+1)-1;
        if length(idx) < 40, continue; end

        rateSweeps(k).SA = SA_blk(idx);
        rateSweeps(k).FY = FY_blk(idx);
        rateSweeps(k).FZ = mean(FZ_blk(idx));
        rateSweeps(k).P  = mean(P(block));
        rateSweeps(k).rateSign = sign(mean(dSA(idx)));  % +1 or −1
        k = k + 1;
    end
end


% ---------- STEP 4: BIN BY PRESSURE AND FZ ----------
% Each bin represents ONE test condition:
% fixed load (FZ) and fixed pressure (P)

Pbin  = round([rateSweeps.P]);
FZbin = round([rateSweeps.FZ]/50)*50;


uniqueP  = unique(Pbin);
uniqueFZ = unique(FZbin);

groups = struct;

for i = 1:length(uniqueP)
    for j = 1:length(uniqueFZ)
        % Select all monotonic sweeps at this (P, FZ)
        mask = (Pbin==uniqueP(i)) & (FZbin==uniqueFZ(j));

        % Each group corresponds to one "sweep group"
        groups(i,j).P = uniqueP(i);
        groups(i,j).FZ = uniqueFZ(j);
        groups(i,j).sweeps = rateSweeps(mask);
    end
end


% ---------- STEP 5: PARAMETER EXTRACTION PER RATE SWEEP ----------
% Extract TTC parameters (Ca, mu, SH, SV) from each monotonic sweep
% Then apply TTC 2+/1− averaging inside each (P, FZ) group
processed = struct;
c = 0;

for g = 1:numel(groups)

    sw = groups(g).sweeps;
    if isempty(sw), continue; end

    Ca_pos = []; Ca_neg = [];
    mu_pos = []; mu_neg = [];
    SH_pos = []; SH_neg = [];
    SV_pos = []; SV_neg = [];

    for k = 1:length(sw)

        SAk = sw(k).SA;
        FYk = sw(k).FY;
        FZk = sw(k).FZ;

        % --- friction ---
        mu = max(abs(FYk)) / abs(FZk);

        % --- stiffness & shifts ---
        [Ca, SH, SV] = find_shift_params(SAk, FYk, FZk);
        if isnan(Ca), continue; end

        % Average parameters within each direction,
        % then average positive vs negative directions

        if sw(k).rateSign > 0
            Ca_pos(end+1) = Ca;
            mu_pos(end+1) = mu;
            SH_pos(end+1) = SH;
            SV_pos(end+1) = SV;
        else
            Ca_neg(end+1) = Ca;
            mu_neg(end+1) = mu;
            SH_neg(end+1) = SH;
            SV_neg(end+1) = SV;
        end
    end

    if isempty(Ca_pos) || isempty(Ca_neg)
        continue;
    end

    % ---------- TTC 2+/1– PARAMETER AVERAGING ----------
    Ca_p = mean(Ca_pos);
    mu_p = mean(mu_pos);
    SH_p = mean(SH_pos);
    SV_p = mean(SV_pos);

    Ca_n = mean(Ca_neg);
    mu_n = mean(mu_neg);
    SH_n = mean(SH_neg);
    SV_n = mean(SV_neg);

    c = c + 1;
    processed(c).Ca = (Ca_p + Ca_n)/2;
    processed(c).mu = (mu_p + mu_n)/2;
    processed(c).SH = (SH_p + SH_n)/2;
    processed(c).SV = (SV_p + SV_n)/2;
    processed(c).FZ = groups(g).FZ;
    processed(c).P  = groups(g).P;

    % representative curve (any triangle from this group)
    processed(c).SA = sw(1).SA;
    processed(c).FY = sw(1).FY;
end



% ----------- Step 6: Normalization ----------
% Remove load and pressure dependence so all curves collapse
% onto a universal, dimensionless shape

normalized = struct;
n = 0;

for i = 1:length(processed)

    SA = processed(i).SA;
    FY = processed(i).FY;
    FZ = processed(i).FZ;

    Ca = processed(i).Ca;
    mu = processed(i).mu;
    SH = processed(i).SH;
    SV = processed(i).SV;

    % Convert slip angle to radians
    alpha = deg2rad(SA(:));

    % --- Apply TTC shifts ---
    alpha_shift = alpha - SH / Ca;
    FY_shift    = FY(:) - abs(FZ) * SV;

    % --- TTC lateral normalization ---
    alpha_n = (Ca * tan(alpha_shift)) / (mu * abs(FZ));
    FY_n    = FY_shift / (mu * abs(FZ));

    n = n + 1;
    normalized(n).SA = alpha_n;
    normalized(n).Fy = FY_n;
    normalized(n).FZ = FZ;
    normalized(n).P  = processed(i).P;
end


FZ_vec = arrayfun(@(s)s.FZ, processed);
P_vec  = arrayfun(@(s)s.P,  processed);

Ca_vec = arrayfun(@(s)s.Ca, processed);
mu_vec = arrayfun(@(s)s.mu, processed);
SH_vec = arrayfun(@(s)s.SH, processed);
SV_vec = arrayfun(@(s)s.SV, processed);

% Force column vectors
FZ_vec = FZ_vec(:);
P_vec  = P_vec(:);
Ca_vec = Ca_vec(:);
mu_vec = mu_vec(:);
SH_vec = SH_vec(:);
SV_vec = SV_vec(:);


% ------------- Step 7: Magic Formula Function -----------
% Fit a universal MF shape to the normalized data
function params = fit_magic_formula(x, y)

    % Simplified MF (normalized form)
    mf = @(p,x) sin(p(2) * atan( ...
         p(1)*x - p(3)*(p(1)*x - atan(p(1)*x)) ));

    p0 = [12, 1.3, 0.1];  %  initial guess

    cost = @(p) sum((mf(p,x) - y).^2);

    options = optimset('Display','off', ...
                       'MaxIter',5000, ...
                       'MaxFunEvals',10000);

    params = fminsearch(cost, p0, options);
end


% ------------- Step 8: Build master dataset ------------
% Combine all normalized curves into one dataset
% Enforce symmetry by mirroring


all_X = [];
all_Y = [];

for i = 1:length(normalized)

    x = normalized(i).SA(:);
    y = normalized(i).Fy(:);

    if max(abs(y)) < 0.7
        continue;  % reject weak sweeps
    end

    % Mirror to full S-shape
    all_X = [all_X; -flipud(x); x];
    all_Y = [all_Y; -flipud(y); y];
end

valid = ~isnan(all_X) & ~isnan(all_Y);
all_X = all_X(valid);
all_Y = all_Y(valid);


% --------------- Step 9: Fit master magic formula ------
master_params = fit_magic_formula(all_X, all_Y);

B = master_params(1);
C = master_params(2);
E = master_params(3);

fprintf('\nMASTER MAGIC FORMULA PARAMETERS:\n');
fprintf('B = %.4f\n', B);
fprintf('C = %.4f\n', C);
fprintf('E = %.4f\n', E);

% -------------- Step 10: Validation Plot ---------
figure; hold on; grid on;

plot(all_X, all_Y, '.', 'Color',[0.7 0.7 0.7], 'MarkerSize', 2);

xfit = linspace(min(all_X), max(all_X), 600);
mf_eval = @(p,x) sin(p(2)*atan(p(1)*x - p(3)*(p(1)*x - atan(p(1)*x))));
yfit = mf_eval(master_params, xfit);

plot(xfit, yfit, 'r', 'LineWidth', 2.5);

xlabel('Normalized Slip  \alphā');
ylabel('Normalized Force  F_ȳ');
title('Master Normalized Pure Slip Magic Formula Fit');
legend('Normalized Data','MF Fit');


% ----------------- Support function ------------
function [Ca, SH, SV] = find_shift_params(SA, Fy, FZ)

    alpha = deg2rad(SA(:));
    Fy = Fy(:);

    % Restrict to near-linear region 
    Fy_pk = max(abs(Fy));
    mask = abs(Fy) < 0.3 * Fy_pk;

    if sum(mask) < 10
        Ca = NaN; SH = NaN; SV = NaN; return;
    end

    p = polyfit(alpha(mask), Fy(mask), 3);
    dp = polyder(p);
    ddp = polyder(dp);

    alpha_ms = -ddp(2)/ddp(1);      % max slope location
    Ca = polyval(dp, alpha_ms);     % stiffness

    Fy_ms = polyval(p, alpha_ms);
    SH = Fy_ms / abs(FZ);
    SV = (Ca * alpha_ms - Fy_ms) / abs(FZ);
end


% ---------- STEP 11: Extract data for response surfaces ----------


% Optional: normalize inputs for numerical stability
FZ0 = mean(FZ_vec);
P0  = mean(P_vec);

x1 = FZ_vec - FZ0;   % centered FZ
x2 = P_vec  - P0;    % centered P


% ---------- STEP 12: Design matrix (quadratic surface) ----------
Phi = [ ...
    ones(size(x1)), ...
    x1, ...
    x2, ...
    x1.^2, ...
    x2.^2, ...
    x1.*x2 ];

% Weight by vertical load
W = diag(abs(FZ_vec) / max(abs(FZ_vec)));

fit_surface = @(y) (Phi' * W * Phi) \ (Phi' * W * y);

beta_Ca = fit_surface(Ca_vec);
beta_mu = fit_surface(mu_vec);
beta_SH = fit_surface(SH_vec);
beta_SV = fit_surface(SV_vec);


Ca_fun = @(FZ,P) eval_surface(FZ,P,beta_Ca,FZ0,P0);
mu_fun = @(FZ,P) eval_surface(FZ,P,beta_mu,FZ0,P0);
SH_fun = @(FZ,P) eval_surface(FZ,P,beta_SH,FZ0,P0);
SV_fun = @(FZ,P) eval_surface(FZ,P,beta_SV,FZ0,P0);


% Helper function for evaluating the response surface
function y = eval_surface(FZ, P, beta, FZ0, P0)
    x1 = FZ - FZ0;
    x2 = P  - P0;

    Phi = [ ...
        ones(size(x1)), ...
        x1, ...
        x2, ...
        x1.^2, ...
        x2.^2, ...
        x1.*x2 ];

    y = Phi * beta;
end


FZ_test = 950;
P_test  = 11;

Ca_test = Ca_fun(FZ_test, P_test);
mu_test = mu_fun(FZ_test, P_test);
SH_test = SH_fun(FZ_test, P_test);
SV_test = SV_fun(FZ_test, P_test);

