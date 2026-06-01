load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")


%% ================= TEST LEVELS =================
P_levels  = [10 12 14];
IA_levels = [0 2 4];
FZ_levels = [-50 -100 -150 -200 -250];

P_tol  = 0.5;
IA_tol = 0.2;
FZ_tol = 10;

results = struct;
r = 0;

%% ================= MAIN LOOP =================
for p = P_levels
for ia = IA_levels
for fz = FZ_levels

    mask = abs(P - p) < P_tol & ...
           abs(IA - ia) < IA_tol & ...
           abs(FZ - fz) < FZ_tol;

    if sum(mask) < 10
        continue
    end

    ETg = ET(mask);
    SAg = SA(mask);
    FYg = FY(mask);
    FZg = FZ(mask);

    [ETg, idx] = sort(ETg);
    SAg = SAg(idx);
    FYg = FYg(idx);
    FZg = FZg(idx);

    %% ---------- Slip-rate Based SWEEP SPLITTING ----------

    dSA = gradient(SAg, ETg);
    
    % Smooth to remove noise
    dSAf = movmean(dSA, 15);
    
    sgn = sign(dSAf);
    
    % Remove near-zero junk
    sgn(abs(dSAf) < 0.01) = 0;
    
    % Fill zeros by nearest neighbor
    for i = 2:length(sgn)-1
        if sgn(i)==0
            sgn(i) = sgn(i-1);
        end
    end
    
    % Find sign changes
    edges = diff(sgn);
    
    cutPts = find(edges~=0);
    
    bounds = [1; cutPts+1; length(sgn)];
    
    sweeps = {};
    
    for i = 1:length(bounds)-1
    
        idx = bounds(i):bounds(i+1);
    
        if length(idx) < 40
            continue
        end
    
        sweeps{end+1}.idx = idx;
        sweeps{end}.sign = sign(mean(dSA(idx)));
    
    end

    % Require 3 sweeps (2+,1−)
    if length(sweeps) < 3
        fprintf('Skip: insufficient sweeps at P=%g IA=%g FZ=%g\n',p,ia,fz);
        continue
    end

    % ---------- If the SA sweep was done more than once for the same condition, only keep the first one----------

    if length(sweeps) > 3
    
        signs = cellfun(@(s) s.sign, sweeps);
    
        % Find first occurrence of [+ - +]
        firstTri = [];
    
        for i = 1:length(signs)-2
            if signs(i)==1 && signs(i+1)==-1 && signs(i+2)==1
                firstTri = i:i+2;
                break
            end
        end
    
        if ~isempty(firstTri)
            sweeps = sweeps(firstTri);
        else
            % fallback: just take first 3
            sweeps = sweeps(1:min(3,end));
        end
    
    end


    %% ---------- POLYNOMIAL FIT PER SWEEP ----------
    Cy = zeros(3,1);
    Sh = zeros(3,1);
    Sv = zeros(3,1);
    mu = zeros(3,1);
    
    for k = 1:3
        idx = sweeps{k}.idx;
        alpha = deg2rad(SAg(idx));
        Fy    = FYg(idx);
        Fzbar = abs(mean(FZg(idx))); % Use mean Fz of the specific sweep
        
        % 1. Smooth Fy to find TRUE peak (ignores noise spikes)
        Fy_smooth = movmean(Fy, 15);
        [~, ipk] = max(abs(Fy_smooth));
        alpha_pk = abs(alpha(ipk));
        
        % 2. Region = ±5% of slip at peak 
        linMask = abs(alpha) < 0.05 * alpha_pk;
        
        % Because 5% is very narrow, ensure we have at least a few points
        if sum(linMask) < 5
            % Fallback slightly wider if data is sparse near the origin
            linMask = abs(alpha) < 0.10 * alpha_pk; 
            if sum(linMask) < 5
                fprintf('Skip: insufficient points in linear region at P=%g IA=%g FZ=%g sweep %d\n',p,ia,fz,k);
                continue
            end
        end
        
        % 3. First order polynomial fit on the linear region: Fy = c*alpha + d
        p1 = polyfit(alpha(linMask), Fy(linMask), 1);
        c_coeff = p1(1); 
        d_coeff = p1(2);
        
        % 4. Find shift and stiffness parameters (Eq 30 - 33)
        % Angular Shift in radians: Sh = -d/c
        alpha_0 = -d_coeff / c_coeff; 
        Sh(k) = alpha_0;
        
        % Vertical Force Shift is neglected
        Sv(k) = 0;
        
        % Cornering Stiffness (Force/Radian) is the linear slope
        Cy(k) = c_coeff;
        
        % 5. Find Peak Friction using the SMOOTHED data
        % (Fy_hat is just Fy_smooth since Sv is 0)
        Fy_hat = Fy_smooth - Sv(k);
        mu(k) = max(abs(Fy_hat)) / Fzbar;
    end


    %% ---------- 2+/1− AVERAGING ----------
    Cy = mean([mean([Cy(1), Cy(3)]), Cy(2)]);
    Sh_avg = mean([mean([Sh(1), Sh(3)]), Sh(2)]);
    Sv_avg = mean([mean([Sv(1), Sv(3)]), Sv(2)]);
    mu_avg = mean([mean([mu(1), mu(3)]), mu(2)]);
    
    
    %% ---------- NORMALIZATION ----------
    Fzbar = abs(mean(FZg));
    alpha = deg2rad(SAg);

    SA_n = ((Cy/Fzbar)/mu_avg)* tan(alpha - (Sh_avg/Fzbar)/(Cy/Fzbar));

    FY_n = (FYg/Fzbar) / mu_avg;


    %% ---------- STORE ----------
    r = r + 1;
    results(r).P  = p;
    results(r).IA = ia;
    results(r).FZ = fz;
    results(r).SA_n = SA_n;
    results(r).FY_n = FY_n;

    % PHYSICAL parameters (NOT normalized)
    results(r).Cy = Cy;
    results(r).Sh = Sh_avg;
    results(r).Sv = Sv_avg;
    results(r).mu = mu_avg;

end
end
end


%% ================= PLOT FIRST GROUP =================
P_target  = 10;
IA_target = 2;
FZ_target = -150;

idx = find([results.P]==P_target & ...
           [results.IA]==IA_target & ...
           [results.FZ]==FZ_target, 1);

if isempty(idx)
    error('Requested group (P=%g, IA=%g, FZ=%g) was not found.', ...
           P_target, IA_target, FZ_target);
end

x = results(idx).SA_n(:);
y = results(idx).FY_n(:);

valid = isfinite(x) & isfinite(y);

if sum(valid) < 10
    error('Normalized data exists but is empty or invalid for P=%g, IA=%g, FZ=%g.', ...
           P_target, IA_target, FZ_target);
end

figure; hold on; grid on;
plot(x(valid), y(valid), '.', 'MarkerSize', 6);
xlabel('Normalized Slip Angle');
ylabel('Normalized Lateral Force');
title(sprintf('Normalized FY vs SA (P=%g, IA=%g, FZ=%g)', ...
      P_target, IA_target, FZ_target));



% ===== MAGIC FORMULA FIT =====

xn = x(valid);
yn = y(valid);


% Split positive / negative slip
posMask = xn >= 0;
negMask = xn <  0;

xPos = xn(posMask);
yPos = yn(posMask);

xNeg = xn(negMask);
yNeg = yn(negMask);

% Simplified Magic Formula
magicFun = @(p,x) ...
    p(1) .* sin( ...
    p(2) .* atan( ...
    p(3).*x - p(4).*(p(3).*x - atan(p(3).*x)) ...
    ));

errFun = @(p,x,y) sum( (y - magicFun(p,x)).^2 );

% Initial Guess
D0 = max(abs(yn));          % peak
C0 = 1.3;                  % typical
B0 = 5 / max(abs(xn));     % stiffness
E0 = 0.5;

p0 = [D0 C0 B0 E0];

options = optimset( ...
    'MaxFunEvals', 1e5, ...
    'MaxIter',     1e5, ...
    'Display',     'off');

% Positive side fit
pPos = fminsearch(@(p) errFun(p,xPos,yPos), p0, options);

% Negative side fit
pNeg = fminsearch(@(p) errFun(p,xNeg,yNeg), p0, options);

xFit = linspace(min(xn), max(xn), 400);

yFitPos = magicFun(pPos, xFit(xFit>=0));
yFitNeg = magicFun(pNeg, xFit(xFit<0));

plot(xFit(xFit>=0), yFitPos, 'r-', 'LineWidth',2);
plot(xFit(xFit<0),  yFitNeg, 'r-', 'LineWidth',2);

legend('Data','Magic Fit','Location','Best');



%===================== Response Surface ================
% RESPONSE SURFACE FITTING FOR Cy, mu, Sh, Sv
% Inputs  : FZ, IA
% Outputs : Continuous TTC parameters
% Method  : Weighted least squares (per paper)
%% =============================================================

%% ---------- STEP 1: Extract data from results struct ----------
FZ_vec = arrayfun(@(s) s.FZ, results).';
IA_vec = arrayfun(@(s) s.IA, results).';

Cy_vec = arrayfun(@(s) s.Cy, results).';
mu_vec = arrayfun(@(s) s.mu, results).';
Sh_vec = arrayfun(@(s) s.Sh, results).';
Sv_vec = arrayfun(@(s) s.Sv, results).';

% Remove invalid entries (safety)
valid = isfinite(FZ_vec) & isfinite(IA_vec) & ...
        isfinite(Cy_vec) & isfinite(mu_vec) & ...
        isfinite(Sh_vec) & isfinite(Sv_vec);

FZ_vec = FZ_vec(valid);
IA_vec = IA_vec(valid);
Cy_vec = Cy_vec(valid);
mu_vec = mu_vec(valid);
Sh_vec = Sh_vec(valid);
Sv_vec = Sv_vec(valid);

%% ---------- STEP 2: Build polynomial design matrix ----------
% Quadratic surface:
% c = p0 + p1*FZ + p2*IA + p3*FZ^2 + p4*IA^2 + p5*FZ*IA

X = [ ...
    ones(size(FZ_vec)), ...
    FZ_vec, ...
    IA_vec, ...
    FZ_vec.^2, ...
    IA_vec.^2, ...
    FZ_vec .* IA_vec ];

%% ---------- STEP 3: Choose weighting (vertical force) ----------
% Per paper: weight by vertical load to minimize absolute error

W = abs(FZ_vec);
W = W / max(W);          % normalize for conditioning
Wmat = diag(W);

%% ---------- STEP 4: Weighted least squares fits ----------
% Existing fits
beta_Cy = (X' * Wmat * X) \ (X' * Wmat * Cy_vec);
beta_mu = (X' * Wmat * X) \ (X' * Wmat * mu_vec);
beta_Sh = (X' * Wmat * X) \ (X' * Wmat * Sh_vec);
beta_Sv = (X' * Wmat * X) \ (X' * Wmat * Sv_vec);

% NEW: Calculate Normalized Cornering Stiffness (Cy / |Fz|) and fit
Cy_bar_vec = Cy_vec ./ abs(FZ_vec); 
beta_Cy_bar = (X' * Wmat * X) \ (X' * Wmat * Cy_bar_vec);

%% ---------- STEP 5: Store model coefficients ----------
responseSurface.Cy = beta_Cy;
responseSurface.mu = beta_mu;
responseSurface.Sh = beta_Sh;
responseSurface.Sv = beta_Sv;
responseSurface.Cy_bar = beta_Cy_bar; % Store new coefficients

%% ---------- STEP 6: Create callable response surface functions ----------
Cy_fun = @(FZ,IA) beta_Cy(1) + beta_Cy(2)*FZ + beta_Cy(3)*IA + ...
                  beta_Cy(4)*FZ.^2 + beta_Cy(5)*IA.^2 + beta_Cy(6)*FZ.*IA;

% NEW: Function for Normalized Cornering Stiffness
Cy_bar_fun = @(FZ,IA) beta_Cy_bar(1) + beta_Cy_bar(2)*FZ + beta_Cy_bar(3)*IA + ...
                      beta_Cy_bar(4)*FZ.^2 + beta_Cy_bar(5)*IA.^2 + beta_Cy_bar(6)*FZ.*IA;

%% ---------- STEP 7: Visualization (paper-style plots) ----------
[FZg, IAg] = meshgrid( ...
    linspace(min(FZ_vec), max(FZ_vec), 30), ...
    linspace(min(IA_vec), max(IA_vec), 30));

Cy_surf = Cy_fun(FZg, IAg);
Cy_bar_surf = Cy_bar_fun(FZg, IAg); % Evaluate new surface

figure('Name', 'Stiffness Response Surfaces', 'Position', [100, 100, 1000, 400]);

% Plot 1: Absolute Cornering Stiffness
subplot(1,2,1)
surf(FZg, IAg, Cy_surf, 'FaceAlpha', 0.8, 'EdgeColor', 'none'); hold on;
scatter3(FZ_vec, IA_vec, Cy_vec, 40, 'r', 'filled')
xlabel('Vertical Load (FZ)'); 
ylabel('Inclination Angle (IA)'); 
zlabel('C_y (Force/Rad)');
title('Cornering Stiffness (C_y)');
grid on; view(-45, 30); colormap(parula);

% Plot 2: Normalized Cornering Stiffness (Efficiency)
subplot(1,2,2)
surf(FZg, IAg, Cy_bar_surf, 'FaceAlpha', 0.8, 'EdgeColor', 'none'); hold on;
scatter3(FZ_vec, IA_vec, Cy_bar_vec, 40, 'r', 'filled')
xlabel('Vertical Load (FZ)'); 
ylabel('Inclination Angle (IA)'); 
zlabel('C_y / F_z (1/Rad)');
title('Normalized Cornering Stiffness (C_y / F_z)');
grid on; view(-45, 30); colormap(parula);




%% ================= MASTER MAGIC FORMULA FIT =================
% 1. Gather ALL normalized data from the results struct to create the Master Cloud
all_X = [];
all_Y = [];
for i = 1:length(results)
    all_X = [all_X; results(i).SA_n(:)];
    all_Y = [all_Y; results(i).FY_n(:)];
end

% 2. Clean the data (remove NaNs and Infs)
valid_mask = isfinite(all_X) & isfinite(all_Y);
all_X = all_X(valid_mask);
all_Y = all_Y(valid_mask);

% 3. Define the Nondimensional Magic Formula
% Since the data is normalized: Peak (D) = 1, and Initial Slope = 1 (so C = 1/B)
% p(1) is B (Stiffness), p(2) is E (Curvature)
mf_master = @(p, x) sin((1/p(1)) * atan(p(1)*x - p(2)*(p(1)*x - atan(p(1)*x))));

% 4. Optimize the fit
p0_master = [10, -1]; % Initial guess: B=10, E=-1
cost_fun = @(p) sum((mf_master(p, all_X) - all_Y).^2);
options = optimset('Display','off');
best_p = fminsearch(cost_fun, p0_master, options);

% 5. DEFINE THE MISSING VARIABLES
B_master = best_p(1);
C_master = 1 / best_p(1);
E_master = best_p(2);

fprintf('\nMaster MF Fit Complete:\n B = %.3f\n C = %.3f\n E = %.3f\n', ...
        B_master, C_master, E_master);

% 6. Now you can safely package them for the calculate_physical_Fy function
masterMF.B = B_master;
masterMF.C = C_master;
masterMF.E = E_master;






%==================== Fy calculator function using the normalized model =======================
function Fy = calculate_physical_Fy(FZ, IA, SA_deg, responseSurface, masterMF)
    % calculate_physical_Fy: Converts physical inputs to physical lateral force
    % using a Nondimensional Tire Model.
    %
    % Inputs:
    %   FZ : Vertical Load (matching your training data sign, e.g., -50)
    %   IA : Inclination Angle (degrees)
    %   SA_deg : Slip Angle (degrees)
    %   responseSurface : Struct containing beta coefficients (Cy, mu, Sh, Sv)
    %   masterMF : Struct containing Master MF parameters (B, C, E)
    %
    % Output:
    %   Fy : Physical lateral force

    %% 1. Evaluate the Response Surfaces for the given FZ and IA
    % Create the polynomial input vector: [1, FZ, IA, FZ^2, IA^2, FZ*IA]
    X_eval = [1, FZ, IA, FZ^2, IA^2, FZ*IA];
    
    % Predict the 4 key tire parameters
    Cy_val = sum(X_eval .* responseSurface.Cy'); % Cornering Stiffness [Force/Rad]
    mu_val = sum(X_eval .* responseSurface.mu'); % Friction Coefficient [unitless]
    Sh_val = sum(X_eval .* responseSurface.Sh'); % Angular Shift [Radians]
    Sv_val = sum(X_eval .* responseSurface.Sv'); % Vertical Shift Ratio [Fy/Fz]
    
    %% 2. Convert to Nondimensional Inputs
    alpha_rad = deg2rad(SA_deg);
    Fz_mag = abs(FZ);
    
    % Calculate normalized cornering stiffness
    Cy_bar = Cy_val / Fz_mag;
    
    % Calculate Normalized Slip Variable (alpha_bar)
    % This handles the horizontal shift and scales the x-axis
    alpha_bar = (Cy_bar / mu_val) * tan(alpha_rad - Sh_val);
    
    %% 3. Evaluate the Master Magic Formula
    B = masterMF.B;
    C = masterMF.C;
    E = masterMF.E;
    
    % Calculate Normalized Lateral Force (Fy_bar)
    % Equation: sin(C * atan(B*x - E*(B*x - atan(B*x))))
    Fy_bar = sin(C * atan(B * alpha_bar - E * (B * alpha_bar - atan(B * alpha_bar))));
    
    %% 4. Un-Normalize back to Physical Force
    % Reverse the Fy_bar equation: Fy_bar = 1/mu * (Fy/Fz - Sv_bar)
    % Solved for Fy:
    Fy = Fz_mag * (Fy_bar * mu_val + Sv_val);

end


% ==================== Validation Main ===================
% 1. Package your Master Formula parameters
masterMF.B = B_master;
masterMF.C = C_master;
masterMF.E = E_master;

% 2. Define a test condition
test_FZ = -100;
test_IA = 0;
test_P  = 10; % <-- Define a specific pressure to filter the raw data

% 3. Create a range of physical slip angles to test
test_SA = linspace(-12, 12, 100); 
test_Fy = zeros(size(test_SA));

% 4. Run the model!
for i = 1:length(test_SA)
    test_Fy(i) = calculate_physical_Fy(test_FZ, test_IA, test_SA(i), responseSurface, masterMF);
end

% 5. Extract the ACTUAL RAW DATA for this condition from TTC

mask = abs(FZ - test_FZ) < FZ_tol & ...
       abs(IA - test_IA) < IA_tol & ...
       abs(P - test_P)   < P_tol;

actual_SA = SA(mask);
actual_FY = FY(mask);

% 6. Plot the physical prediction vs actual data
figure; hold on; grid on;

% Plot actual TTC data as gray/black scatter dots
plot(actual_SA, actual_FY, '.', 'Color', [0.5 0.5 0.5], 'MarkerSize', 6, 'DisplayName', 'Actual TTC Data');

% Plot the Nondimensional Model Prediction as a solid red line
plot(test_SA, test_Fy, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Model Prediction');

xlabel('Slip Angle (deg)');
ylabel('Lateral Force (lbs or N)');
title(sprintf('Model Validation (Fz = %g, IA = %g, P = %g)', test_FZ, test_IA, test_P));
legend('Location', 'best');

