load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

% ---INPUTS ---
% Mx  - overturning moment [lb-ft]
% FZ  - normal load (lb)
% IA  - inclination angle (camber) [deg]
% FZ  - vertical load [lb]

target_SA = 10.9065;  
tolerance = 0.5;    % ± degrees band around that SA

% --- FILTER DATA ---
mask = abs(SA - target_SA) < tolerance & ~isnan(MX);
Mx_f = MX(mask);
FZ_f = FZ(mask);
IA_f = IA(mask);


% --- DEFINE REGULAR GRID (FZ on X-axis, IA on Y-axis) ---
fz_range = linspace(min(FZ_f), max(FZ_f), 50);
ia_range = linspace(min(IA_f), max(IA_f), 50);
[FZ_grid, IA_grid] = meshgrid(fz_range, ia_range);

% --- INTERPOLATION ---
Finterp = scatteredInterpolant(FZ_f, IA_f, Mx_f, 'natural', 'none');
Mx_grid = Finterp(FZ_grid, IA_grid);

% --- PLOT SURFACE ---
figure('Color','w');
surf(FZ_grid, IA_grid, Mx_grid, ...
     'FaceColor','interp','EdgeColor','none');

xlabel('Vertical Load FZ (lb)','FontWeight','bold');
ylabel('Inclination Angle IA (deg)','FontWeight','bold');
zlabel('Overturning Moment Mx','FontWeight','bold');

title(sprintf('Mx Surface at SA = %.2f°', target_SA),'FontWeight','bold');
colorbar;
colormap(jet);
view([-45 25]);
grid on;
box on;