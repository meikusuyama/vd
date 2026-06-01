load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

% ---INPUTS ---
% Mx  - overturning moment [lb-ft]
% SA  - slip angle [deg]
% IA  - inclination angle (camber) [deg]
% FZ  - vertical load [lb]


target_Fz = -200;   % nominal FZ
tolerance = 10;    % ± lb band around that Fz

% --- FILTER DATA ---
mask = abs(FZ - target_Fz) < tolerance & ~isnan(MX);

Mx_f = MX(mask);
SA_f = SA(mask);
IA_f = IA(mask);



% --- INTERPOLATE TO REGULAR GRID ---
sa_range = linspace(min(SA_f), max(SA_f), 50);
ia_range = linspace(min(IA_f), max(IA_f), 50);
[SA_grid, IA_grid] = meshgrid(sa_range, ia_range);

Finterp = scatteredInterpolant(SA_f, IA_f, Mx_f, 'natural', 'none');
Mx_grid = Finterp(SA_grid, IA_grid);

% --- PLOT ---
figure('Color','w');
surf(IA_grid, SA_grid, Mx_grid, 'FaceColor','interp','EdgeColor','none');
xlabel('Inclination Angle (deg)','FontWeight','bold');
ylabel('Slip Angle (deg)','FontWeight','bold');
zlabel('Overturning Moment (lb-ft)','FontWeight','bold');
title(sprintf('Mx Surface at Fz = %.0f lb', target_Fz),'FontWeight','bold');
colorbar;
colormap(jet);
view([-45 25]); % similar viewing angle to BillCobb's plot
grid on;
box on;
