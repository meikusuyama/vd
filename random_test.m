load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")

target = 100;
pres = 10;
ia = 2;

filter = abs(abs(FZ) - target) <= 10;
pres_filter = abs(abs(P) - pres) <= 0.5;
ia_filter = abs(abs(IA) - ia) <= 0.2;

time_filter = abs(ET - 1059) <= 20



figure;
%scatter(ET(time_filter), FY(time_filter));
plot(ET(filter & pres_filter & ia_filter), FY(filter & pres_filter & ia_filter))
%plot(SA(filter & pres_filter & ia_filter), FY(filter & pres_filter & ia_filter))