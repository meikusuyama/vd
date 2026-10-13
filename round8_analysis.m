load("/Users/meikusuyama/Downloads/BFR_VD/RunData_Cornering_Matlab_USCS_10inch_Round8/A1965run15.mat")


figure;

filter = (FZ <= -245 & FZ >= -255)
filter2 = (P >= 9.5 & P <= 10.5)

x = SA(filter & filter2)
y = FY(filter & filter2)
plot(x, y)