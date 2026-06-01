function [Front_Loads,Rear_Loads,F_Geometry_Mat_Front,F_Geometry_Mat_Rear] = ...
Member_Loads(Front_Points,Rear_Points,Ftire_front,Mtire_front,...
Ftire_rear,Mtire_rear)
% Member_Loads Outputs Suspension Member Loads and Unit Vectors
%
% [Front_Loads,Rear_Loads,F_Geometry_Mat_Front,F_Geometry_Mat_Rear] = ...
% Member_Loads(Front_Points,Rear_Points,Ftire_front,Mtire_front,...
% Ftire_rear,Mtire_rear)
%
% Outputs suspension member loads and unit vectors from points, force and moment inputs.
%
% John D. Fratello
% Last Updated 3/10/08

T_front = [Ftire_front; Mtire_front];
T_rear = [Ftire_rear; Mtire_rear];
% Right Rear Points
e = Rear_Points(1:3,1);
f = Rear_Points(1:3,2);
d = Rear_Points(1:3,3);
b = Rear_Points(1:3,4);
c = Rear_Points(1:3,5);
a = Rear_Points(1:3,6);
h = Rear_Points(1:3,7);
g = Rear_Points(1:3,8);
s = Rear_Points(1:3,9);
t = Rear_Points(1:3,10);
rear_wc = Rear_Points(1:3,11);
% Right Front Points
j = Front_Points(1:3,1);
i = Front_Points(1:3,2);
k = Front_Points(1:3,3);
m = Front_Points(1:3,4);
l = Front_Points(1:3,5);
n = Front_Points(1:3,6);
p = Front_Points(1:3,7);
o= Front_Points(1:3,8);
q = Front_Points(1:3,9);
r = Front_Points(1:3,10);
front_wc = Front_Points(1:3,11);
%Magnitudes
OP = norm(p-o);
ML = norm(m-l);
MN = norm(m-n);
JI = norm(j-i);
JK = norm(j-k);
QR = norm(q-r);
BC = norm(b-c);
BA = norm(b-a);
EF = norm(e-f);
ED = norm(e-d);
HG = norm(h-g);
ST = norm(s-t);
%r-vectors (from wc to inboard point)
rp = p - front_wc;
rm = m - front_wc;
rq = q - front_wc;
rj = j - front_wc;
rb = b - rear_wc;
re = e - rear_wc;
rh = h - rear_wc;
rs = s - rear_wc;
%Find all the unit vectors (inboard - outboard)/magnitude
n_op = (o - p)./OP;
n_ml = (l - m)./ML;
n_mn = (n - m)./MN;
n_ji = (i - j)./JI;
n_jk = (k - j)./JK;
n_qr = (r - q)./QR;
n_bc = (c - b)./BC;
n_ba = (a - b)./BA;
n_ef = (f - e)./EF;
n_ed = (d - e)./ED;
n_hg = (g - h)./HG;
n_st = (t - s)./ST;
% Unit Moments
M_op = (cross(rp,n_op));
M_ml = (cross(rm,n_ml));
M_mn = (cross(rm,n_mn));
M_ji = (cross(rj,n_ji));
M_jk = (cross(rj,n_jk));
M_qr = (cross(rq,n_qr));
M_bc = (cross(rb,n_bc));
M_ba = (cross(rb,n_ba));
M_ef = (cross(re,n_ef));
M_ed = (cross(re,n_ed));
M_hg = (cross(rh,n_hg));
M_st = (cross(rs,n_st));
% Geometry matrices (unit vectors and unit moments)
F_Geometry_Mat_Front = [n_mn,n_ml,n_jk,n_ji,n_op,n_qr];
F_Geometry_Mat_Rear = [n_bc,n_ba,n_ef,n_ed,n_hg,n_st];
M_Geometry_Mat_Front = [M_mn,M_ml,M_jk,M_ji,M_op,M_qr]; 
M_Geometry_Mat_Rear = [M_bc,M_ba,M_ef,M_ed,M_hg,M_st];
Geometry_Mat_Front = [F_Geometry_Mat_Front ;
M_Geometry_Mat_Front];
Geometry_Mat_Rear = [F_Geometry_Mat_Rear ; M_Geometry_Mat_Rear];
% Solve for loads (Just solving a system of 6 equations in matrix form)
Front_Loads = (Geometry_Mat_Front)\(-T_front);
Rear_Loads = (Geometry_Mat_Rear)\(-T_rear); 