close all
clear

X = dlmread("data/states.txt");
Xr = dlmread("../opam/data/states.txt");
xa = X(:,1); ya = X(:,2); za = X(:,3);
vxa = X(:,4); vya = X(:,5); za = X(:,6);
axa = X(:,7);
q1a = X(:,15); q2a = X(:,16);


xr = Xr(:,1); yr = Xr(:,2); zr = Xr(:,3);
axr = Xr(:,7); ayr = Xr(:,8); axz = Xr(:,9);
vxr = Xr(:,4); vyr = Xr(:,5); zr = Xr(:,6);

q1r = Xr(:,15); q2r = Xr(:,16);

t = 0:0.1:1;
plot(t, axa, 'r')
hold on
plot(t, axr(1:size(xa,1)), 'b')