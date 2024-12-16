M = 1;
p = linspace(0,1,10);
l = 1;
dq = 1;
q = linspace(0,pi,10);
g = 1;
dtheta = 1;

[P,Q] = meshgrid(p,q);
Fr = (P .* M + 1/3 .* M .* (1 - P)) * l * dtheta.^2 - M * g .* sin(Q);
surf(P,Q,Fr)

