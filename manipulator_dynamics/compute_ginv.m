function g_inv = compute_ginv(g)
% Function to compute the inverse of g in SE(3). g is a 4x4 matrix

R = g(1:3,1:3);
p = g(1:3,4);
g_inv = [R' -R'*p; [0 0 0 1]];