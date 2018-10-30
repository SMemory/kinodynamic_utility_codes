function xi_hat = skew_symm_twist(xi)
% Function to compute the skew symmetric form of a 3 x 1 vector

w = xi(4:6);
v = xi(1:3);
w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
xi_hat = [w_hat v; zeros(1,4)];