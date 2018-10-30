function w_hat = skew_symm(w)
% Function to compute the skew symmetric form of a 3 x 1 vector

w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
