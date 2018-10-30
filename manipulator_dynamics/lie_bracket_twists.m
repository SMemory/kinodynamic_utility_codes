function xi = lie_bracket_twists(xi1, xi2)
% Function to compute the Lie brackets between two twists.

xi1_hat = skew_symm_twist(xi1);
xi2_hat = skew_symm_twist(xi2);
xi_hat = xi1_hat*xi2_hat - xi2_hat*xi1_hat;

w = [xi_hat(3,2);xi_hat(1,3);xi_hat(2,1)];
v = xi_hat(1:3,4);

xi = [w;v];