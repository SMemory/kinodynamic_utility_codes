function Ad_g = compute_adjoint(g)
% This function computes the Adjoint of any g belonging to SE(3).
% To compute the adjoint inverse, you have to feed in the inverse of the
% adjoint matrix as an argument.

R = g(1:3, 1:3);
p = g(1:3, 4);

Ad_g_top = [R skew_symm(p)*R];
Ad_g_bottom = [zeros(3,3) R];
Ad_g = [Ad_g_top; Ad_g_bottom];