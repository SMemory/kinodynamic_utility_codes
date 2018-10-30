function mass_r = compute_reflected_inertia(g_s_ell_0, mass_all_links_cg)
% Function to compute the reflected inertia matrix used in the dynamics
% computation. Note that this is not equivalent to expressing the 
% generalized inertia matrix of a link in the base frame.  

Dof = size(g_s_ell_0,3);
mass_r = zeros(6,6,Dof);
for i = 1:Dof
    g_inv = compute_ginv(g_s_ell_0(:,:,i));
    Ad_g_inv = compute_adjoint(g_inv);
    mass_r(:,:,i) = Ad_g_inv'*mass_all_links_cg(:,:,i)* Ad_g_inv;
end
    
