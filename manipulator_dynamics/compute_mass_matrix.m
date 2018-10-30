function [M, C] = compute_mass_matrix(mass_all_links_s, xi_joints, theta, thetadot)
% Function to compute the mass matrix and coriollis matrix of an n-DoF robot manipulator at a
% given configuration

num_of_joints = length(theta);

A = A_joint_ij(xi_joints, theta);

M = zeros(num_of_joints, num_of_joints);
C = zeros(num_of_joints, num_of_joints);

% Compute the mass matrix
for i = 1:num_of_joints
    for j = 1:num_of_joints
        t1 = max(i,j);
        for u = t1:num_of_joints
            M(i,j) = M(i,j) + xi_joints(:,i)'*A(:,:,u,i)'*mass_all_links_s(:,:,u)*A(:,:,u,j)*xi_joints(:,j);
        end
    end
end

% Compute the partial derivatives of mass matrix
partial_matrix = zeros(num_of_joints,num_of_joints,num_of_joints);

for i = 1:num_of_joints
    for j = 1:num_of_joints
        for k = 1:num_of_joints
            t1 = max(i,j);
            for u = t1:num_of_joints
                if (k > 1)
                    %temp1 = A(:,:,k-1,i)*xi_joints(:,i)
                    %xi_joints(:,k)
                xi_temp1 = lie_bracket_twists(A(:,:,k-1,i)*xi_joints(:,i), xi_joints(:,k));
                xi_temp2 = lie_bracket_twists(A(:,:,k-1,j)*xi_joints(:,j), xi_joints(:,k));
                partial_matrix(i,j,k) = partial_matrix(i,j,k) + ...
                    xi_temp1'*A(:,:,u,k)'*mass_all_links_s(:,:,u)*A(:,:,u,j)*xi_joints(:,j) + ...
                    xi_joints(:,i)'*A(:,:,u,i)'*mass_all_links_s(:,:,u)*A(:,:,u,k)*xi_temp2;
                end
            end
        end
    end
end

% Compute the Coriolis matrix
for i = 1:num_of_joints
    for j = 1:num_of_joints
        for k = 1:num_of_joints
            C(i,j) = C(i,j) + (partial_matrix(i,j,k) + partial_matrix(i,k,j) - partial_matrix(k,j,i))*thetadot(k);
        end
        C(i,j) = C(i,j)/2;
    end
end

