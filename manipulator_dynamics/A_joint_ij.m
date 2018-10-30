function A = A_joint_ij(xi_joints, theta)
% function to compute the 6 x 6 matrix A_ij for all values of i and j

num_of_joints = length(theta);
A = zeros(6,6,num_of_joints,num_of_joints);

G_temp = zeros(4,4,num_of_joints);
for i = 1:num_of_joints
    G_temp(:,:,i) = exp_twist(xi_joints(:,i),theta(i));
end

product_G = zeros(4,4,num_of_joints,num_of_joints);
for i = 2:num_of_joints
    for j = i-1:-1:1
        if i == (j+1)
            product_G(:,:,i,j) = G_temp(:,:,i);
        else
            product_G(:,:,i,j) = product_G(:,:,i,j+1)*product_G(:,:,j+1,j);
        end
    end
end
               
for i = 1:num_of_joints
    for j = 1:i
        if (i == j)
            A(:,:,i,j) = eye(6,6);
        else
            A(:,:,i,j) = compute_adjoint(product_G(:,:,i,j));
        end
    end
end
