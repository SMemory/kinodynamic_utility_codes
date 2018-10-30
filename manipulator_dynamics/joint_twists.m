function xi_joints = joint_twists(type_joint, joint_axes, q_axes)
% Function to compute the joint twists for a manipulator in the base
% configuration.

num_of_joints = length(type_joint);

xi_joints = zeros(6, num_of_joints);

for i = 1:num_of_joints
    if strcmp(type_joint(i), 'R')
        omega = joint_axes(:,i);
        q = q_axes(:,i);
        xi_joints(:,i) = [cross(-omega, q); omega];
    end
    if strcmp(type_joint,'P')
        omega = zeros(3,1);
        v = joint_axes(:,i);
        xi_joints(:,i) = [v; omega];
    end
end