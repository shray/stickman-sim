figure,clf 
%plot stick
plot3(world_stick_ends(1,:), world_stick_ends(2,:), world_stick_ends(3,:),'k*')
hold on
plot3(world_arm_starts(1,:), world_arm_starts(2,:), world_arm_starts(3,:),'g*')
plot3(world_arm_ends(1,:), world_arm_ends(2,:), world_arm_ends(3,:),'b*')

%plot camera
transform_w_to_c = [[camera_rot; zeros(1,3)] [cam_to_world; 1]];
transform_c_to_w = inv(transform_w_to_c);
cam_center = transform_c_to_w * [0 0 0 1]';
cam_center = cam_center(1:3)./cam_center(4);
cam_x = transform_c_to_w * [1 0 0 1]';
cam_y = transform_c_to_w * [0 1 0 1]';
cam_z = transform_c_to_w * [0 0 1 1]';
cam_x = cam_x(1:3)./cam_x(4);
cam_y = cam_y(1:3)./cam_y(4);
cam_z = cam_z(1:3)./cam_z(4);

%plot axes
plot3([cam_center(1),cam_x(1)], [cam_center(2),cam_x(2)], [cam_center(3),cam_x(3)], 'r-')
plot3([cam_center(1),cam_y(1)], [cam_center(2),cam_y(2)], [cam_center(3),cam_y(3)], 'g-')
plot3([cam_center(1),cam_z(1)], [cam_center(2),cam_z(2)], [cam_center(3),cam_z(3)], 'b-')