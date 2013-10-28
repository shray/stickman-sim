figure,clf 
%plot stick
plot3([end_p_1(1) end_p_2(1)], [end_p_1(2) end_p_2(2)], [end_p_1(3) end_p_2(3)],'k*')
hold on
plot3(intr_pt(1), intr_pt(2), intr_pt(3),'r*')
plot3(a1_pt(1,1), a1_pt(2,1), a1_pt(3,1),'g*')
plot3(a1_pt(1,2), a1_pt(2,2), a1_pt(3,2),'b*')

plot3(a2_pt(1,1), a2_pt(2,1), a2_pt(3,1),'g*')
plot3(a2_pt(1,2), a2_pt(2,2), a2_pt(3,2),'b*')

% %plot camera
% transform_w_to_c = [[camera_rot; zeros(1,3)] [cam_to_world; 1]];
% transform_c_to_w = inv(transform_w_to_c);
% cam_center = transform_c_to_w * [0 0 0 1]';
% cam_center = cam_center(1:3)./cam_center(4);
% cam_x = transform_c_to_w * [1 0 0 1]';
% cam_y = transform_c_to_w * [0 1 0 1]';
% cam_z = transform_c_to_w * [0 0 1 1]';
% cam_x = cam_x(1:3)./cam_x(4);
% cam_y = cam_y(1:3)./cam_y(4);
% cam_z = cam_z(1:3)./cam_z(4);
% 
%plot axes
plot3([0,1], [0,0], [0,0], 'r-')
plot3([0,0], [0,1], [0,0], 'g-')
plot3([0,0], [0,0], [0,1], 'b-')