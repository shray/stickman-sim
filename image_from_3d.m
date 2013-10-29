function [ image_pts, debug_intersect ] = image_from_3d( f, camera_rot, camera_to_world, care_points, OCCLUDE_RADIUS, OCCLUDE_WIDTH, intersection_pt )
%IMAGE_FROM_3D Project 3D points to image plane.
% No pixel discretization
% Takes care of occlusion problems

camera_mat = [f 0 0 0;
              0 f 0 0;
              0 0 1 0];        
                                                    
transform_cam_frame = [[camera_rot; zeros(1,3)] [camera_to_world; 1]];

%project point to camera space
point_cam_3d = transform_cam_frame * care_points;
intersection_3d = transform_cam_frame * intersection_pt;

%unhomogenized 3d points
points_3d_nh = [point_cam_3d(1,:)./point_cam_3d(4,:); ...
    point_cam_3d(2,:)./point_cam_3d(4,:); point_cam_3d(3,:)./point_cam_3d(4,:)];
intersection_3d_nh = [intersection_3d(1); intersection_3d(2); intersection_3d(3)]./intersection_3d(4);

project_mat_cam = camera_mat * eye(size(transform_cam_frame));
 %= camera_project(point_cam_3d, project_mat_cam, OCCLUDE_RADIUS, OCCLUDE_WIDTH);

%project to 2d
temp = project_mat_cam * point_cam_3d;
point_2d = [temp(1,:)./temp(3,:); temp(2,:)./temp(3,:)];%unhomogenize

intersect_2d = project_mat_cam * intersection_3d;
intersect_2d = [intersect_2d(1,:)./intersect_2d(3,:); intersect_2d(2,:)./intersect_2d(3,:)];%unhomogenize

%go through 2d points to look for occlusions
p_final = point_2d;

%Remove points occluded by other points
for i=1:size(point_2d,2)-1
    for j=i+1:size(point_2d,2)
        %check if i falls in j's disk
        if ((norm(point_2d(:,i)-point_2d(:,j)) <= OCCLUDE_RADIUS/points_3d_nh(3,j))...
                && (points_3d_nh(3,i) > points_3d_nh(3,j)))
            p_final(:,i) = [inf, inf]';
        %check if j falls in i's disk
        elseif ((norm(point_2d(:,i)-point_2d(:,j)) <= OCCLUDE_RADIUS/points_3d_nh(3,i))...
                && (points_3d_nh(3,j) > points_3d_nh(3,i)))        
            p_final(:,j) = [inf, inf]';
        end
    end
end

%remove points not already removed occluded by the cylinders 
%i.e. the stick or arms

%TODO: Remove approximate depth checking

%line-stick
%check if perpendicular of point on line-segment
for i=3:size(point_2d,2)
    if (sum(isinf(p_final(:,i)))<1)
        is_occluded = line_pt(point_2d(:,1),point_2d(:,2), point_2d(:,i), OCCLUDE_WIDTH,...
            (points_3d_nh(3,1)+points_3d_nh(3,1))/2, points_3d_nh(3,i));
        if (is_occluded)
            p_final(:,i) = [inf, inf]';
        end
    end
end



%line-arm1
%check if perpendicular of point on line-segment
for i=[1,2,4]
    if (sum(isinf(p_final(:,i)))<1)
        is_occluded = line_pt(point_2d(:,3),intersect_2d, point_2d(:,i), OCCLUDE_WIDTH,...
            (points_3d_nh(3,3)+intersection_3d_nh(3))/2, points_3d_nh(3,i));
        if (is_occluded)
            p_final(:,i) = [inf, inf]';
        end
    end
end


%line-arm2
%check if perpendicular of point on line-segment
for i=[1,2,3]
    if (sum(isinf(p_final(:,i)))<1)
        is_occluded = line_pt(point_2d(:,4),intersect_2d, point_2d(:,i), OCCLUDE_WIDTH,...
            (points_3d_nh(3,4)+intersection_3d_nh(3))/2, points_3d_nh(3,i));
        if (is_occluded)
            p_final(:,i) = [inf, inf]';
        end
    end
end

image_pts = p_final;
debug_intersect = intersect_2d;

end

