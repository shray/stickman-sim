function [ p_final ] = camera_project( point_3d, project_mat, occl_rad )
%CAMERA_PROJECT Project 3d points in camera's frame to image plane. Also,
%takes care of occlusions
    temp = project_mat* point_3d;
    point_2d = [temp(1,:)./temp(3,:); temp(2,:)./temp(3,:)];
    
    %go through 2d points to look for occlusions
    p_final = point_2d;
    for i=1:size(point_2d,2)-1
        for j=i+1:size(point_2d,2)
            if (norm(point_2d(:,i)-point_2d(:,j)) <= occl_rad)
                %check which point is ahead (in z component) and remove the other one
                if ((point_3d(3,i)/point_3d(4,i)) > (point_3d(3,j)/point_3d(4,j)) )
                    p_final(:,i) = [inf, inf]';
                elseif ((point_3d(3,i)/point_3d(4,i)) < (point_3d(3,j)/point_3d(4,j)) )
                    p_final(:,j) = [inf, inf]';
                else
                    strcat('Error. Two 3d-points coinciding! Indices: ', int2str(1), ',', int2str(2))
                    point_2d
                    point_3d
                end
            end
        end
    end

end

