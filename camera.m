classdef camera
    %CAMERA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        camera_mat
        world_pos
        world_rot
    end
    
    methods
        function point_2d = project(point_3d)
            project_mat = camera_mat * [[camera_rot; zeros(1,3)] [world_to_cam_pos; 1]];
            temp = project_mat* point_3d;
            point_2d = [temp(1)/temp(3) temp(2)/temp(3)];
        end
    end
    
end

