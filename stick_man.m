%%  Simulate the -- Stickman
clear
close all

%initialize RNG
rng(1,'twister');

%constants
STICK_LEN = 5; %length in axis of rotation
STICK_RATIOS = [.5, .5]; ARM_RATIOS =[ .2, .2]; % ratio to len (arms-intersection, armr, arml)

%Occlusion caused by following - scaled by distance from camera
OCCLUDE_RADIUS = 0.000; %Only for the ends
OCCLUDE_WIDTH = 0.0; %For the arms and the stick - considering cylinder

start_arms_angles = [45, -45]; %degrees

vel_stick_rotation = [5 0]; %(mean, std) deg/frame

arm_rot = [0 5; 0 5]; %(mean,std) gauss to sample rotation speed
no_arms = 2; %number of arms

translation_s_w = [0 -5 0]'; %translation from stick origin to world in world coor

%Camera-1 constants
f1 = 0.005;
camera1_mat = [f1 0 0 0;
              0 f1 0 0;
              0 0 1 0];
%about x-axis
camera1_rot  = [1    0       0
               0 cosd(90) -sind(90)
               0 sind(90)  cosd(90)];           
                                                    
cam1_to_world = [0 0 0]';


%Camera-2 constants
f2 = 0.005;
camera2_mat = [f2 0 0 0;
              0 f2 0 0;
              0 0 1 0];
           
%about y-axis and z
camera2_rot  = [cosd(90) sind(90) 0;
              -sind(90) cosd(90) 0;
               0         0       1] * ...
              [cosd(90)     0          -sind(90)
               0            1           0
               sind(90)     0           cosd(90)];
                        
cam2_to_world = [5 -3 5]';



% points that are projected are two ends of the stick
% state of stick - apart from constants
% assuming a coordinate axis with origin at end 1 and z direction pointing
% pointing towards end1 and x in arm-left

%initialiaze stick-man
theta = start_arms_angles'; %angle between each of the arms and stick
stick_ends = [0 0 0; 0 0 STICK_LEN]'; %3d pos of stick in own frame
phi = 0; %angle of the stick's xy plane to that of world
arm_starts = zeros(3, no_arms);
arm_ends = zeros(3, no_arms);
for i =1:no_arms
    arm_starts(:,i) = (stick_ends(:,2)-stick_ends(:,1)).*STICK_RATIOS(i);
    arm_ends(:,i) =  arm_starts(:,i) + (STICK_LEN*ARM_RATIOS(i)) ...
                        .* [cosd(90-theta(i)) 0 sind(90-theta(i))]';
end


%Tracking initials
lengths = STICK_LEN * [1; STICK_RATIOS(1); ARM_RATIOS(1)];
chosen_cam = 1;

total_frame=100;

%start STICK-MAN dynamics
for cur_frame=0:total_frame
   
    [phi, phi_rot, theta, stick_ends, arm_starts, arm_ends] = ...
        stick_dynamics(phi, vel_stick_rotation, arm_rot, STICK_LEN, ARM_RATIOS, STICK_RATIOS, theta, stick_ends);

           
    %change to world frame
    world_stick_ends = phi_rot*stick_ends + repmat(translation_s_w,1,no_arms);
    world_arm_starts = phi_rot*arm_starts + repmat(translation_s_w,1,no_arms);
    world_arm_ends = phi_rot*arm_ends + repmat(translation_s_w,1,no_arms);

    
    %camera views
    
    %only care about the image of these 3d homogenous points
    care_points = [world_stick_ends world_arm_ends;...
                   ones(1,size(world_stick_ends,2)+...
                   size(world_arm_ends,2))];

    [camera1_view, intersect1] = image_from_3d( f1, camera1_rot, cam1_to_world, care_points,...
                            OCCLUDE_RADIUS, OCCLUDE_WIDTH, [world_arm_starts(:,1);1] );

    
    [camera2_view, intersect2] = image_from_3d( f2, camera2_rot, cam2_to_world, care_points,...
                            OCCLUDE_RADIUS, OCCLUDE_WIDTH, [world_arm_starts(:,1);1] );
                
                        
    %Camera visualizations - Nice!

    %draw - orthographic projection
    figure(1)    
    clf
    
    subplot(3,1,1)
    
    xlim([-3 3]);
    ylim([-1 7]);
    line([world_stick_ends(1,1);world_stick_ends(1,2)],...
          [world_stick_ends(3,1);world_stick_ends(3,2)]);
    color_name=[];
    for i=1:no_arms
        hold on
        if mod(i,2)
        color_name='r';
        else
        color_name='g';
        end
    
        line([[world_arm_starts(1,i)]'; [world_arm_ends(1,i)]'], ...
            [[world_arm_starts(3,i)]'; [world_arm_ends(3,i)]'], 'Color', color_name);
    
    end   
    
    %draw camera view
    subplot(3,1,2)

    xlim([-.0035 .0035]);
    ylim([-.001 .006]);
    %stick
    if (sum(isinf(camera1_view(:,1:2)))<1)
        line([camera1_view(1,1);camera1_view(1,2)], ...
            [camera1_view(2,1);camera1_view(2,2)], 'Color', 'k');
    end
    hold on
    %arms
    if (sum(isinf(camera1_view(:,3)))<1)
        line([camera1_view(1,3); intersect1(1)],...
             [camera1_view(2,3); intersect1(2)], 'Color', 'r');
    end
    if (sum(isinf(camera1_view(:,4)))<1)
        line([camera1_view(1,4); intersect1(1)],...
             [camera1_view(2,4); intersect1(2)], 'Color', 'g');         
    end
    plot(camera1_view(1,:), camera1_view(2,:), '.b');
    
    
    subplot(3,1,3)
    xlim([-.0035 .0035]);
    ylim([-.0035 .0035]);
    %stick
    if (sum(isinf(camera2_view(:,1:2)))<1)
        line([camera2_view(1,1);camera2_view(1,2)], ...
            [camera2_view(2,1);camera2_view(2,2)], 'Color', 'k');
    end
    hold on
    %arms
    if(sum(isinf(camera2_view(:,3)))<1)
        line([camera2_view(1,3); intersect2(1)],...
             [camera2_view(2,3); intersect2(2)], 'Color', 'r');
    end
    if(sum(isinf(camera2_view(:,4)))<1)
        line([camera2_view(1,4); intersect2(1)],...
             [camera2_view(2,4); intersect2(2)], 'Color', 'g');         
    end
    plot(camera2_view(1,:), camera2_view(2,:), '.b');

    pause(.31)   
    
    
end


%% Tracking

% Sensor Selection acc to the paper

if chosen_cam==1
    measure_pts = point_cam1_2d;
    measure_f = f1;
else
    measure_pts = point_cam2_2d;
    measure_f = f2;
end


if (sum(isInf(measure_pts(:))) > 0)
    cam2world_miss();    
else
    measure_3d = cam2world(measure_pts, measure_f);
end


pred_state = prev_state + [0; cur_state(end); 0; 0; 0];

%Correct with measurement
if (sum(isInf(measure_pts(:))) > 0)
    correct_state = pred_state;
else
    measure_state = state_est(measure_pts);
    if frame>1
        correct_state(end) = measure_state(2) - prev_state(2);
    end
    %presently, just set corrected the same as measured
    correct_state(1:end-1) = measure_state;
end

angle_of_interest = correct_state(3) + correct_state(4);


stat_vector = [stat_vector; chosen_cam, angle_of_interest, truth_angle];
    