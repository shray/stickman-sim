%%  Simulate the physical system -- Stickman
clear
close all

%initialize RNG
rng(1,'twister');

%constants
STICK_LEN = 5; %length in axis of rotation
STICK_RATIOS = [.5, .5]; ARM_RATIOS =[ .2, .2]; % ratio to len (arms-intersection, armr, arml)

%Occlusion caused by following - scaled by distance from camera
OCCLUDE_RADIUS = 0.1; %Only for the ends
OCCLUDE_WIDTH = 0; %For the arms and the stick - considering cylinder

start_arms_angles = [45, -45]; %degrees

vel_stick_rotation = [5 0]; %(mean, std) deg/frame

arm_rot = [0 5; 0 5]; %(mean,std) gauss to sample rotation speed
no_arms = 2; %number of arms

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

total_frame=10;
%start STICK-MAN dynamics
for cur_frame=0:total_frame

translation_1 = [0 -5 0]'; %translation from stick origin to world in world coor

%rotate stick - according to velocity subject to less than 360
phi = mod(phi + vel_stick_rotation(1) + vel_stick_rotation(2)*randn(),360);
%rotate by phi
phi_rot = [cosd(-phi) -sind(-phi) 0;
           sind(-phi) cosd(-phi)  0;
                0       0        1];
            
%change theta - or  start waving
% theta = [start_arms_angles(1)+theta_del start_arms_angles(2)-theta_del];
theta = theta + arm_rot(:,1) + arm_rot(:,2) .* randn(size(theta));

%TODO: generalize
%checks that arms don't rotate around towards the end for either end
if theta(1)>179 
    theta(1)=179;
elseif theta(1)<1
        theta(1)=1;
end

if theta(2)<-179 
    theta(2)=-179;
elseif theta(2)>-1
        theta(2)=-1;
end

%currently the stick-ends are fixed - could make it a random walk later
stick_ends = [0 0 0; 0 0 STICK_LEN]';

arm_starts = zeros(3, no_arms);
arm_ends = zeros(3, no_arms);

for i = 1:no_arms
    arm_starts(:,i) = (stick_ends(:,2)-stick_ends(:,1)).*STICK_RATIOS(i);
    arm_ends(:,i) =  arm_starts(:,i) + (STICK_LEN*ARM_RATIOS(i)) ...
                        .* [cosd(90-theta(i)) 0 sind(90-theta(i))]';
end            
           
%change to world frame
world_stick_ends = phi_rot*stick_ends + repmat(translation_1,1,no_arms);
world_arm_starts = phi_rot*arm_starts + repmat(translation_1,1,no_arms);
world_arm_ends = phi_rot*arm_ends + repmat(translation_1,1,no_arms);

    
%draw - orthographic projection
clf
figure(2)
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
   
   pause(0.3)   
end

%only care about the image of these 3d homogenous points
care_points = [world_stick_ends world_arm_ends;...
               ones(1,size(world_stick_ends,2)+...
               size(world_arm_ends,2))];



%% Camera1 projection - pinhole
f1 = 0.005;
camera1_mat = [f1 0 0 0;
              0 f1 0 0;
              0 0 1 0];
%about x-axis
camera1_rot  = [1    0       0
               0 cosd(90) -sind(90)
               0 sind(90)  cosd(90)];           
                                                    
cam1_to_world = [0 0 0]';

transform_cam1_frame = [[camera1_rot; zeros(1,3)] [cam1_to_world; 1]];

%project point to camera space
point_cam1_3d = transform_cam1_frame * care_points;
project_mat_cam1 = camera1_mat * eye(size(transform_cam1_frame));
point_cam1_2d = camera_project(point_cam1_3d, project_mat_cam1, OCCLUDE_RADIUS);

%display view
figure, scatter(point_cam1_2d(1,:), point_cam1_2d(2,:));

%% Camera projection2 - pinhole
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

%care about points
% care_points = [world_stick_ends world_arm_starts world_arm_ends;...
%                 ones(1,size(world_stick_ends,2)+...
%                 size(world_arm_ends,2)+size(world_arm_starts,2))];

transform_cam2_frame = [[camera2_rot; zeros(1,3)] [cam2_to_world; 1]];

%project point to camera space
point_cam2_3d = transform_cam2_frame * care_points;
project_mat_cam2 = camera2_mat * eye(size(transform_cam2_frame));
point_cam2_2d = camera_project(point_cam2_3d, project_mat_cam2, OCCLUDE_RADIUS);

%display view from camera
figure, scatter(point_cam2_2d(1,:), point_cam2_2d(2,:));


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
    