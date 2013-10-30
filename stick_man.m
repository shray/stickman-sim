%%  Simulate the -- Stickman
clear
close all

%initialize RNG
rng(1,'twister');
visual=1;

%constants
STICK_LEN = 5; %length in axis of rotation
STICK_RATIOS = [.5, .5]; ARM_RATIOS = [.2, .2]; % ratio to len (arms-intersection, armr, arml)

%Occlusion caused by following - scaled by distance from camera
OCCLUDE_RADIUS = 0.001; %Only for the ends
OCCLUDE_WIDTH = 0.0005; %For the arms and the stick - considering cylinder

start_arms_angles = [45, -45]; %degrees

vel_stick_rotation = [5 0]; %(mean, std) deg/frame

arm_rot = [0 5; 0 5]; %(mean,std) gauss to sample rotation speed
no_arms = 2; %number of arms

translation_s_w = [0 -5 0]'; %translation from stick origin to world in world coor

%Camera-1 constants
f1 = 1;
camera1_mat = [f1 0 0 0;
              0 f1 0 0;
              0 0 1 0];
%about x-axis
camera1_rot  = [1    0       0
               0 cosd(90) -sind(90)
               0 sind(90)  cosd(90)];           
                                                    
cam1_to_world = [0 0 0]';
camera1_noise_std = .003;


%Camera-2 constants
f2 = 1;
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
camera2_noise_std = .003;



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

%Trackers 1 & 2, particle filters on first and second camera views
%state is (St-Endx, St-Endy, St-Endz, Phi, theta1, theta2, vel-phi)'
%initialize part-filter
t1_no_particles = 1000;
t1_theta_sigma = 10;
t1_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);

%initialize state by adding gaussian noise
t1_prev_state = t1_prev_state + .1*randn(size(t1_prev_state));
t1_prev_weights = 1/t1_no_particles * ones(t1_no_particles,1);
t1_new_weights = zeros(size(t1_prev_weights));

%Tracker2
t2_no_particles = 1000;
t2_theta_sigma = 10;
t2_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);

%initialize state by adding gaussian noise
t2_prev_state = t2_prev_state + .1*randn(size(t2_prev_state));
t2_prev_weights = 1/t2_no_particles * ones(t2_no_particles,1);
t2_new_weights = zeros(size(t2_prev_weights));


%Tracker3 - Multiplexer
t3_no_particles = 1000;
t3_theta_sigma = 10;
t3_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);

%initialize state by adding gaussian noise
t3_prev_state = t3_prev_state + .1*randn(size(t3_prev_state));
t3_prev_weights = 1/t3_no_particles * ones(t3_no_particles,1);
t3_new_weights = zeros(size(t3_prev_weights));

total_frame=500;

ground_truth=[];
t1_estimates=[];
t1_error=[];
t2_estimates=[];
t2_error=[];
t3_estimates=[];
t3_error=[];

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
    %additive gaussian noise
    camera1_view = camera1_view + camera1_noise_std * randn(size(camera1_view));
    
    [camera2_view, intersect2] = image_from_3d( f2, camera2_rot, cam2_to_world, care_points,...
                            OCCLUDE_RADIUS, OCCLUDE_WIDTH, [world_arm_starts(:,1);1] );
    %additive gaussian noise
    camera2_view = camera2_view + camera2_noise_std * randn(size(camera2_view));                
                        
    %Camera visualizations - Nice!
    if visual==1
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

    xlim([-.7 .7]);
    ylim([-.2 1.2]);
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
    xlim([-.7 .7]);
    ylim([-.7 .7]);
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
    
%     pause(0.5)
    end
    
    
    %% Tracking        
    
    %Traker1
    %check if N-effective less than threshold
    t1_N_eff = 1/(t1_prev_weights'*t1_prev_weights);
    if t1_N_eff < .2*t1_no_particles
        [t1_prev_state, t1_prev_weights] = resample(t1_prev_state, t1_prev_weights);
    end

    %use dynamics to predict next
    t1_dyn_state = zeros(size(t1_prev_state));
    t1_dyn_state(4,:) = t1_prev_state(end,:);
    t1_dyn_state(5:6,:) = t1_theta_sigma .* randn(2,t1_no_particles);    
    
    %predict next
    t1_next_state = t1_prev_state + t1_dyn_state;
    
    %check angle ranges
    t1_next_state(4,:) = mod(t1_next_state(4,:),360);
    t1_next_state(5,find(t1_next_state(5,:)>179)) = 179;
    t1_next_state(5,find(t1_next_state(5,:)<1)) = 1;
    t1_next_state(6,find(t1_next_state(6,:)<-179)) = -179;
    t1_next_state(6,find(t1_next_state(6,:)>-1)) = -1;

    %find new weights for particles by measurement
    for i=1:t1_no_particles
        t1_new_weights(i) = similarity(t1_next_state(:,i), camera1_view, f1, .0001,...
            OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS, ARM_RATIOS, camera1_rot, cam1_to_world);
        if t1_new_weights(i)==0
            'Zero?';
        end
    end
    %renormalize
    t1_new_weights = t1_new_weights./sum(t1_new_weights);
    %compute best theta estimate
    t1_angle_estimate = t1_next_state(5,:)*t1_new_weights - t1_next_state(6,:)*t1_new_weights;
    
    %debug
    if (isnan(t1_angle_estimate))
        t1_new_weights = t1_prev_weights;
        t1_angle_estimate = t1_next_state(5,:)*t1_new_weights - t1_next_state(6,:)*t1_new_weights;

    end
    
    %propagate
    t1_prev_state = t1_next_state;
    t1_prev_weights = t1_new_weights;
    
    
    %Traker2
    %check if N-effective less than threshold
    t2_N_eff = 1/(t2_prev_weights'*t2_prev_weights);
    if t2_N_eff < .2*t2_no_particles
        [t2_prev_state, t2_prev_weights] = resample(t2_prev_state, t2_prev_weights);
    end

    %use dynamics to predict next
    t2_dyn_state = zeros(size(t2_prev_state));
    t2_dyn_state(4,:) = t2_prev_state(end,:);
    t2_dyn_state(5:6,:) = t2_theta_sigma .* randn(2,t2_no_particles);    
    
    %predict next
    t2_next_state = t2_prev_state + t2_dyn_state;
    
    %check angle ranges
    t2_next_state(4,:) = mod(t2_next_state(4,:),360);
    t2_next_state(5,find(t2_next_state(5,:)>179)) = 179;
    t2_next_state(5,find(t2_next_state(5,:)<1)) = 1;
    t2_next_state(6,find(t2_next_state(6,:)<-179)) = -179;
    t2_next_state(6,find(t2_next_state(6,:)>-1)) = -1;

    %find new weights for particles by measurement
    for i=1:t2_no_particles
        t2_new_weights(i) = similarity(t2_next_state(:,i), camera2_view, f2, .0001,...
            OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS, ARM_RATIOS, camera2_rot, cam2_to_world);
        if t2_new_weights(i)==0
            'Zero?';
        end
    end
    %renormalize
    t2_new_weights = t2_new_weights./sum(t2_new_weights);
    %compute best theta estimate
    t2_angle_estimate = t2_next_state(5,:)*t2_new_weights - t2_next_state(6,:)*t2_new_weights;
    
    %debug
    if (isnan(t2_angle_estimate))
        t2_new_weights = t2_prev_weights;
        t2_angle_estimate = t2_next_state(5,:)*t2_new_weights - t2_next_state(6,:)*t2_new_weights;

    end
    
    %propagate
    t2_prev_state = t2_next_state;
    t2_prev_weights = t2_new_weights;
    
    
    %Traker3 - Multiplexer
    %check if N-effective less than threshold
    t3_N_eff = 1/(t3_prev_weights'*t3_prev_weights);
    if t3_N_eff < .2*t3_no_particles
        [t3_prev_state, t3_prev_weights] = resample(t3_prev_state, t3_prev_weights);
    end

    %use dynamics to predict next
    t3_dyn_state = zeros(size(t3_prev_state));
    t3_dyn_state(4,:) = t3_prev_state(end,:);
    t3_dyn_state(5:6,:) = t3_theta_sigma .* randn(2,t3_no_particles);    
    
    %predict next
    t3_next_state = t3_prev_state + t3_dyn_state;
    
    %check angle ranges
    t3_next_state(4,:) = mod(t3_next_state(4,:),360);
    t3_next_state(5,find(t3_next_state(5,:)>179)) = 179;
    t3_next_state(5,find(t3_next_state(5,:)<1)) = 1;
    t3_next_state(6,find(t3_next_state(6,:)<-179)) = -179;
    t3_next_state(6,find(t3_next_state(6,:)>-1)) = -1;
    
    %flip-camera
    if (mod(cur_frame,2)==0)
        t3_cam_view = camera1_view;
        t3_f = f1;
        t3_cam_rot = camera1_rot;
        t3_cam_to_world = cam1_to_world;
    else        
        t3_cam_view = camera2_view;
        t3_f = f2;
        t3_cam_rot = camera2_rot;
        t3_cam_to_world = cam2_to_world;
    end

    %find new weights for particles by measurement
    for i=1:t3_no_particles
        t3_new_weights(i) = similarity(t3_next_state(:,i), t3_cam_view, t3_f, .0001,...
            OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS, ARM_RATIOS, t3_cam_rot, t3_cam_to_world);
        if t3_new_weights(i)==0
            'Zero?';
        end
    end
    %renormalize
    t3_new_weights = t3_new_weights./sum(t3_new_weights);
    %compute best theta estimate
    t3_angle_estimate = t3_next_state(5,:)*t3_new_weights - t3_next_state(6,:)*t3_new_weights;
    
    %debug
    if (isnan(t3_angle_estimate))
        t3_new_weights = t3_prev_weights;
        t3_angle_estimate = t3_next_state(5,:)*t3_new_weights - t3_next_state(6,:)*t3_new_weights;

    end
    
    %propagate
    t3_prev_state = t3_next_state;
    t3_prev_weights = t3_new_weights;
    
        
    ground_truth=[ground_truth; theta(1)-theta(2)];
    error_t1 = abs(theta(1)-theta(2) - t1_angle_estimate);
    error_t2 = abs(theta(1)-theta(2) - t2_angle_estimate);
    error_t3 = abs(theta(1)-theta(2) - t2_angle_estimate)
    t1_estimates=[t1_estimates; t1_angle_estimate];
    t1_error = [t1_error error_t1];
    t2_estimates=[t2_estimates; t2_angle_estimate];
    t2_error = [t2_error error_t2];    
    t3_estimates=[t3_estimates; t3_angle_estimate];
    t2_error = [t3_error error_t3];
    
    
    

    
end
