%%  Simulate the -- Stickman
clear
close all

%initialize RNG
rng(30,'twister');
visual=0;

%constants
STICK_LEN = 5; %length in axis of rotation
STICK_RATIOS = [.5, .5]; ARM_RATIOS = [.2, .2]; % ratio to len (arms-intersection, armr, arml)

%Occlusion caused by following - scaled by distance from camera
OCCLUDE_RADIUS = .1; %Only for the ends
OCCLUDE_WIDTH = .05; %For the arms and the stick - considering cylinder

start_arms_angles = [45, -45]; %degrees

vel_stick_rotation = [5 0]; %(mean, std) deg/frame

arm_rot = [0 5; 0 5]; %(mean,std) gauss to sample rotation speed
no_arms = 2; %number of arms

translation_s_w = [0 -5 0]'; %translation from stick origin to world in world coor

%Camera-1 constants
f1 = 650; %in pixels
camera1_mat = [f1 0 0 0;
              0 f1 0 0;
              0 0 1 0];
%about x-axis
camera1_rot  = [1    0       0
               0 cosd(-90) -sind(-90)
               0 sind(-90)  cosd(-90)];           
                                                    
cam1_to_world = [0 0 0]';
camera1_noise_std = 2.;
camera1_occl = [];


%Camera-2 constants
f2 = 650;
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
camera2_noise_std = 2.;
camera2_occl = [];


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
t1_no_particles = 6000;
t1_theta_sigma = 10;
t1_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);

%initialize state by adding gaussian noise
t1_prev_state = t1_prev_state + .1*randn(size(t1_prev_state));
t1_prev_weights = 1/t1_no_particles * ones(t1_no_particles,1);
t1_new_weights = zeros(size(t1_prev_weights));

%Tracker2
t2_no_particles = 6000;
t2_theta_sigma = 10;
t2_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);

%initialize state by adding gaussian noise
t2_prev_state = t2_prev_state + .1*randn(size(t2_prev_state));
t2_prev_weights = 1/t2_no_particles * ones(t2_no_particles,1);
t2_new_weights = zeros(size(t2_prev_weights));


%Tracker3 - Multiplexer
t3_no_particles = 6000;
t3_theta_sigma = 10;
t3_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);

%initialize state by adding gaussian noise
t3_prev_state = t3_prev_state + .1*randn(size(t3_prev_state));
t3_prev_weights = 1/t3_no_particles * ones(t3_no_particles,1);
t3_new_weights = zeros(size(t3_prev_weights));

%Tracker4 - Multiplexer
t4_no_particles = 6000;
t4_theta_sigma = 10;
t4_prev_state = repmat([translation_s_w;phi;theta(1);theta(2);vel_stick_rotation(1)],1,t1_no_particles);
t4_cam_choice = 1;

%initialize state by adding gaussian noise
t4_prev_state = t4_prev_state + .1*randn(size(t4_prev_state));
t4_prev_weights = 1/t4_no_particles * ones(t4_no_particles,1);
t4_new_weights = zeros(size(t4_prev_weights));
t4_no_predicts  = 10;
t4_no_measures = 3;
total_frame=200;

ground_truth=[];
t1_estimates=[];
t1_error=[];
t2_estimates=[];
t2_error=[];
t3_estimates=[];
t3_error=[];
t4_estimates=[];
t4_error=[];

t1_parz_guess = [];
t1_parz_entrp = [];
t2_parz_guess = [];
t2_parz_entrp = [];
t3_parz_guess = [];
t3_parz_entrp = [];
t4_parz_guess = [];
t4_parz_entrp = [];
t4_cam_choices = [t4_cam_choice];

%start STICK-MAN dynamics
for cur_frame=0:total_frame
   
    cur_frame
    
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
    %record presence of occlusions
    if (inf>sum(camera1_view(:)))
        camera1_occl = [camera1_occl; 0];
    else
        camera1_occl = [camera1_occl; 1];
    end

    
    [camera2_view, intersect2] = image_from_3d( f2, camera2_rot, cam2_to_world, care_points,...
                            OCCLUDE_RADIUS, OCCLUDE_WIDTH, [world_arm_starts(:,1);1] );
    %additive gaussian noise
    camera2_view = camera2_view + camera2_noise_std * randn(size(camera2_view));
    %record presence of occlusions
    if (inf>sum(camera2_view(:)))
        camera2_occl = [camera2_occl; 0];
    else
        camera2_occl = [camera2_occl; 1];
    end

                        
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

%             xlim([-.7 .7]);
%             ylim([-.2 1.2]);
            xlim([-700 700])
            ylim([-500 900])
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
%             xlim([-.7 .7]);
%             ylim([-.7 .7]);
            xlim([-700 700])
            ylim([-700 700])
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
        t1_new_weights(i) = similarity(t1_next_state(:,i), camera1_view, f1, 16,...
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
    
    %parzen-window estimate
    t1_theta_diff = t1_next_state(5,:) - t1_next_state(6,:);
    [t1_td_est, t1_td_dist] = p_win(t1_theta_diff', t1_new_weights, 50, 0, 360);
    %normalize
    t1_td_dist = t1_td_dist./sum(t1_td_dist);
    t1_td_conv = imfilter(t1_td_dist, ones(101,1));
    [t1_maxp, max_idx] = max(t1_td_conv);
    max_idx = max(max_idx-50,1):min(max_idx+50,numel(t1_td_dist));
    %Angle guess from density
    t1_td_guess = sum(t1_td_dist(max_idx).*t1_td_est(max_idx))/sum(t1_td_dist(max_idx));
    %Entropy Calculation
    t1_td_log = log(t1_td_dist);
    t1_td_log(find(t1_td_dist < eps)) = log(eps);
    t1_td_dist(find(t1_td_dist < eps)) = eps;
    t1_td_entropy =  - t1_td_log'*t1_td_dist;
    
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
        t2_new_weights(i) = similarity(t2_next_state(:,i), camera2_view, f2, 16,...
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
    
    %parzen-window estimate
    t2_theta_diff = t2_next_state(5,:) - t2_next_state(6,:);
    [t2_td_est, t2_td_dist] = p_win(t2_theta_diff', t2_new_weights, 50, 0, 360);
    %normalize
    t2_td_dist = t2_td_dist./sum(t2_td_dist);
    t2_td_conv = imfilter(t2_td_dist, ones(101,1));
    [t2_maxp, max_idx] = max(t2_td_conv);
    max_idx = max(max_idx-50,1):min(max_idx+50,numel(t2_td_dist));
    %Angle guess from density
    t2_td_guess = sum(t2_td_dist(max_idx).*t2_td_est(max_idx))/sum(t2_td_dist(max_idx));
    %Entropy Calculation
    t2_td_log = log(t2_td_dist);
    t2_td_log(find(t2_td_dist < eps)) = log(eps);
    t2_td_dist(find(t2_td_dist < eps)) = eps;
    t2_td_entropy =  - t2_td_log'*t2_td_dist;
    
    
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
        t3_new_weights(i) = similarity(t3_next_state(:,i), t3_cam_view, t3_f, 16,...
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
    
    %parzen-window estimate
    t3_theta_diff = t3_next_state(5,:) - t3_next_state(6,:);
    [t3_td_est, t3_td_dist] = p_win(t3_theta_diff', t3_new_weights, 50, 0, 360);
    %normalize
    t3_td_dist = t3_td_dist./sum(t3_td_dist);
    t3_td_conv = imfilter(t3_td_dist, ones(101,1));
    [t3_maxp, max_idx] = max(t3_td_conv);
    max_idx = max(max_idx-50,1):min(max_idx+50,numel(t3_td_dist));
    %Angle guess from density
    t3_td_guess = sum(t3_td_dist(max_idx).*t3_td_est(max_idx))/sum(t3_td_dist(max_idx));
    %Entropy Calculation
    t3_td_log = log(t3_td_dist);
    t3_td_log(find(t3_td_dist < eps)) = log(eps);
    t3_td_dist(find(t3_td_dist < eps)) = eps;
    t3_td_entropy =  - t3_td_log'*t3_td_dist;
    
    %propagate
    t3_prev_state = t3_next_state;
    t3_prev_weights = t3_new_weights;
    
    
    %Traker4 - Intelligent Switching
    %check if N-effective less than threshold
    if cur_frame==13
        'Oh'
    end
    t4_N_eff = 1/(t4_prev_weights'*t4_prev_weights);
    if t4_N_eff < .2*t4_no_particles
        [t4_prev_state, t4_prev_weights] = resample(t4_prev_state, t4_prev_weights);
    end

    %use dynamics to predict next
    t4_dyn_state = zeros(size(t4_prev_state));
    t4_dyn_state(4,:) = t4_prev_state(end,:);
    t4_dyn_state(5:6,:) = t4_theta_sigma .* randn(2,t4_no_particles);    
    
    %predict next
    t4_next_state = t4_prev_state + t4_dyn_state;
    
    %check angle ranges
    t4_next_state(4,:) = mod(t4_next_state(4,:),360);
    t4_next_state(5,find(t4_next_state(5,:)>179)) = 179;
    t4_next_state(5,find(t4_next_state(5,:)<1)) = 1;
    t4_next_state(6,find(t4_next_state(6,:)<-179)) = -179;
    t4_next_state(6,find(t4_next_state(6,:)>-1)) = -1;
    
    %use chosen-camera
    if (t4_cam_choice==1)
        t4_cam_view = camera1_view;
        t4_f = f1;
        t4_cam_rot = camera1_rot;
        t4_cam_to_world = cam1_to_world;
    else        
        t4_cam_view = camera2_view;
        t4_f = f2;
        t4_cam_rot = camera2_rot;
        t4_cam_to_world = cam2_to_world;
    end

    %find new weights for particles by measurement
    for i=1:t4_no_particles
        t4_new_weights(i) = similarity(t4_next_state(:,i), t4_cam_view, t4_f, 16,...
            OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS, ARM_RATIOS, t4_cam_rot, t4_cam_to_world);
        if t4_new_weights(i)==0
            'Zero?';
        end
    end
    %renormalize
    t4_new_weights = t4_new_weights./sum(t4_new_weights);
    %compute best theta estimate
    t4_angle_estimate = t4_next_state(5,:)*t4_new_weights - t4_next_state(6,:)*t4_new_weights;
    
    %debug
    if (isnan(t4_angle_estimate))
        t4_new_weights = t4_prev_weights;
        t4_angle_estimate = t4_next_state(5,:)*t4_new_weights - t4_next_state(6,:)*t4_new_weights;

    end
    
    %parzen-window estimate
    t4_theta_diff = t4_next_state(5,:) - t4_next_state(6,:);
    [t4_td_est, t4_td_dist] = p_win(t4_theta_diff', t4_new_weights, 50, 0, 360);
    %normalize
    t4_td_dist = t4_td_dist./sum(t4_td_dist);
    t4_td_conv = imfilter(t4_td_dist, ones(101,1));
    [t4_maxp, max_idx] = max(t4_td_conv);
    max_idx = max(max_idx-50,1):min(max_idx+50,numel(t4_td_dist));
    %Angle guess from density
    t4_td_guess = sum(t4_td_dist(max_idx).*t4_td_est(max_idx))/sum(t4_td_dist(max_idx));
    %Entropy Calculation
    t4_td_log = log(t4_td_dist);
    t4_td_log(find(t4_td_dist < eps)) = log(eps);
    t4_td_dist(find(t4_td_dist < eps)) = eps;
    t4_td_entropy =  - t4_td_log'*t4_td_dist;
    
    %propagate
    t4_prev_state = t4_next_state;
    t4_prev_weights = t4_new_weights;
    
    %CHOOSE the next camera to use
    %use dynamics to predict next
    t4_dyn_state = zeros(size(t4_prev_state));
    t4_dyn_state(4,:) = t4_prev_state(end,:);
    t4_dyn_state(5:6,:) = t4_theta_sigma .* randn(2,t4_no_particles);    
    %predict next
    t4_nxt_state = t4_prev_state + t4_dyn_state;
    %check angle ranges
    t4_nxt_state(4,:) = mod(t4_nxt_state(4,:),360);
    t4_nxt_state(5,find(t4_nxt_state(5,:)>179)) = 179;
    t4_nxt_state(5,find(t4_nxt_state(5,:)<1)) = 1;
    t4_nxt_state(6,find(t4_nxt_state(6,:)<-179)) = -179;
    t4_nxt_state(6,find(t4_nxt_state(6,:)>-1)) = -1;
    %hallucinate measurements
    t4_cam1_entropy = 0;
    t4_cam2_entropy = 0;
    t4_cam1_weights = t4_new_weights;
    t4_cam2_weights = t4_new_weights;
    %random sample 
    rand_particles = randsample(1:t4_no_particles, t4_no_predicts);
    for p=1:t4_no_predicts
        t4_predict_state = t4_nxt_state(:,rand_particles(p));
    %find new weights for particles by measurement
    for m=1:t4_no_measures
        t4_cam1_halluci = state2img( t4_predict_state, f1, 1, OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS,ARM_RATIOS, camera1_rot, cam1_to_world);
        t4_cam2_halluci = state2img( t4_predict_state, f2, 1, OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS,ARM_RATIOS, camera2_rot, cam2_to_world);

    for i=1:t4_no_particles
        t4_cam1_weights(i) = similarity(t4_nxt_state(:,i), t4_cam1_halluci, f1, 16,...
            OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS, ARM_RATIOS, camera1_rot, cam1_to_world);
        t4_cam2_weights(i) = similarity(t4_nxt_state(:,i), t4_cam2_halluci, f2, 16,...
            OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS, ARM_RATIOS, camera2_rot, cam2_to_world);
        
        if (t4_cam1_weights(i)==0 || t4_cam2_weights(i)==0) 
            'Zero?';
        end
    end
    
    %parzen-window estimate
    t4_theta_diff = t4_nxt_state(5,:) - t4_nxt_state(6,:);
    [meh, t4_cam1_dist] = p_win(t4_theta_diff', t4_cam1_weights, 50, 0, 360);
    [meh, t4_cam2_dist] = p_win(t4_theta_diff', t4_cam2_weights, 50, 0, 360);
    %normalize
    t4_cam1_dist = t4_cam1_dist./sum(t4_cam1_dist);
    %Entropy Calculation
    t4_cam1_log = log(t4_cam1_dist);
    t4_cam1_log(find(t4_cam1_dist < eps)) = log(eps);
    t4_cam1_dist(find(t4_cam1_dist < eps)) = eps;
    t4_cam1_entropy =  t4_cam1_entropy - t4_cam1_log'*t4_cam1_dist;
    %normalize
    t4_cam2_dist = t4_cam2_dist./sum(t4_cam2_dist);
    %Entropy Calculation
    t4_cam2_log = log(t4_cam2_dist);
    t4_cam2_log(find(t4_cam2_dist < eps)) = log(eps);
    t4_cam2_dist(find(t4_cam2_dist < eps)) = eps;
    t4_cam2_entropy =  t4_cam2_entropy - t4_cam2_log'*t4_cam2_dist;
    
    
    end
    end
    %compare entropies to choose camera
    t4_cam1_entropy = t4_cam1_entropy/(t4_no_predicts*t4_no_measures);
    t4_cam2_entropy = t4_cam2_entropy/(t4_no_predicts*t4_no_measures);
    if t4_cam_choice==1 && t4_cam1_entropy-t4_cam2_entropy>.5
        t4_cam_choice=2;
    elseif t4_cam_choice==2 && t4_cam2_entropy-t4_cam1_entropy>.5
        t4_cam_choice=1;
    end
        
    ground_truth=[ground_truth; theta(1)-theta(2)];
    error_t1 = abs(theta(1)-theta(2) - t1_angle_estimate);
    error_t2 = abs(theta(1)-theta(2) - t2_angle_estimate);
    error_t3 = abs(theta(1)-theta(2) - t3_angle_estimate);
    error_t4 = abs(theta(1)-theta(2) - t4_angle_estimate);
    t1_estimates=[t1_estimates; t1_angle_estimate];
    t1_error = [t1_error error_t1];
    t2_estimates=[t2_estimates; t2_angle_estimate];
    t2_error = [t2_error error_t2];    
    t3_estimates=[t3_estimates; t3_angle_estimate];
    t3_error = [t3_error error_t3];
    t4_estimates=[t4_estimates; t4_angle_estimate];
    t4_error = [t4_error error_t4];
    
    %more t1 - vars of interest
    t1_parz_guess = [t1_parz_guess; t1_td_guess]; 
    t1_parz_entrp = [t1_parz_entrp; t1_td_entropy];
    
    %more t2 - vars of interest
    t2_parz_guess = [t2_parz_guess; t2_td_guess]; 
    t2_parz_entrp = [t2_parz_entrp; t2_td_entropy];
    
    %more t3 - vars of interest
    t3_parz_guess = [t3_parz_guess; t3_td_guess]; 
    t3_parz_entrp = [t3_parz_entrp; t3_td_entropy];
    
    %more t4 - vars of interest
    t4_parz_guess = [t4_parz_guess; t4_td_guess]; 
    t4_parz_entrp = [t4_parz_entrp; t4_td_entropy];
    t4_cam_choices = [t4_cam_choices; t4_cam_choice];

    
end
