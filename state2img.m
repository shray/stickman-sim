function [ camera_view ] = state2img( state, f, image_noise, OCCLUDE_RADIUS, OCCLUDE_WIDTH, STICK_LEN, STICK_RATIOS,ARM_RATIOS, camera_rot, camera_trans )
% SIMILARITY - computes a similarity score between image derived from
% state of stickman and the image observed.

%get stick ends in its own frame of reference
%rotate stick - according to velocity subject to less than 360
phi = state(4);
%rotate by phi
phi_rot = [cosd(-phi) -sind(-phi) 0;
           sind(-phi) cosd(-phi)  0;
                0       0        1];
            
%change theta - or  start waving
% theta = [start_arms_angles(1)+theta_del start_arms_angles(2)-theta_del];
theta = [state(5); state(6)];

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

no_arms = 2;

arm_starts = zeros(3, no_arms);
arm_ends = zeros(3, no_arms);

for i = 1:no_arms
    arm_starts(:,i) = (stick_ends(:,2)-stick_ends(:,1)).*STICK_RATIOS(i);
    arm_ends(:,i) =  arm_starts(:,i) + (STICK_LEN*ARM_RATIOS(i)) ...
                        .* [cosd(90-theta(i)) 0 sind(90-theta(i))]';
end         

%change to world frame
world_stick_ends = phi_rot*stick_ends + repmat([state(1);state(2);state(3)],1,no_arms);
world_arm_starts = phi_rot*arm_starts + repmat([state(1);state(2);state(3)],1,no_arms);
world_arm_ends = phi_rot*arm_ends + repmat([state(1);state(2);state(3)],1,no_arms);

care_points = [world_stick_ends world_arm_ends;...
                   ones(1,size(world_stick_ends,2)+...
                   size(world_arm_ends,2))];

[camera_view, intersect1] = image_from_3d( f, camera_rot, camera_trans, care_points,...
                                OCCLUDE_RADIUS, OCCLUDE_WIDTH, [world_arm_starts(:,1);1] );
camera_view = camera_view + image_noise * rand(size(camera_view));
                            
end

