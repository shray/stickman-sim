function [ phi, phi_rot, theta, stick_ends, arm_starts, arm_ends ] = stick_dynamics( phi, vel_stick_rotation, arm_rot, STICK_LEN, ARM_RATIOS, STICK_RATIOS, theta, stick_ends)
%STICK_DYNAMICS - Dynamics of my stick-man. 
%Arguments the current state of the stick man
%Returns a new state of the stick man perturbed according to dynamics model

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

no_arms = 2;

arm_starts = zeros(3, no_arms);
arm_ends = zeros(3, no_arms);

for i = 1:no_arms
    arm_starts(:,i) = (stick_ends(:,2)-stick_ends(:,1)).*STICK_RATIOS(i);
    arm_ends(:,i) =  arm_starts(:,i) + (STICK_LEN*ARM_RATIOS(i)) ...
                        .* [cosd(90-theta(i)) 0 sind(90-theta(i))]';
end         


end

