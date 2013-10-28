function [ c_3d ] = cam2world( c_pts, lens, f ) 
%The points are expected to be in the following configuration,
% (stick-end points, arm-end-points)
%in c_3d we return 3d points in the foll. config
% (stick-end point1, arm-end-points, intersection)

%get first end point of stick
e1_z = abs((lens(1) * f)/(c_pts(2,1)-c_pts(2,2)));
end_p_1 = [e1_z*c_pts(1,1), e1_z*c_pts(2,1), e1_z*f]'.*(1/f);

%one way of getting the second end-point
end_p_2 = end_p_1 + [0 lens(1) 0]';

%find point of intersection, assuming straight stick and lower end-pt
intr_pt = end_p_1 + [0 lens(2) 0]';

%get arm1
a = ((c_pts(1,3)/f)^2 + (c_pts(2,3)/f)^2 +1);
b =  -2*(intr_pt'*[c_pts(:,3)./f;1]);
c = (intr_pt'*intr_pt) - lens(3)^2;
a1_z1 = (-b + (b^2 - 4*a*c)^.5) / (2*a);
a1_z2 = (-b - (b^2 - 4*a*c)^.5) / (2*a);

%check that one of the possible values is positive while the other is not
%also if real, crazy if's
if (~isreal(a1_z2))    
     'its all gone wrong for first arm, unreal'
    return    
elseif (a1_z1 < 0 && a1_z1 < 0)
    'its all gone wrong for first arm, negatives'
    return
elseif (a1_z1 < 0)
    candidates_a1 = a1_z2;
elseif (a1_z2 < 0)
    candidates_a1 = a1_z1;
else
    candidates_a1 = [a1_z1; a1_z2];
end

%get arm1 3dpoints
a1_pt = [candidates_a1.*c_pts(1,3), candidates_a1.*c_pts(2,3), f*candidates_a1]'.*(1/f);


%get arm2
a2 = ((c_pts(1,4)/f)^2 + (c_pts(2,4)/f)^2 +1);
b2 =  -2*(intr_pt'*[c_pts(:,4)./f;1]);
c2 = (intr_pt'*intr_pt) - lens(3)^2;
a2_z1 = (-b2 + (b2^2 - 4*a2*c2)^.5) / (2*a2);
a2_z2 = (-b2 - (b2^2 - 4*a2*c2)^.5) / (2*a2);

%check that one of the possible values is positive while the other is not
%also if real, crazy if's
if (~isreal(a2_z2))    
     'its all gone wrong for first arm, unreal'
    return    
elseif (a2_z1 < 0 && a2_z1 < 0)
    'its all gone wrong for first arm, negatives'
    return
elseif (a2_z1 < 0)
    candidates_a2 = a2_z2;
elseif (a2_z2 < 0)
    candidates_a2 = a2_z1;
else
    candidates_a2 = [a2_z1; a2_z2];
end

%get arm2 3dpoints
a2_pt = [candidates_a2.*c_pts(1,4), candidates_a2.*c_pts(2,4), f*candidates_a2]'.*(1/f);


%determine the winners from likely candidates
a1 = 0; a2 = 0;
for i=1:size(a1_pt,2)
    for j=1:size(a2_pt,2)
        a1_proj = (a1_pt(:,i)-intr_pt) - dot((a1_pt(:,i)-intr_pt),[0 1 0]') * [0 1 0]';
        a2_proj = (a2_pt(:,j)-intr_pt) - dot((a2_pt(:,j)-intr_pt),[0 1 0]') * [0 1 0]';
        if (dot(a1_proj, a2_proj) == (-norm(a1_proj)*norm(a2_proj)))
            a1 = a1_pt(:,i);
            a2 = a2_pt(:,j);
        end

    end
end

c_3d = [end_p_1, a1, a2, intr_pt];

end

