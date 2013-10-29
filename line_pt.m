function [ occluded ] = line_pt( v, w, p, thresh, l_z, p_z)
%LINE_PT Summary of this function goes here
%   Detailed explanation goes here

    occluded=0;
    l_len2 = dot([v-w]',v-w);

    t = dot([p-v]',w-v)/l_len2;

    if (t>0 && t<1)
        %projection of point falls on line segment
        p_proj = v + t * (w-v);
        dist = norm(p-p_proj);
        if (dist<thresh && l_z<p_z)
            occluded=1;
        end
    end
end

