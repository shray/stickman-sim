function [ vec ] = project_plane( inp, plane_normal )
%PROJECT_PLANE projects given vector onto given plane and returns the
%vector projection

vec = inp - (dot(inp, plane_normal/norm(plane_normal)) * plane_normal)/norm(plane_normal);

end

