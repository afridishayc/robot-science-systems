% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    smoothed_path = [];
    can_be_smooted = true;
    current_idx = 1;
    while can_be_smooted
        alt_idx = current_idx + 2;
        if alt_idx > size(path, 1)
            can_be_smoothed = false;
            break
        end
        q_begin = path(current_idx, :);
        q_end = path(alt_idx, :);
        in_collision = check_edge(robot, q_begin, q_end, link_radius, sphere_centers, sphere_radii);
        if ~in_collision
            path(current_idx+1, :) = [];
        end
        current_idx  = current_idx + 1;
    end
    smoothed_path = path;
end