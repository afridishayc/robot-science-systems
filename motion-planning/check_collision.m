% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q -> 1x4 vector denoting the configuration to check for collision
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: in_collision -> Boolean, true if the robot at configuration q is
%                         in collision with the given spherical obstacles

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, resolution)
    % calculating link homogeneous transforms(and extracting translation part) between the base frame and link frames
    x1 = [0 0 0]';
    T2 = robot.A(1,q) * robot.A(2,q) * robot.A(3,q);
    x2 = T2.t;
    T3 = T2 * robot.A(4,q);
    x3 = T3.t;
    
    if nargin < 6
        resolution = 11;
    end
    % generating linearly spaced vector
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    
    % getting points along the direction of each link or points in the
    % links (linearly separated)
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1);
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    
    % collection of points on both the links of the arm
    points = [x12 x23];
    
    in_collision = false;
    % for each spherical obstacle in the workspace
    for i = 1:size(sphere_centers, 1)
        % calculating the distance between points on links and the
        % center of a spherical obstacle. If the calculated distance is
        % less the the sum of link radius and a spherical obstacle then the
        % link is in collision with the obstacle otherwise not.
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
end