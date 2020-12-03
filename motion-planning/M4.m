% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    path = [];
    path_found = false;
    samples = [q_start];
    num_of_nodes = 400;
    for i = 1:num_of_nodes
        if rand() < 0.05
            % set q_target to goal config
            q_target = q_goal;
        else
            % random config
            q_target = unifrnd(q_min, q_max);
        end
        q_nn_idx = knnsearch(samples, q_target, 'K', 1);
        q_near = samples(q_nn_idx, :);
        current_norm = norm(q_target - q_near);
        q_new = q_near + ((0.2) * (q_target - q_near)) / current_norm;
        in_collision = check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii);
        if ~in_collision
            edge_collision = check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii);
            if ~edge_collision
                samples = [samples; q_new];
            end
        end
    end
    
    num_samples = size(samples, 1);
    adjacency = zeros(num_samples, num_samples);
    num_neighbors = 10;
    for sample_index = 1:num_samples
       current_sample = samples(sample_index, :);
       neighbour_indexes = knnsearch(samples, current_sample, 'K', num_neighbors + 1);
       %disp([sample_index, neighbour_indexes])
       for nei_idx = neighbour_indexes
           if nei_idx ~= sample_index & adjacency(sample_index, nei_idx) == 0
               in_collision = check_edge(robot, current_sample, samples(nei_idx,:), link_radius, sphere_centers, sphere_radii);
               if ~in_collision
                   temp_norm = norm(current_sample - samples(nei_idx,:));
                   adjacency(sample_index, nei_idx) = temp_norm;
                   adjacency(nei_idx, sample_index) = temp_norm;
               end
           end
       end
    end
    
    graph_ = graph(adjacency);
    q_goal_idx = knnsearch(samples, q_goal, 'K', 1);
    path_indexes = shortestpath(graph_, 1, q_goal_idx);
    if length(path_indexes) > 0
        path_found = true;
        for path_idx = path_indexes
            path = [path; samples(path_idx, :)];
        end
        path = [path; q_goal];
    end
end