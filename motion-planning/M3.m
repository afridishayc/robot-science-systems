% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    
    path_found = false;
    path = [];
    graph_ = graph(adjacency);
    q_start_sorted = knnsearch(samples, q_start, 'K', 100);
    q_goal_sorted= knnsearch(samples, q_goal, 'K', 100);
    q_start_graph = zeros(1, 4);
    q_goal_graph = zeros(1, 4);
    start_edge_found = false;
    goal_edge_found = false;
    for q_start_index = q_start_sorted
        current_q_config = samples(q_start_index, :);
        config_in_collision = check_collision(robot, current_q_config, link_radius, sphere_centers, sphere_radii);
        if ~config_in_collision
            edge_in_collision = check_edge(robot, q_start, current_q_config, link_radius, sphere_centers, sphere_radii);
            if ~edge_in_collision
                q_start_graph = current_q_config;
                start_edge_found = true;
                break
            end
        end
    end
    
    for q_goal_index = q_goal_sorted
        current_q_config = samples(q_goal_index, :);
        config_in_collision = check_collision(robot, current_q_config, link_radius, sphere_centers, sphere_radii);
        if ~config_in_collision
            edge_in_collision = check_edge(robot, current_q_config, q_goal, link_radius, sphere_centers, sphere_radii);
            if ~edge_in_collision
                q_goal_graph = current_q_config;
                goal_edge_found = true;
                break
            end
        end
    end
    
    if goal_edge_found & start_edge_found
        path_indexes = shortestpath(graph_, q_start_index, q_goal_index);
        if length(path_indexes) > 0
            path_found = true;
        end
        path = [path; q_start];
        for path_idx = path_indexes
            path = [path; samples(path_idx, :)];
        end
        path = [path; q_goal];
    end
end