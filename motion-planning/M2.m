% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    
    samples = [];
    adjacency = zeros(num_samples, num_samples);
    while size(samples, 1) < num_samples
        new_sample = unifrnd(q_min, q_max);
        in_collision = check_collision(robot, new_sample, link_radius, sphere_centers, sphere_radii);
        if ~in_collision
            samples = [samples;new_sample];
        end
    end
    
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
                   % disp([sample_index nei_idx norm(current_sample - samples(nei_idx,:)) ])
                   %disp(["Sample Index " sample_index "Nei index" nei_idx adjacency(sample_index, nei_idx), adjacency(nei_idx, sample_index)])
                   %disp(["Sample Index", sample_index, "Nei index", nei_idx, "Collision free"]);
               else
                   %disp(["Sample Index", sample_index, "Nei index", nei_idx, "In collision"]);
               end
           end
       end
    end
end