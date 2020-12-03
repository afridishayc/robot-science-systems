% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    closest_q_start = [];
    closest_q_start_indexes = [];
    norm_ = Inf;
    index_i = 1;
    for i = q_grid
        index_j = 1;
        for j = q_grid
            temp_norm = norm([i j]' - q_start);
            if temp_norm < norm_
               closest_q_start = [i j]';
               norm_ = temp_norm;
               closest_q_start_indexes = [index_i;index_j];
            end
            index_j = index_j + 1;
        end
        index_i = index_i + 1;
    end

    current_cell = closest_q_start_indexes;
    path = [closest_q_start_indexes'] 
    while distances(current_cell(1), current_cell(2)) ~= 2
        neighbours = get_neighbours(current_cell);
        current_dist = distances(current_cell(1), current_cell(2));
        for nei = neighbours
           if nei(1) < 101 &  nei(2) < 101 & nei(1) > 0 & nei(2) > 0
               temp_cell = nei;
               temp_dist = distances(nei(1), nei(2));
               if temp_dist >= 2
                   if temp_dist < current_dist
                       current_cell = temp_cell;
                   end
               end
           end
        end
        path = [path; current_cell'];
    end
end