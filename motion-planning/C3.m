% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    
    distances = cspace;
    closest_q_goal = [];
    closest_q_goal_indexes = [];
    norm_ = Inf;
    index_i = 1;
    for i = q_grid
        index_j = 1;
        for j = q_grid
            temp_norm = norm([i j]' - q_goal);
            if temp_norm < norm_
               closest_q_goal = [i j]';
               norm_ = temp_norm;
               closest_q_goal_indexes = [index_i;index_j];
            end
            index_j = index_j + 1;
        end
        index_i = index_i + 1;
    end
    
    %set goal value to 2
    distances(closest_q_goal_indexes(1), closest_q_goal_indexes(2)) = 2;
    
    function neighbours = get_neighbours(current_pos)
        neighbours_r1 = [current_pos(1)-1  current_pos(1)-1 current_pos(1)-1 current_pos(1) current_pos(1) current_pos(1)+1  current_pos(1)+1 current_pos(1)+1];
        neighbours_r2 = [current_pos(2)-1  current_pos(2) current_pos(2)+1 current_pos(2)-1 current_pos(2)+1 current_pos(2)-1  current_pos(2) current_pos(2)+1];
        neighbours = [neighbours_r1 ; neighbours_r2];
    end
    
    L = [closest_q_goal_indexes];
    while ~isempty(L)
        current_cell = L(:, 1);
        current_dist = distances(current_cell(1), current_cell(2));
        neighbours = get_neighbours(current_cell);
        L(:, 1) = [];
        for nei = neighbours
            if nei(1) < 101 &  nei(2) < 101 & nei(1) > 0 & nei(2) > 0
                if distances(nei(1), nei(2)) == 0
                    distances(nei(1), nei(2)) = current_dist + 1;
                    L = [L [nei(1);nei(2)]];
                end
            end
        end
    end
end