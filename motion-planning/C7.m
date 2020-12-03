% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    padded_cspace = cspace;
    cspace_shape = size(cspace);
    function neighbours = get_next_neighbours(current_pos)
        neighbours_r1 = [current_pos(1)+1 current_pos(1)-1 current_pos(1) current_pos(1)+1];
        neighbours_r2 = [current_pos(2)  current_pos(2)+1 current_pos(2)+1 current_pos(2)+1];
        neighbours = [neighbours_r1 ; neighbours_r2];
    end
    for i = 1:cspace_shape(1)
        for j = 1:cspace_shape(2)
            neighbours = get_next_neighbours([i;j]);
            for nei = neighbours
                if nei(1) < 101 & nei(2) < 101 & nei(1) > 0 & nei(2) > 0
                   if cspace(nei(1), nei(2)) == 1 
                      padded_cspace(i, j) = 1;
                   end 
                end
            end
        end
    end
end