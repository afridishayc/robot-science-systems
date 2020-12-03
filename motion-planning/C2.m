% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    q_grid_size = size(q_grid);
    cspace = zeros(q_grid_size(2), q_grid_size(2));
    index_1 = 1;
    for i = q_grid
        index_2 = 1;
        for j = q_grid
            [poly1, poly2, pivot1, pivot2] = q2poly(robot, [i j]');
            for obstacle = obstacles
                 polyout1 = intersect(poly1, obstacle).NumRegions;
                 polyout2 = intersect(poly2, obstacle).NumRegions;
                 if polyout1 > 0 | polyout2 > 0
                    cspace(index_1, index_2) = 1;
                 end
            end
            index_2 = index_2 + 1;
        end
        index_1 = index_1 + 1;
    end
end