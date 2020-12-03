function neighbours = get_neighbours(current_pos)
    neighbours_r1 = [current_pos(1)-1  current_pos(1)-1 current_pos(1)-1 current_pos(1) current_pos(1) current_pos(1)+1  current_pos(1)+1 current_pos(1)+1];
    neighbours_r2 = [current_pos(2)-1  current_pos(2) current_pos(2)+1 current_pos(2)-1 current_pos(2)+1 current_pos(2)-1  current_pos(2) current_pos(2)+1];
    neighbours = flip([neighbours_r1 ; neighbours_r2], 2);
end