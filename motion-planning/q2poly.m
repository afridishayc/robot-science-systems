% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    frame_1_transform_wbase = transl2(robot.pivot1(1), robot.pivot1(2)) * trot2(q(1));
    frame_2_transform_wframe1 = transl2(robot.pivot2(1), robot.pivot2(2)) * trot2(q(2));
    frame_2_transform_wbase = frame_1_transform_wbase * frame_2_transform_wframe1;
    link1_edges_wbase = frame_1_transform_wbase * [robot.link1; 1 1 1 1];
    link2_edges_wbase = frame_2_transform_wbase * [robot.link2; 1 1 1 1];
    poly1 = polyshape(link1_edges_wbase(1,:), link1_edges_wbase(2,:));
    poly2 = polyshape(link2_edges_wbase(1,:), link2_edges_wbase(2,:));
    pivot1 = robot.pivot1;
    int_pivot2 = frame_1_transform_wbase * [robot.pivot2; 1];
    pivot2 = int_pivot2([1, 2], :);
end