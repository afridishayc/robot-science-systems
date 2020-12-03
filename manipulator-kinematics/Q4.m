% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
    traj = []
    mat_shape = size(circle, 2);
    qCurrent = qInit;
    for i=1:mat_shape[1]
        tm = f.fkine(qCurrent);
        current_pose = tm.t;
        posGoal = circle(:, i);
        dir_vec = posGoal - current_pose;
        current_dist = norm(dir_vec);
        disp("=====")
        disp(current_dist)
        while current_dist > 0.1
            for i = 1:10
                tm = f.fkine(qCurrent);
                current_pose = tm.t;
                dir_vec = posGoal - current_pose;
                current_dist = norm(dir_vec);
                jacob = f.jacob0(qCurrent);
                inv_jacob = pinv(jacob);
                intermediate = velocity * dir_vec ;
                dq = inv_jacob * [intermediate; 0; 0; 0];
                qCurrent = qCurrent + dq';
            end
            traj = [traj ; qCurrent];
        end
    end
end