% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
    traj = [];
    qCurrent = qInit;
    tm = f.fkine(qCurrent);
    current_pose = tm.t;
    current_dist = norm(posGoal - current_pose);
    while current_dist > epsilon
        for i = 1:10
            tm = f.fkine(qCurrent);
            current_pose = tm.t;
            dir_vec = posGoal - current_pose
            current_dist = norm(dir_vec);
            jacob = f.jacob0(qCurrent);
            inv_jacob = pinv(jacob);
            disp(current_pose);
            intermediate = ( velocity * dir_vec );
            dq = inv_jacob * [intermediate; 0; 0; 0];
            qCurrent = qCurrent + dq';
        end
        traj = [traj ; qCurrent];
    end
end