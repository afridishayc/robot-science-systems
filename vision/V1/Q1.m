% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
function [center,radius] = Q1(ptCloud)
    function d = get_distance(X, p)
        l = size(X, 1);
        d = X - repmat(p, l, 1);
        d = sqrt(sum(d.^2,2));
    end

    function [center, radius] = fit_sphere(P)
        mat = zeros(3,5);
        for j=1:4   
            x = P(j,1); 
            y = P(j,2); 
            z = P(j,3); 
            mat(j, :) = [(x^2 + y^2 + z^2) x y z 1];
        end
        d =  det( mat(:, 2:5));
        x = -det([mat(:, 1:1) mat(:, 3:5)]) / (2*d);
        y =  det([mat(:, 1:2) mat(:, 4:5)]) / (2*d);
        z = -det([mat(:, 1:3) mat(:, 5:5)]) / (2*d);
        r = -det( mat(:, 1:4))              / (1*d);
        r = r + x^2 + y^2 + z^2;
        radius = sqrt(r);
        center = [-x -y -z];
    end

    runs = 1000;
    thresold = 0.001;
    pc  = ptCloud.Location;
    in_c = -1;
    for i=1:runs
        points = datasample(pc, 4);
        [nc, nr] = fit_sphere(points);
        dist = get_distance(pc, nc);
        dist = abs(dist - nr) < thresold;
        c  = sum(dist)
        if(c>in_c)
            center = nc;
            radius = nr;
            in_c  = c;
            disp(c);
        end;
    end
end