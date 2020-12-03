% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder

function [center,axis,radius] = Q2(ptCloud)
    function d = get_distance(X, p)
        l = size(X, 1);
        d = X - repmat(p, l, 1);
        d = sqrt(sum(d.^2,2));
    end
    function n = reSamplePts(pts, minDist)
        max = size(pts,1);
        n(1,:)= pts(randi(max),:);
        cx = 1;
        for i=1:max
            dist = get_distance(n, pts(i,:));
            dist = sum(dist<minDist);
            if(dist>1) continue; end;
            cx = cx + 1;
            n(cx,:)= pts(i,:);
        end;
    end

    function p_3D = translate2D_3D(pts, locx, locy, origin)
        p_3D = zeros(size(pts, 1), 3);
        for i=1:size(pts, 1)
            Lx = pts(i,1); Ly = pts(i,2);
            p_3D(i,:) = origin + Lx*locx + Ly*locy;
        end;
    end

    function [p_2D, locx, locy, origin] = translate3D_2D(pts)
        p0 = pts(1,:); p1 = pts(2,:); p2 = pts(3,:);
        loc0 = p0;                       
        locx = p1 - loc0;                
        locz = cross(locx, p2 - loc0);   
        locy = cross(locz, locx);        
        locx = locx/norm(locx);
        locy = locy/norm(locy);
        p_2D = zeros(size(pts, 1), 2);
        for i=1:size(pts, 1)
            p = pts(i,:) - loc0;
            p_2D(i,:) = [dot(p, locx) dot(p, locy)]; 
        end
        origin = loc0;
    end
    function [center, radius] = fit_circle(pts)
        mat = zeros(3,4);
        for j=1:3   
            x = pts(j,1); y = pts(j,2);
            mat(j, :) = [(x^2 + y^2) x y 1];
        end
        d =  det( mat(:, 2:4));
        x = -det([mat(:, 1:1) mat(:, 3:4)])/(2*d);
        y =  det([mat(:, 1:2) mat(:, 4:4)])/(2*d);
        r = +det( mat(:, 1:3))/(1*d);
        r = r + x^2 + y^2;
        radius = sqrt(r);
        center = [-x -y];
    end
    
    eps = 0.001;     
    pcnn_count = 500;
    sample_dist = 0.01;
    runs  = 500;       
    thresold_ = 10000;     
    
    normals  = pcnormals(ptCloud, pcnn_count);
    pc  = ptCloud.Location;
    pct = pc';
    inCnt = -1;
    found = false;
    while ~found
        n  = datasample(normals, 2); 
        n1 = n(1,:); 
        n2 = n(2,:);
        nat = cross(n1, n2);
        nat = nat / norm(nat);
        na  = nat';
        xplane  = (eye(3,3) - na*nat) * pct;
        xplane  = xplane';
        xplane1 = reSamplePts(xplane, sample_dist);
        if(size(xplane1,1)<3) 
            continue
        end
        [pts2D, locx, locy, or] = translate3D_2D(xplane1);
        for j=1:runs
            pts = datasample(pts2D, 3);
            [nc, nr] = fit_circle(pts);
            if(isnan(nr) || nr>0.3) 
                continue;
            end
            nc = translate2D_3D(nc, locx, locy, or);
            dist = get_distance(xplane, nc);
            dist = abs(dist - nr);
            dist = dist < eps;
            cx  = sum(dist);
            if(cx> inCnt)
                inCnt  = cx;
                disp(cx);
            end
            if(cx > thresold_)
                center = nc';
                radius = nr;
                axis   = na;
                found  = true;
                axis(3) = abs(axis(3));
                temp  = (eye(3,3) - n1'*n1) * pct;
                [temp, ~, ~, ~] = translate3D_2D(temp');
                temp   = max(temp) - min(temp);
                height = temp(2);
                center = center + (axis * height/2);
            end
        end
    end
end

