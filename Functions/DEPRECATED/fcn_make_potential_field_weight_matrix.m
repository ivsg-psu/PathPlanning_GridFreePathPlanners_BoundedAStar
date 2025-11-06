function weight_matrix = fcn_make_potential_field_weight_matrix(vgraph, all_pts, integration_method)
warning('on','backtrace');
warning(['fcn_make_potential_field_weight_matrix is being deprecated. ' ...
    'Use fcn_BoundedAStar_makePotentialFieldWeightMatrix instead.']);



    weight_matrix = ones(size(vgraph,1), size(vgraph,2));
    %% implements a way of solving line integrals in a vector field per:
    % https://www.mathworks.com/matlabcentral/answers/1436619-how-can-i-compute-line-integral-problem-in-matlab
    if strcmp(integration_method, 'symbolic')
        syms F1(x,y,z) F2(x,y,z) F3(x,y,z) r1(t) r2(t) r3(t)
        F1(x,y,z) = (x-2.2);
        F2(x,y,z) = (y-3);
        F3(x,y,z) = 0*z;
        sz = [size(vgraph,1) size(vgraph,2)];
        ind = find(vgraph-eye(size(vgraph,1))==1);
        [row,col] = ind2sub(sz,ind);
        for i = 1:length(row)
            P1 = (all_pts(row(i),1:3))'; % start point
            P2 = (all_pts(col(i),1:3))'; % end point
            r1(t) = P1(1) + t*(P2(1)-P1(1)); % parameterization of the line segment from P1 to P2
            r2(t) = P1(2) + t*(P2(2)-P1(2)); % parameterization of the line segment from P1 to P2
            r3(t) = P1(3) + t*(P2(3)-P1(3)); % parameterization of the line segment from P1 to P2
            r = [r1;r2;r3];
            F = [F1;F2;F3];
            Fr = F(r1(t),r2(t),r3(t));
            dr = diff(r(t),t);
            integrand = dot(Fr,dr);
            weight = int(integrand,t,[0 1]);
            weight_matrix(row(i),col(i)) = double(weight);
        end
    end
    if strcmp(integration_method, 'numeric')
        step_size = 0.05;
        t = (0:step_size:1)'; % line segment will be parameterized as r(t)
        sz = [size(vgraph,1) size(vgraph,2)];
        ind = find(vgraph-eye(size(vgraph,1))==1);
        [row,col] = ind2sub(sz,ind);
        for i = 1:length(row)
            P1 = (all_pts(row(i),1:3)); % start point
            P2 = (all_pts(col(i),1:3)); % end point
            r = ones(length(t),1)*P1 + t*(P2-P1); % parameterization of the line segment from P1 to P2
            r1x = r(:,1);
            r2y = r(:,2);
            r3t = r(:,3);
            F1 = (r1x-2.2);
            F2 = (r2y-3);
            F3 = 0*r3t;
            Fr = [F1 F2 F3];
            Fr = Fr(2:end,:); % right difference method so drop first entry
            dr = diff(r)/step_size;
            integrand = dot(Fr,dr,2); % treat each row as a vector
            % TODO do we want sum of integrand or sum of integrand times dt
            weight = sum(integrand*step_size);
            weight_matrix(row(i),col(i)) = weight;
        end
    end
    %% full sumbolic implementation
    % syms F1(x,y,z) F2(x,y,z) F3(x,y,z) r1(t) r2(t) r3(t)
    % F1(x,y,z) = -(x-0.4);
    % F2(x,y,z) = -(y-0.4);
    % F3(x,y,z) = 0*z;
    % sz = [size(vgraph,1) size(vgraph,2)];
    % ind = find(vgraph-eye(size(vgraph,1))==1);
    % [row,col] = ind2sub(sz,ind);
    % for i = 1:length(row)
    %     P1 = (all_pts(row(i),1:3))'; % start point
    %     P2 = (all_pts(col(i),1:3))'; % end point
    %     r1(t) = P1(1) + t*(P2(1)-P1(1)); % parameterization of the line segment from P1 to P2
    %     r2(t) = P1(2) + t*(P2(2)-P1(2)); % parameterization of the line segment from P1 to P2
    %     r3(t) = P1(3) + t*(P2(3)-P1(3)); % parameterization of the line segment from P1 to P2
    %     r = [r1;r2;r3];
    %     F = [F1;F2;F3];
    %     Fr = F(r1(t),r2(t),r3(t));
    %     dr = diff(r(t),t);
    %     integrand = dot(Fr,dr);
    %     weight = int(integrand,t,[0 1]);
    %     weight_matrix(row(i),col(i)) = double(weight);
    % end
    %% numeric integral version of this algo with explicit t
    % t = (0:0.1:1)'; % line segment will be parameterized as r(t)
    % for i = 1:size(vgraph,1)
    %     P1 = all_pts(i,1:3); % start point
    %     for j = 1:size(vgraph,2)
    %         P2 = all_pts(2,1:3); % end point
    %         r = ones(length(t),1)*P1 + t*(P2-P1); % parameterization of the line segment from P1 to P2
    %     end
    % end
    %% matlab example
    % syms F1(x,y) F2(x,y) r1(t) r2(t)
    % r1(t) = cos(t)+2;
    % r2(t) = sin(t);
    % F1 = -y;
    % F2 = x;
    % r = [r1;r2];
    % F = [F1;F2];
    % Fr = F(r1(t),r2(t));
    % dr = diff(r(t),t);
    % integrand = dot(Fr,dr);
    % work = int(integrand,t,[0,2*pi])

    %% some plotting code
    % % source
    % [startx,starty] = meshgrid(0:0.05:1, 0:0.05:1);
    % startz = zeros(21);
    % u = (startx-0.4);
    % v = (starty-0.4);
    % w = startz*0;
    % quiver3(startx,starty,startz,u,v,w)
    % % sink
    % [startx,starty] = meshgrid(0.3:0.05:0.5, 0.3:0.05:0.5);
    % startz = zeros(5,5);
    % u = (startx-0.4);
    % v = (starty-0.4);
    % w = startz*0;
    % quiver3(startx,starty,startz,u,v,w)
end
