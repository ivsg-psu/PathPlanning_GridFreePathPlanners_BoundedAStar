function vgraph = fcn_Visibility_3dGraphGlobal(verts, start, finish, all_surfels, speed_limit, time_space_polytopes, dt)
% fcn_Visibility_3dGraphGlobal
%
% Forms the 3D visibility graph, the edges of which connect nodes that are connected by straight,
% collision-free path segments. This leverages the Moller-Trumbore algorithm to check potential
% graph edges for intersections with the 3D obstacles. The visibility graph can also take in a speed
%  limit, which in the case of XYT rather than XYZ is useful for pruning edges that would go backwards
% in time or traverse too much distance in too short of a time
%
%
%
% FORMAT:
% vgraph = fcn_Visibility_3dGraphGlobal(verts, start, finish, all_surfels, speed_limit)
%
%
% INPUTS:
%
%    verts: matrix of all obstacle vertices in the polytope field.  Each row should be a point, and each column is x, y, and z or T
%
%    start: the start point vector (x,t,t)
%
%    finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,t)
%
%     all_surfels: a matrix with all triangular surface elements (surfels) from all timespace polytopes
%     there is one row for each surfel
%     each row has 9 columns representing the x,y,t coordinates of each point of the triangle ordered
%     x1 y1 t1 x2 y2 t2 x3 y3 t3
%
%    speed_limit: a double representing the speed limit for timespace.  Visibilty graph edges
%       violating this speed limit in distance (x y combined) over time (z or t axis) will be discarded
%       running without a speed limit creates an XYZ visibility graph rather than XYT where traversal is possible
%       backwards in time i.e. decreasing in z
%
%
% OUTPUTS:
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
% DEPENDENCIES:
%
% none but surfels can be created from polytopes using fcn_BoundedAStar_makeTriangularSurfelsFromFacets and
% vertices can be interpolated in t using fcn_BoundedAStar_interpolatePolytopesInTime
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a full test suite.
%
% This function was written on summer 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, summer by Steve Harnett
% -- first write of function
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_visibility_graph_3d_global to follow
%    library convention
%
% TO DO:
%
% -- fill in to-do items here.

    all_pts = [verts; start; finish];
    num_pts = size(all_pts,1); % number of rows
    all_pts_idx = 1:1:num_pts; % array of all possible pt idx
    all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts
    all_pt_combos = nchoosek(all_pts_idx,2); % each row of this matrix is a combination of 2 point idxs

    % need to form all possible rays starting at one point and ending at another
    all_ray_starts = all_pts(all_pt_combos(:,1),:); % take all cols of all_pts at the row provided by the first col of all combos
    all_ray_ends = all_pts(all_pt_combos(:,2),:); % take all cols of all_pts at the row provided by the second col of all combos
    all_ray_dirs = all_ray_ends - all_ray_starts; % TriangleRayIntersection takes a ray direction which is end minus beginning
    num_rays = size(all_ray_starts,1);


    % need to form all possible combinations of a ray to check and a surfel it may collide with
    num_surfels = size(all_surfels,1);
    all_ray_idx = 1:1:num_rays;
    all_surfel_idx = 1:1:num_surfels;
    all_surfel_ray_combos = table2array(combinations(all_ray_idx,all_surfel_idx));
    % this means each ray and each surfel will appear more than once
    all_ray_starts_repeated = all_ray_starts(all_surfel_ray_combos(:,1),:);
    all_ray_ends_repeated = all_ray_ends(all_surfel_ray_combos(:,1),:);
    all_ray_dirs_repeated = all_ray_dirs(all_surfel_ray_combos(:,1),:);
    all_surfels_repeated = all_surfels(all_surfel_ray_combos(:,2),:);
    % now we can do the vectorized call to TriangleRayIntersection to check all rays against each surfel
    [intersects, ts, us, vs, xcoors] = TriangleRayIntersection (all_ray_starts_repeated(:,1:3), all_ray_dirs_repeated(:,1:3), all_surfels_repeated(:,1:3),all_surfels_repeated(:,4:6),all_surfels_repeated(:,7:9),'lineType','segment','border','normal');

    vgraph = ones(num_pts); % initialize vgraph as ones, remove edges when intersection occurs
    intersects_idx = find(intersects);
    for k = 1:1:length(intersects_idx)
        i = intersects_idx(k);
        % if the intersection occured at a node that implies that the end of the ray
        % touched a plane segment, rather than the ray passing through the plane
        intersect_x = xcoors(i,1);
        intersect_y = xcoors(i,2);
        intersect_t = xcoors(i,3);
        verts_x = all_pts(:,1);
        verts_y = all_pts(:,2);
        verts_t = all_pts(:,3);
        % thus if one intersection location minus vertex location is approximately zero, the intersection is at a vertex and should not count
        diff_intersect_and_verts = abs([verts_x - intersect_x, verts_y - intersect_y, verts_t - intersect_t]);
        total_diffs = sum(diff_intersect_and_verts,2);
        small_diffs_bool = total_diffs < 10e-14; % this is just a tolerance based on the precision of MATLAB.  Numbers less than 10e-14 are effectively 0 as matlab cannot tell the difference between them and zero.
        % if an intersection occurred and was not at a vertex, we want to set the ray start to ray end and ray end to ray start as 0 in the vgraph (invalid for traversal)
        if sum(small_diffs_bool) == 0
            % plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'Color',[1 0 0],'LineWidth',1)
            % plot3(rmmissing(xcoors(i,1)),rmmissing(xcoors(i,2)),rmmissing(xcoors(i,3)),'cx','MarkerSize',10)
            start_id = all_ray_starts_repeated(i,4);
            end_id = all_ray_ends_repeated(i,4);
            vgraph(start_id,end_id) = 0;
            vgraph(end_id,start_id) = 0;
        end
    end

    %% discard rays too high in velocity using all pts array
    all_delta_ts = (-all_pts(:,3) + (all_pts(:,3))');
    % run is change in total length regardless of x or y
    all_delta_xs = (all_pts(:,1) - (all_pts(:,1))');
    all_delta_ys = (all_pts(:,2) - (all_pts(:,2))');
    all_delta_dist = (all_delta_xs.^2 + all_delta_ys.^2).^0.5;
    all_slopes = all_delta_ts./all_delta_dist; % slope above the horizontal plane is time/dist or 1/speed

    speed_violation_idx = find(all_slopes <= 1/speed_limit ); % find where slope (1/speed) violates the speed limit
    for l = 1:1:length(speed_violation_idx)
        i = speed_violation_idx(l);
        % remove rays that violate the speed limit.  notice this is directional because if beg to term violates speed limit, term to beg may not
        [beg,term] = ind2sub(size(all_slopes),i);
        % plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],'k','LineWidth',2)
        start_id = all_pts(beg,4);
        end_id = all_pts(term,4);
        vgraph(start_id,end_id) = 0;
    end

    %% check for edges entirely contained by polytopes
    % first, we need polytopes at intermediate positions
    [~, double_time_space_polytopes] = fcn_BoundedAStar_interpolatePolytopesInTime(time_space_polytopes,dt/2);
    % for each edge that is allowed
    vgraph_without_self_visible = vgraph - eye(size(vgraph,1));
    linear_idx = find(vgraph_without_self_visible); % find 1s in vgraph
    [rows_of_1s, cols_of_1s] = ind2sub(size(vgraph_without_self_visible),linear_idx); % convert linear idx to r,c
    num_1s = length(rows_of_1s);
    for e = 1:num_1s
        start_pt = all_pts(rows_of_1s(e),1:3);
        end_pt = all_pts(cols_of_1s(e),1:3);
        % parametric equation for line in 3D: https://math.stackexchange.com/questions/404440/what-is-the-equation-for-a-3d-line
        % [x y z]' = [a b c]'*t + [x0 y0 z0]'
        abc_vec = end_pt - start_pt;
        mid_pt = start_pt + 0.5*abc_vec; % find the middle of the edge
        % for each polytope...
        num_polys = length(double_time_space_polytopes);
        p = 1;
        while p <= num_polys
            % first check that the obstacle exists at the time of the midpoint
            tmax = max(double_time_space_polytopes(p).dense_vertices(:,3));
            tmin = min(double_time_space_polytopes(p).dense_vertices(:,3));
            if mid_pt(3) > tmax || mid_pt(3) < tmin
                % if not, it implies the edge is above or below the obstacle but not inside
                p = p+1;
                continue
            end
            % get verts only at this time
            verts_this_time = double_time_space_polytopes(p).dense_vertices(find(double_time_space_polytopes(p).dense_vertices(:,3)==mid_pt(3)),:);
            verts_this_time = sortrows(verts_this_time,4);
            % get xmin and xmax also ymin and ymax
            xmax = max(verts_this_time(:,1));
            xmin = min(verts_this_time(:,1));
            ymax = max(verts_this_time(:,2));
            ymin = min(verts_this_time(:,2));
            in_AABB = (mid_pt(1) < xmax && mid_pt(1) > xmin) && (mid_pt(2) < ymax && mid_pt(2) > ymin);
            % is point between xmin xmax and ymin max? if not continue
            if ~in_AABB
                p = p+1;
                continue
            end
            % if point is in AABB make polyshape from these verts
            polyshape_p = polyshape(verts_this_time(:,1:2));
            % is point in but not on polyshape?
            [is_in,is_on] = isinterior(polyshape_p,mid_pt(1:2));
            % if so, remove the edge, and stop trying polytopes
            if is_in && ~ is_on
                vgraph(rows_of_1s(e),cols_of_1s(e)) = 0;
                p = num_polys+1;
            end
            % if not, continue
            p = p + 1;
        end
    end
end
