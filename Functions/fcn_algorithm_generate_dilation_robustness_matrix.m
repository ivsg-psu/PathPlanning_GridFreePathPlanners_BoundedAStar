function [dilation_robustness_matrix] = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode, polytopes)
% fcn_algorithm_generate_dilation_robustness_matrix
%
% A function for generating a cost matrix and heuristic cost vector.  The cost matrix describes the
% actual cost to go from one point to another in a map.  The heuristic vector describes the estimated
% cost of going from each point to the goal.  If there are multiple goals, this is the minimum of
% going to any goal because the heuristic should be an underestimate of the actual cost (see:
% http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html).  Note using this function
% is completely optional.  It is commonly used prior to calling a planner such as Astar
% to conveniently generate a cost map but it may be circumvented if the user wishes to generate
% their own cost function and cost function matrix
%
%
% FORMAT:
% dilation_robustness_matrix = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode)
%
%
% INPUTS:
%
%   start: the start point vector as (x,y,id) or (x,t,t,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point
%     vector as (x,y,id) or (x,y,t,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,t,id)
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%   mode: a string for what dimension the inputs are in. The mode argument must be a string with
%     one of the following values:
%       - "3D" - this implies xyz or xyt space
%       - "2D" - this implies xy spatial only dimensions only
%
% OUTPUTS:
%
%  dilation_robustness_matrix - nxn matrix where n is the number of points (nodes) in the map.
%    The value of element i,j is the estimated coridor width surrounding the line segment
%    from point i to point j. I.e., navigating the line segment from i to j affords you Dij lateral width
%
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_fcn_algorithm_generate_dilation_robustness_matrix
% for a full test suite.
%
% This function was written in January 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu
%
% REVISION HISTORY:
%
% January 2024 by Steve Harnett
% -- first write of function
% February 2024 by Steve Harnett
% -- function updated to make a right left distinction using cross products
%
% TO DO:
%
% -- fill in to-do items here.
    vgraph = vgraph - eye(size(vgraph)); % we don't want to consider self interactions here
    all_pts = [all_pts; start; finish];
    % initialize to zero value for each edge (zero implying the edge has no corridor width, i.e. is blocked)
    dilation_robustness_matrix = zeros(size(vgraph,1),size(vgraph,2),2);
    % only need to perform this operation for 1's in vgraph, 0's are blocked edges and have 0 corridor width
    idx_of_valid_edges = find(vgraph==1);
    num_edges = length(idx_of_valid_edges);
    % recall vgrpah edge ij is the line segment from point to point j
    [edge_start_idx, edge_end_idx] = ind2sub(size(vgraph),idx_of_valid_edges);

    % loop through all primary edges
    for i = 1:num_edges
        %%  get the primary edge
        % get the physical location of the edge start and end
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            primary_edge_start = all_pts(edge_start_idx(i),1:2);
            primary_edge_end = all_pts(edge_end_idx(i),1:2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            primary_edge_start = all_pts(edge_start_idx(i),1:3);
            primary_edge_end = all_pts(edge_end_idx(i),1:3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        % find edge direction vector
        primary_edge_dir = primary_edge_end - primary_edge_start;

        %% get the unit normal of the primary edge
        % calculate direction normal to edge
        if primary_edge_dir(2) ~= 0
            normal_dir(1) = 1;
            normal_dir(2) = -(primary_edge_dir(1)*normal_dir(1))./primary_edge_dir(2);
        elseif primary_edge_dir(1) ~=0
            normal_dir(2) = 1;
            normal_dir(1) = -(primary_edge_dir(2)*normal_dir(2))./primary_edge_dir(1);
        else
            warning("visibility graph edge has zero length")
            corridor_width = inf; % for an edge of zero length, routing down the edge
            % is equivalent to staying still which should have no corridor width restriction as
            % staying still is "free" kinematically
            dilation_robustness_matrix(edge_start_idx(i), edge_end_idx(i),:) = corridor_width;
            continue % skip to next edge if this edge had zero length
        end

        normal_mag = norm(normal_dir); % find normal vector magnitude
        unit_normal = normal_dir/normal_mag; % create unit normal from normal vector

        %% get the secondary edges
        % find all visibility graph edges with same origin (these are the secondary edges)
        secondary_edge_ends_idx = find(vgraph(edge_start_idx(i), :)==1);
        % remove the primary edge from the list of secondary edges
        % i.e., we don't want to use the same edge as the primary and secondary edge because there is
        % no corridor width (i.e., lateral spacing) between an edge and itself
        secondary_edge_ends_idx(find(secondary_edge_ends_idx==edge_end_idx(i))) = [];

        % get the physical location of the secondary edges
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            secondary_edge_ends = all_pts(secondary_edge_ends_idx,1:2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            secondary_edge_ends = all_pts(secondary_edge_ends_idx,1:3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        % find the secondary edge direction vectors
        secondary_edge_dirs = secondary_edge_ends - primary_edge_start;


        %% discard irrelevant secondary edges
        % we want to discard edges that either have no component in the direction of the original vector
        % or that end at a point too far away to cut off the original vector

        % TODO set cost as sum of min of left set and min of right set
        % TODO (@sjharentt) need a tool to look up if edge is a polytope side wall

        % vector of all the dot products between the primary edge and all secondary edges
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            dot_secondary_with_unit_primary = secondary_edge_dirs(:,1)*primary_edge_dir(1) + secondary_edge_dirs(:,2)*primary_edge_dir(2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            dot_secondary_with_unit_primary = secondary_edge_dirs(:,1)*primary_edge_dir(1) + secondary_edge_dirs(:,2)*primary_edge_dir(2) + secondary_edge_dirs(:,3)*primary_edge_dir(3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        % dot the unit vector in the direction of the primary edge with the secondary edge
        % to get the component of the secondary vector in the direction of the primary vector
        secondary_component_in_dir_of_primary = dot_secondary_with_unit_primary/norm(primary_edge_dir);
        % if the result is negative, the secondary edge ends "behind" the primary edge
        % if the result is longer than the primary edge it ends "after" the primary edge
        % both cases are discarded
        secondary_edge_dirs(find(secondary_component_in_dir_of_primary <= 0 | secondary_component_in_dir_of_primary > norm(primary_edge_dir)),:) = [];

        %% find if dot product is right or left
        num_secondary_edges = size(secondary_edge_dirs,1);
        primary_edge_dir_repeated = repmat(primary_edge_dir,num_secondary_edges,1);
        cross_primary_with_secondary = cross([primary_edge_dir_repeated,zeros(num_secondary_edges,1)], [secondary_edge_dirs,zeros(num_secondary_edges,1)], 2);

        % dot the other edges with the unit normal to find the corridor width defined by each vgraph edge
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            dot_secondary_with_unit_normal = secondary_edge_dirs(:,1)*unit_normal(1) + secondary_edge_dirs(:,2)*unit_normal(2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            dot_secondary_with_unit_normal = secondary_edge_dirs(:,1)*unit_normal(1) + secondary_edge_dirs(:,2)*unit_normal(2) + secondary_edge_dirs(:,3)*unit_normal(3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        % dot_secondary_with_unit_normal_left = dot_secondary_with_unit_normal(dot_secondary_with_unit_normal>0);
        % dot_secondary_with_unit_normal_right = dot_secondary_with_unit_normal(dot_secondary_with_unit_normal<0);
        dot_secondary_with_unit_normal_left = dot_secondary_with_unit_normal(find(cross_primary_with_secondary(:,3)>0));
        dot_secondary_with_unit_normal_right = dot_secondary_with_unit_normal(find(cross_primary_with_secondary(:,3)<0));
        corridor_width_left = min(abs(dot_secondary_with_unit_normal_left));
        corridor_width_right = min(abs(dot_secondary_with_unit_normal_right)); % these are negative so switch to positive before taking min
        %% check for polytope walls
        % if there are no dot products, there's either nothing to that side, so the space is infinte
        % or there's a polytope to that side so the width is zero

        % first want to find if unit vector points left or right of primary vector
        primary_edge_mid_point = primary_edge_start + 0.5*primary_edge_dir; % find the middle of the edge
        cross_primary_with_normal = cross([primary_edge_dir 0],[unit_normal 0]);
        my_eps = 1e-7;
        if cross_primary_with_normal(3) > 0
            point_to_left = primary_edge_mid_point + my_eps*unit_normal;
            point_to_right = primary_edge_mid_point - my_eps*unit_normal;
        elseif cross_primary_with_normal(3) < 0
            point_to_right = primary_edge_mid_point + my_eps*unit_normal;
            point_to_left = primary_edge_mid_point - my_eps*unit_normal;
        else
            error('primary edge crossed with unit normal is 0')
            % just make the midpoint the point in this case
        end

        num_polys = length(polytopes);
        p = 1;
        is_in_left = 0;
        is_in_right = 0;
        while p <= num_polys
            verts = polytopes(p).vertices;
            % get xmin and xmax also ymin and ymax
            xmax = max(verts(:,1));
            xmin = min(verts(:,1));
            ymax = max(verts(:,2));
            ymin = min(verts(:,2));
            % check axis aligned bounding box before checking for polytope containment of the midpoint
            in_AABB = (primary_edge_mid_point(1) <= xmax && primary_edge_mid_point(1) >= xmin) && (primary_edge_mid_point(2) <= ymax && primary_edge_mid_point(2) >= ymin);
            % is point between xmin xmax and ymin max? if not continue
            if ~in_AABB
                p = p+1;
                continue
            end
            % if point is in AABB make polyshape from these verts
            polyshape_p = polyshape(verts);
            % is point in but not on polyshape?
            [is_in,is_on] = isinterior(polyshape_p,primary_edge_mid_point);
            % if so, remove the edge, and stop trying polytopes
            if is_on
                [is_in_left,is_on_left] = isinterior(polyshape_p,point_to_left);
                [is_in_right,is_on_right] = isinterior(polyshape_p,point_to_right);
                if is_in_left && is_in_right
                    error('the point is somehow left and right of the polytope')
                end
                % if it is in one polytope, we needn't check any others
                p = num_polys+1;
            end
            % if not, continue to check the next polytope
            p = p + 1;
        end

        if isempty(corridor_width_left)
            if is_in_left
                corridor_width_left = 0;
            else
                corridor_width_left = inf;
            end
        end
        if isempty(corridor_width_right)
            if is_in_right
                corridor_width_right = 0;
            else
                corridor_width_right = inf;
            end
        end
        dilation_robustness_matrix(edge_start_idx(i), edge_end_idx(i), 1) = corridor_width_left;
        dilation_robustness_matrix(edge_start_idx(i), edge_end_idx(i), 2) = corridor_width_right;
    end
end
