function [dilation_robustness_matrix] = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode)
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
%
% TO DO:
%
% -- fill in to-do items here.
    vgraph = vgraph - eye(size(vgraph)); % we don't want to consider self interactions here
    all_pts = [all_pts; start; finish];
    % initialize to zero value for each edge (zero implying the edge has no corridor width, i.e. is blocked)
    dilation_robustness_matrix = zeros(size(vgraph));
    % only need to perform this operation for 1's in vgraph, 0's are blocked edges and have 0 corridor width
    idx_of_valid_edges = find(vgraph==1);
    num_edges = length(idx_of_valid_edges);
    % recall vgrpah edge ij is the line segment from point to point j
    [edge_start_idx, edge_end_idx] = ind2sub(size(vgraph),idx_of_valid_edges);

    for i = 1:num_edges
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            edge_start = all_pts(edge_start_idx(i),1:2);
            edge_end = all_pts(edge_end_idx(i),1:2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            edge_start = all_pts(edge_start_idx(i),1:3);
            edge_end = all_pts(edge_end_idx(i),1:3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        % find edge direction vector
        edge_dir = edge_end - edge_start;
        if edge_dir(2) ~= 0
            normal_dir(1) = 1;
            normal_dir(2) = -(edge_dir(1)*normal_dir(1))./edge_dir(2);
        elseif edge_dir(1) ~=0
            normal_dir(2) = 1;
            normal_dir(1) = -(edge_dir(2)*normal_dir(2))./edge_dir(1);
        else
            error("visibility graph edge has zero length")
        end
        normal_mag = norm(normal_dir);
        unit_normal = normal_dir/normal_mag;
        % find all visibility graph edges with same origin
        other_edge_ends_idx = find(vgraph(edge_start_idx(i), :)==1);
        other_edge_ends_idx(find(other_edge_ends_idx==edge_end_idx(i))) = [];
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            other_edge_ends = all_pts(other_edge_ends_idx,1:2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            other_edge_ends = all_pts(other_edge_ends_idx,1:3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        % this line removes the currently considered edge end from the other edge end list
        other_edge_dirs = other_edge_ends - edge_start;
        % we want to discard edges that either have no component in the direction of the original vector
        % or that end at a point too far away to cut off the original vector
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            dot_products = other_edge_dirs(:,1)*edge_dir(1) + other_edge_dirs(:,2)*edge_dir(2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            dot_products = other_edge_dirs(:,1)*edge_dir(1) + other_edge_dirs(:,2)*edge_dir(2) + other_edge_dirs(:,3)*edge_dir(3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        other_edge_projected_to_edge = dot_products/norm(edge_dir);
        other_edge_dirs(find(other_edge_projected_to_edge <= 0 | other_edge_projected_to_edge > norm(edge_dir)),:) = [];

        % dot the other edges with the unit normal to find the corridor width defined by each vgraph edge
        if strcmp(mode, "2d") || strcmp(mode,"2D")
            dot_products = other_edge_dirs(:,1)*unit_normal(1) + other_edge_dirs(:,2)*unit_normal(2);
        elseif strcmp(mode, "3d") || strcmp(mode,"3D")
            dot_products = other_edge_dirs(:,1)*unit_normal(1) + other_edge_dirs(:,2)*unit_normal(2) + other_edge_dirs(:,3)*unit_normal(3);
        else
            error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
        end
        corridor_width = min(abs(dot_products));
        if isempty(corridor_width)
            corridor_width = inf;
        end
        dilation_robustness_matrix(edge_start_idx(i), edge_end_idx(i)) = corridor_width;
    end
end
