function [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids)
% fcn_MedialAxis_makeCostGraphAndAllPoints
%
% This function turns the medial axis graph (the adjacency matrix and triangle chains
% structure) into a cost graph (cgraph) and all points table (all_pts) for use in
% planning with Astar or Bounded Astar
%
%
% FORMAT:
% [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids)
%
%
% INPUTS:
%
%   start: the start point vector (x,y,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,id)
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%    cgraph: the cost graph matrix. A cost matrix is an nxn matrix where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element i-j is the cost of routing from i to j.
%
%    hvec: the heuristic cost vector. A 1xn vector where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element k is the estimated cost of routing from point k to
%      the finish based on a heuristic cost estimation method.
%
% OUTPUTS:
%
%     cost: the total cost of the selected route
%
%    route: the matrix as produced by fcn_algorithm_Astar3d consisting of waypoints.  Each row is a
%    waypoint, and each column is x, y, and point ID
%
%
% DEPENDENCIES:
%
% none but several functions exist to create visibility matrices and fcn_algorithm_generate_cost_graph can create cost matrices (cgraph) and heuristic cost vectors (hvec)
%
% EXAMPLES:
%
% See the script: script_test_fcn_algorithm_Astar
% for a full test suite.
%
% This function was written on spring 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, spring by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.

    num_nodes = length(nodes);
    all_pts = nan(num_nodes,3);
    for i = 1:num_nodes
        if isnan(nodes(i))
            continue
        end
        all_pts(i,:) = [xcc(nodes(i)), ycc(nodes(i)), i];
    end
    %% form cost graph from triangle_chains
    % cost is of the form: total cost = w*length + (1-w)*corridor_width
    cgraph = nan(size(adjacency_matrix)); % initialize cgraph
    % since there can be multiple chains between two nodes, we need to note which one we are using
    best_chain_idx_matrix = nan(size(adjacency_matrix));
    % for every one in the adjacency matrix, i.e., every connected pair of nodes
    [r, c] = find((adjacency_matrix));
    for i = 1:length(r)
        % if this is the self adjacent node...
        if r(i) == c(i)
            cgraph(r(i),c(i)) = 0; % it's always free to stay still
            continue
        end
        % find all the chains connecting r and c in adjacency that also meet minimum corridor width requirement
        idx_chain_rc = find([triangle_chains{:,1}]'== r(i) & [triangle_chains{:,2}]'== c(i) & [triangle_chains{:,4}]' > min_corridor_width);
        % also need to allow for filtering on banned chains
        idx_chain_rc = setdiff(idx_chain_rc, denylist_route_chain_ids); % want to not use triangle chains that were in previous routes
        % if there are no matches meeting the start, goal, and min corridor width, set adjacency to zero and move on
        if isempty(idx_chain_rc)
            adjacency_matrix(r(i),c(i)) = 0;
            continue
        end
        % we want to only use the chain with the lowest total cost form r to c
        corridor_widths = [triangle_chains{idx_chain_rc, 4}]; % the corridor width of all valid chains
        lengths = [triangle_chains{idx_chain_rc, 5}]; % the length of all valid chains
        possible_costs = w*lengths + (1-w)*(corridor_widths).^(-1); % vectorized total cost
        [min_cost, min_cost_location] = min(possible_costs); % the min cost is what we use as cost
        cgraph(r(i),c(i)) = min_cost;
        best_chain_idx_matrix(r(i),c(i)) = idx_chain_rc(min_cost_location); % need to remember which chain we want to use
    end

    %  set the start for the planner as the start node not the startxy
    start = [xcc(start_closest_tri) ycc(start_closest_tri) start_closest_node];
    finish = [xcc(finish_closest_tri) ycc(finish_closest_tri) finish_closest_node];
end
