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
%    Useful variables for inputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    adjacency_matrix: Like the visibility graph but rather than indicating 2 nodes are visible,
%       it indicates 2 nodes are connected by an edge.
%       This an NxN matrix where N is the number of nodes in the map.
%       A 1 is in position i,j if node j is visible from point i.  0 otherwise.
%
%    cgraph: the cost graph matrix. A cost matrix is an nxn matrix where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element i-j is the cost of routing from i to j.
%
%   all_pts: the point matrix of all nodes that the planner can consider, except the start and finish where
%       each row is a single point vector (x,y,id)
%
%   start: the start point vector (x,y,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,id)
%
%    best_chain_idx_matrix: Like the visibility graph but rather than indicating 2 nodes are visible,
%       it indicates the best (lowest cost) triangle chain (edge) connecting 2 nodes.
%       This an NxN matrix where N is the number of nodes in the map.
%       A 12 is in position i,j if the best triangle chain to the node in nodes(j) from the node in
%       nodes(i) is located in triangle_chains{12,:}. The reason this matrix exists is because there
%       may be multiple triangle chains connecting the same two nodes in the medial axis graph (unlike
%       in the visibility graph) so the planner needs to evaluate the cost from i to j by looking
%       at the optimal cost available.
%
%
% OUTPUTS:
%
%    adjacency_matrix: Like the visibility graph but rather than indicating 2 nodes are visible,
%       it indicates 2 nodes are connected by an edge.
%       This an NxN matrix where N is the number of nodes in the map.
%       A 1 is in position i,j if node j is visible from point i.  0 otherwise.
%
%    triangle_chains: an Mx5 cell array with a row for each edge in the medial axis graph.  The first
%      column contains an int for the node ID for the start of the chain.  The second is the end node.
%      The third column is a 1xP_M array of integers representing IDs of the triangles whose circumcenters
%      form the "chain of triangles" connecting the two nodes. P_M can be different for each row, M.
%      The 4th column contains the estimated corridor width (the minimum lateral free space a vehicle
%      would have when routing down the edge) and the 5th column contains the length of the edge.
%
%    nodes: a Nx1 array of integers.  The integers are the IDs of the triangles that are 3-connected,
%      i.e., their circumcenters are nodes in the medial axis graph.  The position in the nodes array
%      is the node ID and the value is the triangle ID.  E.g., if nodes(10)=146, then the 10th node
%      in the adjacency_matrix and triangle_chains struct is the 146th triangle in
%      the Delaunay triangulation.
%
%    xcc: Qx1 array of doubles.  The x positions of the circumcenters of the triangles.
%
%    ycc: Qx1 array of doubles.  The y positions of the circumcenters of the triangles.
%
%    start_closest_tri: the ID of the triangle whose circumcenter is closest to the start point
%
%    start_closest_node: the ID of the new node added to be is closest to the start point
%
%    finish_closest_tri: the ID of the triangle whose circumcenter is closest to the finish point
%
%    finish_closest_node: the ID of the new node added to be is closest to the finish point
%
%    w: scalar relative weighting of cost function, cost = w*length_cost + (1-w)*corridor width
%
%    min_corridor_width: scalar value of the minimum allowable corridor width.  Edges (triangle chains)
%      that have a smaller estimated corridor width will be filtered out
%
%    denylist_route_chain_ids: array of integer values of rows in triangle_chains that should not be
%       routeable.  This is useful for filtering out edges from the medial axis graph that you
%       do not wish to consider in path planning for arbitrary reasons such as not using edges
%       that have been previously used, manually marking edges block to "close a road" so to speak,
%       etc.of the minimum allowable corridor width.  Edges (triangle chains)
%
%
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning* for examples of the script in use.
%        script_test_voronoi_planning - basic example of medial axis planning
%        ||_alt_paths - example of generating several paths from the start to the finish using different corridors
%        ||_alt_paths_from_node - example of generating several paths from an arbitrary node to the finish using different corridors
%        ||_alt_paths_local - example of generating several paths from an each node along the initial route to the finish.  This
%                             script has a flag for which corridors are blocked on replanning: just the next segment in the
%                             initial route, the entire initial route, or all previously calculated routes (initial and alternate)
%        ||_hill - example of incorporating elevation into a medial axis graph.  This script is just a WIP demonstration
% See ../Documentation/medial_axis_planning.pptx for a flow chart of the medial axis/voronoi planning stack
%
% This function was written Spring 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024, Spring by Steve Harnett
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
        all_pts(i,:) = [xcc(nodes(i)), ycc(nodes(i)), i]; % use the circumcenter of triangle node(i) as the position
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
        % in case we want to not use triangle chains that were in previous routes or we noticed a route was blocked and want to manually denylist it
        idx_chain_rc = setdiff(idx_chain_rc, denylist_route_chain_ids);
        % if there are no matches meeting the start, goal, and min corridor width, set adjacency to zero and move on
        if isempty(idx_chain_rc)
            adjacency_matrix(r(i),c(i)) = 0;
            continue
        end
        % we want to only use the chain with the lowest total cost form r to c
        corridor_widths = [triangle_chains{idx_chain_rc, 4}]; % the corridor width of all valid chains
        lengths = [triangle_chains{idx_chain_rc, 5}]; % the length of all valid chains
        possible_costs = w*lengths + (1-w)*(corridor_widths).^(-1); % vectorized total cost (note we use inverse of width because width is good and length is bad)
        [min_cost, min_cost_location] = min(possible_costs); % the min cost is what we use as cost
        cgraph(r(i),c(i)) = min_cost;
        best_chain_idx_matrix(r(i),c(i)) = idx_chain_rc(min_cost_location); % need to remember which chain we want to use
    end
    %  set the start for the planner as the start node not the startxy
    start = [xcc(start_closest_tri) ycc(start_closest_tri) start_closest_node];
    finish = [xcc(finish_closest_tri) ycc(finish_closest_tri) finish_closest_node];
end
