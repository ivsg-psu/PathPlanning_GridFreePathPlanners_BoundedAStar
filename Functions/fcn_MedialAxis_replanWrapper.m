function [route_full, route_length, route_choke] = fcn_MedialAxis_replanWrapper(replan_point, finish_xy, min_corridor_width, length_cost_weight)
% fcn_MedialAxis_replannerWrapper
%
% This function wraps the basic call stack to perform replanning in the medial axis graph.
% This function would typically be called after calling fcn_MedialAxis_plannerWrapper, which
% creates, prunes, and plans in the medial axis.  fcn_MedialAxis_replanWrapper does not form a medial
% axis graph, rather it replans in an existing medial axis graph, while blocking the currently occupied
% edge which is assumed to contain an issue rendering the remainder of the path infeasible.
% This function uses global variables to allow the medial axis graph and the denylist of blocked
% triangle chains to persist from the call to fcn_MedialAxis_plannerWrapper
%
% The functions called herein are called individually in script_test_voronoi_planning.m
%
% FORMAT:
%
% [route_full, route_length, route_choke] = fcn_MedialAxis_replanWrapper(replan_point, finish_xy, min_corridor_width, length_cost_weight)
%
%
% INPUTS:
%
% replan_point - 1x2 vector of doubles defining the (x,y) coordinates of the position from which to replan
%
% finish_xy - 1x2 vector of doubles defining the (x,y) coordinates of the start position
%
% min_corridor_width - double value of the narrowest corridor that should be routed through by the planner
%   setting to 0 will use all available corridors without restriction
% length_cost_weight - scalar relative weighting of cost function, cost = w*length_cost + (1-w)*corridor width
%   setting to 1 gives minimum distance path
%
% OUTPUTS:
%    Useful variables for outputs: R_P - number of xy points in the route.  R_N - number of nodes in the route.
%       R_N < R_P
%
%    route_full: the R_Px2 series of R_P-points making up the route where the first column is the x-coordinate
%        and the second column is the y-coordinate.
%
%    route_length: double representing the length of the full route from start_xy to finish_xy along
%        the medial axis graph edges
%
%    route_choke: double representing the narrowest estimated corridor width along the route
%
%
% DEPENDENCIES:
%   fcn_MapGen_fillPolytopeFieldsFromVertices
%   fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains
%   fcn_MedialAxis_pruneGraph
%   fcn_MedialAxis_addCostsToTriangleChains
%   fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains
%   fcn_MedialAxis_makeCostGraphAndAllPoints
%   fcn_BoundedAStar_checkReachability
%   fcn_algorithm_Astar
%   fcn_MedialAxis_processRoute
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning_interface for an example of the script in use.
%        script_test_voronoi_planning - shows the functions wrapped by fcn_MedialAxis_replanWrapper in use independently
% See ../Documentation/medial_axis_planning.pptx for a flow chart of the medial axis/voronoi planning stack
%
% This function was written Nov 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024, November by Steve Harnett
% -- first write of function
% 2025_10_06 - S. Brennan
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability

% TO DO:
%
% -- fill in to-do items here.
    flag_do_plot = 0;

    global medial_axis_graph; % grab the global medial axis graph so we don't have to recalculate it
    if isempty(medial_axis_graph)
        error('Medial axis graph is not created yet.  Please call fcn_MedialAxis_plannerWrapper before calling fcn_MedialAxis_replanWrapper.')
    end

    global denylist_route_chain_ids % allow the denylist to persist and grow between calls so previously blocked edges are not reused

    % unpack medial axis graph struct
    adjacency_matrix = medial_axis_graph.adjacency_matrix;
    triangle_chains = medial_axis_graph.triangle_chains;
    max_side_lengths_per_tri = medial_axis_graph.max_side_lengths_per_tri;
    nodes = medial_axis_graph.nodes;
    xcc = medial_axis_graph.xcc;
    ycc = medial_axis_graph.ycc;
    tr = medial_axis_graph.tr;
    old_route = medial_axis_graph.route;
    old_route_triangle_chain = medial_axis_graph.route_triangle_chain;
    old_route_triangle_chain_ids = medial_axis_graph.route_triangle_chain_ids;

    start_xy = replan_point; % the new start is the replanning point

    [adjacency_matrix, triangle_chains, nodes, start_closest_tri, start_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(start_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);

    % this line is weird: the last 4 triangle chains  in the cell array will go from the replanning
    % point to two existing nodes
    nodes_near_replanning_point_incl_replan_point = unique([triangle_chains{end-3:end,1:2}]);
    % we can ditch the highest node ID as this is the replanning point, keeping only the two
    % nodes that were bookending the replanning point
    nodes_near_replanning_point = nodes_near_replanning_point_incl_replan_point(1:2);
    replanning_node = nodes_near_replanning_point_incl_replan_point(3);
    % then we want to prevent planning to that node
    idx_of_unreached_node_in_route = max(find(ismember(old_route(:,3),nodes_near_replanning_point)));
    idx_of_departed_node_in_route = min(find(ismember(old_route(:,3),nodes_near_replanning_point)));
    unreached_node = old_route(idx_of_unreached_node_in_route,3); % node at end of corridor containing replanning point
    departed_node = old_route(idx_of_departed_node_in_route,3); % node at beginning of corridor containing replanning point

    % the edge that we couldn't route is the edge representing the corridor containing the replanning point
    edge_that_failed = find([triangle_chains{:,1}]' == departed_node & [triangle_chains{:,2}]' == unreached_node);

    % the edge from the replanning point to the end of that corridor is likely to have the same issue as the above edge
    edge_that_will_fail = find([triangle_chains{:,1}]' == replanning_node & [triangle_chains{:,2}]' == unreached_node);

    % TODO do we need to remove the backwards version of the above two edges as well? Recall this is a directed graph
    % edge_that_failed_bw = find([triangle_chains{:,2}]' == departed_node & [triangle_chains{:,1}]' == unreached_node);
    % edge_that_will_fail_bw = find([triangle_chains{:,2}]' == replanning_node & [triangle_chains{:,1}]' == unreached_node);
    % denylist_route_chain_ids = [edge_that_failed, edge_that_will_fail, edge_that_failed_bw, edge_that_will_fail_bw]

    % need to note the problem edge in the denylist but be sure to retain alread denylisted nodes
    if isempty(denylist_route_chain_ids)
        denylist_route_chain_ids = [edge_that_failed, edge_that_will_fail];
    else
        denylist_route_chain_ids = [denylist_route_chain_ids, edge_that_failed, edge_that_will_fail];
    end

    [adjacency_matrix, triangle_chains, nodes, finish_closest_tri, finish_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(finish_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
    % loop over cost function weights
    % make cost matrix
    [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, length_cost_weight, min_corridor_width, denylist_route_chain_ids);

    %% adjacency matrix is vgraph
    vgraph = adjacency_matrix;
    num_nodes = length(nodes);
    vgraph(1:num_nodes+1:end) = 1;
    % check reachability
    [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
    if ~is_reachable
        error('start and finish are not connected in medial axis graph')
    end
    % run Dijkstra's algorithm (no heuristic Astar)
    hvec = zeros(1,num_nodes);

    % plan a path
    [cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

    % expand route nodes into actual path
    [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);

    % set globals:
    medial_axis_graph.route = route;
    medial_axis_graph.route_triangle_chain = route_triangle_chain;
    medial_axis_graph.route_triangle_chain_ids = route_triangle_chain_ids;


end % end function
