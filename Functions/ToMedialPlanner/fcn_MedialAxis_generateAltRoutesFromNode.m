function [alternate_routes, alternate_routes_nodes, alternate_routes_chain_ids, smallest_corridors, route_lengths] = fcn_MedialAxis_generateAltRoutesFromNode(idx_of_start_node, adjacency_matrix, triangle_chains, nodes, xcc, ycc, finish_xy, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);
% fcn_MedialAxis_generateAltRoutesFromNode
%
% This function wraps several functions from the voronoi/medial axis graph planning workflow
% to quickly and easily generate alternate routes branching off of a node on an existing route.
% Nominally, the medial axis graph nodes are 3 connected so this returns 3 paths leaving a given
% node, all going to the same finish.  However, there may be fewer routes if a less connected node
% is given as a start or if denylisting prevents certain avenues from having a possible route.
%
% FORMAT:
%
% [alternate_routes, alternate_routes_nodes, alternate_routes_chain_ids, smallest_corridors, route_lengths] = ...
% fcn_MedialAxis_generateAltRoutesFromNode(idx_of_start_node, adjacency_matrix, triangle_chains,...
%                                            nodes, xcc, ycc, finish_xy, finish_closest_tri,...
%                                            finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);
%
%
% INPUTS:
%
%    Useful variables for inputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation, N_R - number of alt routes (usually 3)
%       R_P - number of xy points in the route.  R_N - number of nodes in the route.
%       R_N < R_P
%
%    idx_of_start_node: integer ID of node to start generating alternate routes from.  This is not the
%      triangle ID, i.e., this is the positon of the node in the 'nodes' array, not the value.
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
%    finish_xy: 1x2 double of the xy coordinates of the finish point.  This will be appended to the
%       route because the start_xy is note a node in the medial axis graph.
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
% OUTPUTS:
%
%
%    alternate_routes: 1xN_R cell array of the alternate routes.  Each cell contains a 2-column matrix.
%        The R_Px2 series of R_P-points making up the route where the first column is the x-coordinate
%        and the second column is the y-coordinate. Each route (cell) may have a different R_P.
%
%    alternate_routes_nodes: 1xN_R cell array where each cell contains the matrix as produced by
%       fcn_algorithm_Astar consisting of nodes.  The matrix is R_Nx3.  Each row is a
%       node, and each column is x, y, and point ID.  Each route (cell) may have a different R_N.
%
%    alternate_routes_chain_ids: 1xN_R cell array where each cell contains the triangle chain
%        IDs for a single alt route.  The rows in the triangle_chains struct indicating which edges from the
%        medial axis graph were assembled into the route.  Each cell contains an array that has dimension 1x(R_N-1) as each
%        edge connected two nodes in the route.  Each route (cell) may have a different R_N.
%
%    smallest_corridors: 1xN_R double representing the narrowest estimated corridor width along each route
%
%    route_lengths: 1xN_R double representing the length of each alt. route from the start node to finish_xy along
%        the medial axis graph edges.  Note this does not include the distance along the initial route until the
%        node where the replanning (alternate route generation) was triggered from.  I.e., this is the distance
%        required to finish each alternate route, not the total distance from global start to global finish along
%        each alternate route.
%
% DEPENDENCIES:
%
%   fcn_MedialAxis_makeCostGraphAndAllPoints
%   fcn_BoundedAStar_checkReachability
%   fcn_algorithm_Astar
%   fcn_MedialAxis_processRoute
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
% 2025_10_06 - S. Brennan
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability

% TO DO:
%
% -- fill in to-do items here.

    % data for all routes
    alternate_routes = {};
    alternate_routes_nodes = {}; % ids of the nodes in the each route
    alternate_routes_chain_ids = {}; % ids of the edges in each route
    smallest_corridors = [];
    route_lengths = [];

    start_closest_node = idx_of_start_node;
    start_closest_tri = nodes(idx_of_start_node);
    start_xy = [xcc(start_closest_tri) ycc(start_closest_tri)];
    % initialize storing data on replan timeliness
    replanning_times = [];
    % find list of edges leaving node
    idx_chain_leaving_node = find([triangle_chains{:,1}]'== idx_of_start_node);
    possible_ids = 1:length(idx_chain_leaving_node); % might be 1,2,3
    for iterations = 1:length(idx_chain_leaving_node)
        replanning_time = tic;
        if isempty(triangle_chains{idx_chain_leaving_node(iterations),3})
            continue
        end
        ids_to_denylist = setdiff(possible_ids,iterations); % so at step 1 this should be 2,3
        denylist_route_chain_ids_incl_chains_leaving_node = [denylist_route_chain_ids; idx_chain_leaving_node(ids_to_denylist)];% so at step 1, we would block 2, 3, plus whatever user input denlylist we're given

        [adjacency_matrix_small, cgraph, all_pts, start, finish, best_chain_idx_matrix_small] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids_incl_chains_leaving_node);

        % adjacency matrix is vgraph
        vgraph = adjacency_matrix_small;
        num_nodes = length(nodes);
        vgraph(1:num_nodes+1:end) = 1;
        % check reachability
        [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
        if ~is_reachable
            % if this iteration is not possible, issue a warning and try again
            my_warn = sprintf('alternate route %i planning not possible',iterations);
            warning(my_warn)
            %% update for next iteration of alt route
            alternate_routes_nodes{end+1}  = nan;
            alternate_routes_chain_ids{end+1}  = nan;
            alternate_routes{end+1}  = nan;
            smallest_corridors = [smallest_corridors, nan];
            route_lengths = [route_lengths, nan];
            iterations = iterations+ 1;
            continue
        end
        % run Dijkstra's algorithm (no heuristic)
        hvec = zeros(1,num_nodes);

        % plan a path
        [cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

        [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix_small, xcc, ycc, start_xy, finish_xy);

        replanning_times = [replanning_times, toc(replanning_time)]


        %% update for next iteration of alt route
        alternate_routes_nodes{end+1}  = route;
        alternate_routes_chain_ids{end+1}  = route_triangle_chain_ids;
        alternate_routes{end+1}  = route_full;
        smallest_corridors = [smallest_corridors, route_choke];
        route_lengths = [route_lengths, route_length];
        iterations = iterations+ 1;
    end % end loop over departing edges
    num_routes = length(alternate_routes);
    route_combos = nchoosek(1:num_routes,2);
    duplicate_routes_idx = [];
    for i = 1:size(route_combos,1)
        if isequal(alternate_routes{route_combos(i,1)}, alternate_routes{route_combos(i,2)})
            duplicate_routes_idx = [duplicate_routes_idx, route_combos(i,2)];
        end
    end
    for i = 1:length(duplicate_routes_idx)
        alternate_routes_nodes(duplicate_routes_idx(i)) = [];
        alternate_routes_chain_ids(duplicate_routes_idx(i))  = [];
        alternate_routes(duplicate_routes_idx(i)) = [];
        smallest_corridors(duplicate_routes_idx(i)) = [];
        route_lengths(duplicate_routes_idx(i)) =[];
    end
end % end function
