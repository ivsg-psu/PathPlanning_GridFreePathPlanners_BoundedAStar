[route_full, route_length, route_choke] = fcn_MedialAxis_replanWrapper(replan_point, finish, min_corridor_width, length_cost_weight);
% fcn_MedialAxis_replannerWrapper
%
% This function wraps the basic call stack to perform planning in thethe medial axis graph.
% The functions called herein are called individually in script_test_voronoi_planning.m
%
% FORMAT:
%
% [route_full, route_length, route_choke] = fcn_MedialAxis_plannerWrapper(polytope_vertices, start_xy, finish_xy, boundary_verts, min_corridor_width, length_cost_weight)
%
%
% INPUTS:
% polytope_vertices - 1xP cell array where P is the number of polytopes.
%   Each cell contains a Vx2 matrix of doubles where V is the number of vertices
%   in a polytope.  Column 1 contains x-values and column 2 contains y-values.
%
% start_xy - 1x2 vector of doubles defining the (x,y) coordinates of the start position
%
% finish_xy - 1x2 vector of doubles defining the (x,y) coordinates of the start position
%
% boundary_verts - Bx2 matrix containing the (x,y) coordinates of the B-vertices forming a boundary
%   around the polytope obstacles in the map.  The boundary does not need to contain the start or finish.
%   The boundary is necessary as the medial axis is defined as the furthest distance from the obstacles in the
%   domain of the free space, thus the medial axis would be infinitely far from the obstacles on the edge of the map
%   without a boundary.
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
%   fcn_check_reachability
%   fcn_algorithm_Astar
%   fcn_MedialAxis_processRoute
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning_interface for an example of the script in use.
%        script_test_voronoi_planning - shows the functions wrapped by fcn_MedialAxis_plannerWrapper in use independently
% See ../Documentation/medial_axis_planning.pptx for a flow chart of the medial axis/voronoi planning stack
%
% This function was written Oct 2024 by Steve Harnett
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
    flag_do_plot = 0;

    %% turn vertices cell array into polytopes struct array
    clear polytopes
    for poly = 1:length(polytope_vertices)
        polytopes(poly).vertices = polytope_vertices{poly};
        polytopes(poly).vertices = [polytopes(poly).vertices; polytopes(poly).vertices(1,:)]; % repeat first vert at end
    end
    boundary.vertices = boundary_verts;
    boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % repeat first vert at end
    boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
    shrunk_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes);
    shrunk_polytopes = [boundary, shrunk_polytopes]; % put the boundary polytope as the first polytope

    %% assume necessary triangulation resolution
    resolution_scale = 0.5;

    %% constrained delaunay triangulation
    [adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);

    %% prune graph
    [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, flag_do_plot);

    %% get costs for navigating each triangle chain
    % TODO add zcc as optional input
    [triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, flag_do_plot);

    %% planning through triangle graph
    % add start and finish to nearest medial axis edge
    % TODO add zcc as optional input
    [adjacency_matrix, triangle_chains, nodes, start_closest_tri, start_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(start_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
    [adjacency_matrix, triangle_chains, nodes, finish_closest_tri, finish_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(finish_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);

    % loop over cost function weights
    denylist_route_chain_ids = []; % no need to denylist any triangle chains
    % make cost matrix
    [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, length_cost_weight, min_corridor_width, denylist_route_chain_ids);
    % adjacency matrix is vgraph
    vgraph = adjacency_matrix;
    num_nodes = length(nodes);
    vgraph(1:num_nodes+1:end) = 1;
    % check reachability
    [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
    if ~is_reachable
        error('start and finish are not connected in medial axis graph')
    end
    % run Dijkstra's algorithm (no heuristic Astar)
    hvec = zeros(1,num_nodes);

    % plan a path
    [cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

    % expand route nodes into actual path
    [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);
end % end function

