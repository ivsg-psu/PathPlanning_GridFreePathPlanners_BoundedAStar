function [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy)
% fcn_MedialAxis_processRoute
%
% This function takes the route, as output by Astar (i.e., a series of nodes)
% and turns it into what a human or vehicle would expect from the route (i.e.,
% a series of x-y locations representing the path to drive).  In vgraph planning,
% the nodes are the waypoints of the route because the nodes are connected by
% straight lines (the visibility graph).  Thus this function is not necessary.
% When using the medial axis graph for planning, the nodes are not simply connec-
% ted by straight lines, they are conected by the chains of triangle circum-
% centers so this function is necessary to transform a series of nodes into
% a series of x-y waypoints by identifying the necessary triangle circumcenters.
%
% FORMAT:
%
% [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy)
%
%
% INPUTS:
%
%    Useful variables for inputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    route: the matrix as produced by fcn_algorithm_Astar consisting of waypoints.  Each row is a
%       waypoint, and each column is x, y, and point ID
%
%    triangle_chains: an Mx5 cell array with a row for each edge in the medial axis graph.  The first
%      column contains an int for the node ID for the start of the chain.  The second is the end node.
%      The third column is a 1xP_M array of integers representing IDs of the triangles whose circumcenters
%      form the "chain of triangles" connecting the two nodes. P_M can be different for each row, M.
%      The 4th column contains the estimated corridor width (the minimum lateral free space a vehicle
%      would have when routing down the edge) and the 5th column contains the length of the edge.
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
%    xcc: Qx1 array of doubles.  The x positions of the circumcenters of the triangles.
%
%    ycc: Qx1 array of doubles.  The y positions of the circumcenters of the triangles.
%
%    start_xy: 1x2 double of the xy coordinates of the start point.  This will be prepended to the
%       route because the start_xy is note a node in the medial axis graph.
%
%    finish_xy: 1x2 double of the xy coordinates of the finish point.  This will be appended to the
%       route because the start_xy is note a node in the medial axis graph.
%
%   (optional arguments)
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
%    route_triangle_chain: the triangles whose circumcenters make up the x-y positions of the route.
%        This array has dimension 1x(R_P-2) as the start_xy and finish_xy are in the route but do
%        not come from a triangle circumcenter.
%
%    route_triangle_chain_ids: the rows in the triangle_chains struct indicating which edges from the
%        medial axis graph were assembled into the route.  This array has dimension 1x(R_N-1) as each
%        edge connected two nodes in the route
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
    % initialize storage
    route_triangle_chain = [];
    route_triangle_chain_ids = [];
    route_choke = inf; % the choke has infinite width until we know otherwise
    % going to expand the route one edge at a time
    for i = 1:(size(route,1)-1)
        % for route to route + 1 get tri chain representing the edge between these two nodes
        beg_seg = route(i,3);
        end_seg = route(i+1,3);
        idx_chain = find([triangle_chains{:,1}]'== beg_seg & [triangle_chains{:,2}]'== end_seg);
        % if there's none, error...this would happen if there was a discrepancy between the
        % adjacency matrix and the triangle chain structure
        if isempty(idx_chain)
            error('no triangle chain exists for this route segment')
        else
            % if there are valid connections, we want to take the best one because there could be
            % multiple edges connecting the same two nodes.  The edges may have different costs
            best_chain_idx = best_chain_idx_matrix(beg_seg,end_seg);
            % append the fond triangle chain to list of triangle chains
            route_triangle_chain_ids = [route_triangle_chain_ids, best_chain_idx];
            route_triangle_chain = [route_triangle_chain, triangle_chains{best_chain_idx,3}];
            % if this segment's worst choke point is narrower than the global worst choke point, overwrite
            segment_choke = triangle_chains{best_chain_idx,4};
            route_choke = min(route_choke, segment_choke);
        end
    end
    % deduplicate in cast triangle chains at the nodes were included twice (e.g., if I say go from
    % A to B and then go from B to C, you might write the route as A-B-B-C but you should have written
    % A-B-C)
    route_triangle_chain = unique(route_triangle_chain,'stable');
    % prepend and append the start_xy and finish_xy to the route as the
    % fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains function will create nodes on the
    % medial axis such that the arbitrary points (start_xy and finish_xy) can be joined to these nodes
    % by a direct line, pointing normal to the medial axis edge
    route_full = [start_xy; xcc(route_triangle_chain), ycc(route_triangle_chain); finish_xy];
    % calculate route length
    route_x = route_full(:,1);
    route_y = route_full(:,2);
    route_deltas = diff([route_x(:) route_y(:)]);
    route_length = sum(sqrt(sum(route_deltas.*route_deltas,2)));
end
