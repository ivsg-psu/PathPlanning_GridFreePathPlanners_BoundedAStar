function [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeDeadEnds(adjacency_matrix, triangle_chains, nodes, max_branching_factor)
% fcn_MedialAxis_removeDeadEnds
%
% This trims the medial axis graph, as made by fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains,
% removing dead ends (edges that branch off from the graph and do not rec-
% onnect anywhere).  This can be used by itself but is also wrapped by fcn_MedialAxis_pruneGraph.
% The user may wish
% to use this function to reduce graph size while still having an edge per gap between obstacles
% but the user may wish to skip using this function if having the graph cover a greater spatial ex-
% tent is desirable.
%
% FORMAT:
%
% [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeDeadEnds(adjacency_matrix, triangle_chains, nodes, max_branching_factor)
%
%
% INPUTS:
%
%    Useful variables for inputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    adjacency_matrix: Like the visibility graph but rather than indicating 2 nodes are visible,
%       it indicates 2 nodes are connected by an edge.
%       This an NxN matrix where N is the number of nodes in the map.
%       A 1 is in position i,j if node j is visible from point i.  0 otherwise.
%
%    triangle_chains: an Mx3 cell array with a row for each edge in the medial axis graph.  The first
%      column contains an int for the node ID for the start of the chain.  The second is the end node.
%      The third column is a 1xP_M array of integers representing IDs of the triangles whose circumcenters
%      form the "chain of triangles" connecting the two nodes. P_M can be different for each row, M.
%
%    nodes: a Nx1 array of integers.  The integers are the IDs of the triangles that are 3-connected,
%      i.e., their circumcenters are nodes in the medial axis graph.  The position in the nodes array
%      is the node ID and the value is the triangle ID.  E.g., if nodes(10)=146, then the 10th node
%      in the adjacency_matrix and triangle_chains struct is the 146th triangle in
%      the Delaunay triangulation.
%
%    max_branching_factor: a Nx1 array of integers.
%      branching factor is the number of nodes connected to each node. An asymm-
%      etric graph can have a different number of nodes connected by outbound edges and nodes conne-
%      cted by inbound edges so this is the maximum of these two numbers
%
% OUTPUTS:
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    adjacency_matrix: same size as the input adjacency matrix but nodes that are no longer adjacent
%      due to pruning have their 1's set to 0's
%
%    triangle_chains: same size as input triangle_chains data structure but edges that have been
%      pruned have an empty third column (i.e., there is no chain of triangles between the nodes)
%
%    nodes: same size as input nodes array but nodes that have been removed due to pruning are set
%      to NaN
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning* for examples of the script in use (it's wrapped by fcn_MedialAxis_pruneGraph).
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
    idx_1_connected_nodes = find(max_branching_factor == 1); % all one connected nodes are dead ends
    % remove the node from the adjacency matrix
    adjacency_matrix(idx_1_connected_nodes, :) = zeros(length(idx_1_connected_nodes), size(adjacency_matrix,1));
    adjacency_matrix(:, idx_1_connected_nodes) = zeros(size(adjacency_matrix,2), length(idx_1_connected_nodes));
    % remove the node from the node list
    nodes(idx_1_connected_nodes) = nan;
    % find triangle chains that start and end at this node
    idx_chain_starts_at_1_connected_node = find(ismember([triangle_chains{:,1}]', idx_1_connected_nodes));
    idx_chain_ends_at_1_connected_node = find(ismember([triangle_chains{:,2}]', idx_1_connected_nodes));
    % remove these triangle chains
    [triangle_chains{idx_chain_ends_at_1_connected_node,3}] = deal([]);
    [triangle_chains{idx_chain_starts_at_1_connected_node,3}] = deal([]);
end
