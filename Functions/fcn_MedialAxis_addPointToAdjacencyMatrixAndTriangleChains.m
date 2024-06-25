function [adjacency_matrix, triangle_chains, nodes, point_closest_tri, point_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(point_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri)
% fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains
%
% This function is useful for adding arbitrary points (such as a start or finish)
% to the medial axis graph (composed of an adjacency matrix and the
% triangle_chains structure describing the edges).  The function places a node at
% the nearest point on the medial axis to the arbitrary point.  I.e., the closest
% edge to the new point is identified and a node is placed along that edge such
% that it is closest along the existing edge to the new point.
% This node can then be attached to the arbitrary point via a straight line which
% extends normally from the medial axes.  This creates a scenario where, if the
% vehicle starts at the new point, it can drive straight to the new node
% in the shortest distance possible to center in the corridor and route along
% medial axes.
%
%
% FORMAT:
%
% [adjacency_matrix, triangle_chains, nodes, point_closest_tri, point_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(point_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri)
%
%
% INPUTS:
%
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    point_xy: 1x2 double of the xy coordinates of the point to add to the medial axis graph
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
%    max_side_lengths_per_tri: a Qx1 array of doubles.  The double is the length of the shortest
%      side of each triangle.
%
%
% OUTPUTS:
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    adjacency_matrix: same as the input but with an additional row and column for the added node
%
%    triangle_chains: same as the input but with 4 additional tri chains to connect the added node
%      which will be midway along an existing edge, the original 2 ends of that edge. I.e., if the
%      nearest edge to new point k is edge i-j, tri chains will be added for:
%        - i-k
%        - k-j
%        - k-i
%        - j-k
%
%    nodes: same as the input but with an additional triangle added as a node, selected such that
%      it is added along an existing edge at the location on the
%      medial axis graph closest to the arbitrary point.
%
%    point_closest_tri: the ID of the triangle whose circumcenter is closest to the input point
%
%    point_closest_node: the ID of the new node added to be is closest to the input point
%
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning* for examples of the script in use.
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
    % don't want to look at all triangles in constrained triangulation, TR as not all of them are medial axis edges
    tris_in_graph = unique([triangle_chains{:,3}]');
    % find distance from arbitrary point to circumcenters of all triangles in graph
    point_delta_from_all_tris = point_xy - [xcc(tris_in_graph), ycc(tris_in_graph)];
    point_dist_from_all_tris = (point_delta_from_all_tris(:,1).^2 + point_delta_from_all_tris(:,2).^2).^0.5;
    [~, point_closest_tri_loc] = min(point_dist_from_all_tris); % find closest circumcenter
    point_closest_tri = tris_in_graph(point_closest_tri_loc); % note the closest triangle circumcenter
    % find which chains the closest tri is in (should really only be 2)
    idx_chains_containing_point = [];
    for i = 1:(size(triangle_chains,1))
        % pop off a triangle chain
        chain_of_note = triangle_chains{i,3};
        if ismember(point_closest_tri, chain_of_note)
            idx_chains_containing_point = [idx_chains_containing_point i];
        end
    end
    % make point closest tri a node
    nodes = [nodes; point_closest_tri];
    point_closest_node = find(nodes == point_closest_tri);
    % make a new adjacency matrix row and column for the point triangle
    adjacency_matrix = [adjacency_matrix, zeros(size(adjacency_matrix,2),1); zeros(1,size(adjacency_matrix,1)+1)];
    for i = 1:length(idx_chains_containing_point)
        % pop off the triangle chain containing the point triangle
        first_node = triangle_chains{idx_chains_containing_point(i),1};
        last_node = triangle_chains{idx_chains_containing_point(i),2};
        chain_of_note = triangle_chains{idx_chains_containing_point(i),3};
        % find where the point triangle is in the chain
        point_tri_location = find(chain_of_note == point_closest_tri);
        % make two new chains from beginning to point tri and point tri to end
        first_chain = chain_of_note(1:point_tri_location);
        last_chain = chain_of_note(point_tri_location:end);
        triangle_chains{end+1,1} = first_node;
        triangle_chains{end,2} = point_closest_node;
        triangle_chains{end,3} = first_chain;
        triangle_chains{end,4} = min(max_side_lengths_per_tri(first_chain)); % add min corridor width
        % the cumulative length of all the distance between circumcenters is the triangle chain length
        delta_x_and_y = diff([xcc(first_chain) ycc(first_chain)]);
        triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
        triangle_chains{end,5} = triangle_chain_length;
        triangle_chains{end+1,1} = point_closest_node;
        triangle_chains{end,2} = last_node;
        triangle_chains{end,3} = last_chain;
        triangle_chains{end,4} = min(max_side_lengths_per_tri(last_chain)); % add min corridor width
        % the cumulative length of all the distance between circumcenters is the triangle chain length
        delta_x_and_y = diff([xcc(last_chain) ycc(last_chain)]);
        triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
        triangle_chains{end,5} = triangle_chain_length;
        % add the new chains to adjacency
        adjacency_matrix(point_closest_node,point_closest_node) = 1;
        adjacency_matrix(first_node,point_closest_node) = 1;
        adjacency_matrix(point_closest_node,last_node) = 1;
    end
end
