function [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeDeadEnds(adjacency_matrix, triangle_chains, nodes, max_branching_factor)
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
