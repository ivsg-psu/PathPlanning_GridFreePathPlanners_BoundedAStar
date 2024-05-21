function [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids)
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
