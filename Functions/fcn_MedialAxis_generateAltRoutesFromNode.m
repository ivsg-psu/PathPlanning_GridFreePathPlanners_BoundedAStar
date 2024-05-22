function [alternate_routes, alternate_routes_nodes, alternate_routes_chain_ids, smallest_corridors, route_lengths] = fcn_MedialAxis_generateAltRoutesFromNode(idx_of_start_node, adjacency_matrix, triangle_chains, nodes, xcc, ycc, finish_xy, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);

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
        ids_to_denylist = setdiff(possible_ids,iterations); % so at step 1 this should be 2,3
        denylist_route_chain_ids_incl_chains_leaving_node = [denylist_route_chain_ids, idx_chain_leaving_node(ids_to_denylist)];% so at step 1, we would block 2, 3, plus whatever user input denlylist we're given

        [adjacency_matrix_small, cgraph, all_pts, start, finish, best_chain_idx_matrix_small] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids_incl_chains_leaving_node);

        % adjacency matrix is vgraph
        vgraph = adjacency_matrix_small;
        num_nodes = length(nodes);
        vgraph(1:num_nodes+1:end) = 1;
        % check reachability
        [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
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
end % end function
