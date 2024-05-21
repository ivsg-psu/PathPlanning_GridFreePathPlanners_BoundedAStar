function [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy)
    route_triangle_chain = [];
    route_triangle_chain_ids = [];
    route_choke = inf;
    for i = 1:(size(route,1)-1)
        % for route to route + 1 get tri chain
        beg_seg = route(i,3);
        end_seg = route(i+1,3);
        idx_chain = find([triangle_chains{:,1}]'== beg_seg & [triangle_chains{:,2}]'== end_seg);
        % if there's none, error
        if isempty(idx_chain)
            error('no triangle chain exists for this route segment')
        % elseif length(idx_chain) == 1
        %     route_triangle_chain = [route_triangle_chain, triangle_chains{chain_idx,3}];
        %     % coule extract length and min width here
        % if there's two take best
        % if there's one, take it
        else
            best_chain_idx = best_chain_idx_matrix(beg_seg,end_seg);
            % append to list of triangle chains
            route_triangle_chain_ids = [route_triangle_chain_ids, best_chain_idx];
            route_triangle_chain = [route_triangle_chain, triangle_chains{best_chain_idx,3}];
            segment_choke = triangle_chains{best_chain_idx,4};
            route_choke = min(route_choke, segment_choke);
        end
    end
    % dedup
    route_triangle_chain = unique(route_triangle_chain,'stable');
    % append the straightline from startxy to start node to the beginning of the route when transforming the route to tri chains
    route_full = [start_xy; xcc(route_triangle_chain), ycc(route_triangle_chain); finish_xy];
    route_x = route_full(:,1);
    route_y = route_full(:,2);
    route_deltas = diff([route_x(:) route_y(:)]);
    route_length = sum(sqrt(sum(route_deltas.*route_deltas,2)));
end
