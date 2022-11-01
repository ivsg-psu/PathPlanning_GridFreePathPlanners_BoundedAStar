function cost_matrix = fcn_find_edge_weights(polytopes, all_pts)
    % all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
    % pts : obs_id
    % want to form a matrix of 1 and 0 for visibility
    visibility_matrix = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts);
    num_points = size(all_pts,1)
    % for each 1 in the visibility matrix...
    [r, c] = find(visibility_matrix==1);
    % find the point pairs that are visible to each other
    first_pts = all_pts(r,:);
    second_pts = all_pts(c,:);
    % find the corresponding polytopes
    first_polys = polytopes(first_pts(:,4));
    second_polys = polytopes(first_pts(:,4));
    % find the corresponding costs
    first_traversal_costs = extractfield(first_polys,'cost');
    second_traversal_costs = extractfield(second_polys,'cost');
    % edges weights will be the minimum of the obstacles it spans
    min_traversal_costs = min(first_traversal_costs,second_traversal_costs);
    idx = sub2ind(size(visibility_matrix), r, c);
    visibility_matrix(idx) = min_traversal_costs;
end
