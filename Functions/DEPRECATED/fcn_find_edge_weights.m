function [cost_matrix, visibility_matrix_original] = fcn_find_edge_weights(polytopes, all_pts, gap_size)
warning('on','backtrace');
warning(['fcn_find_edge_weights is being deprecated. ' ...
    'Use fcn_BoundedAStar_findEdgeWeights instead.']);
    % WARNING WORK IN PROGRESS FUNCTION
    % TODO if the visibility matrix is reduced, this should be modified to
    % find the min cost, of the two polytopes that are both members of pt 1
    % and pt 2
    % all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
    % pts : obs_id
    % want to form a matrix of 1 and 0 for visibility
    visibility_matrix = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts, gap_size);
    visibility_matrix_original = visibility_matrix;
    num_points = size(all_pts,1);
    % for each 1 in the visibility matrix...
    [r, c] = find(visibility_matrix==1);
    % find the point pairs that are visible to each other
    first_pts = all_pts(r,:);
    second_pts = all_pts(c,:);
    % only want to keep costs polys if BOTH points are on the poly, not one
    % if only one point in the pair belongs to a certain polytope,
    % this means the edge does not go through or along that polytope
    % thus that edge cost cannot be used
    % same_poly_edges = first_pts(:,4)==second_pts(:,4);
    first_pts_redux = first_pts%(first_pts(:,4)==second_pts(:,4),:);
    second_pts_redux = second_pts%(first_pts(:,4)==second_pts(:,4),:);
    % find the corresponding polytopes
    first_polys = polytopes(first_pts_redux(:,4));
    second_polys = polytopes(second_pts_redux(:,4));
    % find the corresponding costs
    first_traversal_costs = extractfield(first_polys,'cost');
    second_traversal_costs = extractfield(second_polys,'cost');
    % edges weights will be the minimum of the obstacles it spans
    min_traversal_costs = min(first_traversal_costs,second_traversal_costs);
    % explanation of the following line: https://www.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html
    idx = sub2ind(size(visibility_matrix), r, c);
    visibility_matrix(idx) = min_traversal_costs;
    cost_matrix = visibility_matrix;
end
