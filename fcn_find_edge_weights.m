function weighted_edge_map = fcn_find_edge_weights(all_pts)
    % all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
    % pts : obs_id
    % want to form a matrix of 1 and 0 for visibility
    % could be done with this: https://github.com/davetcoleman/visibility_graph/blob/master/Visibility_Graph_Algorithm.pdf
    % or repeated calls to clear and blocked points
    % this gives all possible edges
    % edges weights : min(obs_id1 obs_id2)
    % x coord array of all x's
    % y coord array of all y's
end
