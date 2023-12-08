function cgraph = fcn_algorithm_generate_cost_graph(all_pts, start, finish, dimensions, mode, visibility_weight, reachability_weight)
    % generate matrix of costs from each point to each other point
    % dimension - '2d' or '3d' to indicate if each point has 2 or three dimensions
    % mode indicates how the cost should be expressed and can be "2d spatial", "time or z", "3d spatial"
    % visibility_weight is the relative weighting of the visibility term
    % reachability_weight is the relative weighting of the reachability term

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];
    xs = all_pts_plus_start_and_fin(:,1); % vector of all x coords
    ys = all_pts_plus_start_and_fin(:,2); % vector of all y coords
    zs = all_pts_plus_start_and_fin(:,3); % vector of all y coords
    if mode == "3d spatial"
        cgraph = sqrt((xs - xs').^2 + (ys - ys').^2 + (zs - zs').^2)'; % distance of every pt from all other pts
    elseif mode == "time or z"
        cgraph = sqrt((zs - zs').^2)'; % distance of every pt from all other pts
    elseif mode == "2d spatial"
        cgraph = sqrt((xs - xs').^2 + (ys - ys').^2)'; % distance of every pt from all other pts
    else
        error("mode must be either 3d spatial, time or z, or 2d spatial \n you may make a cost matrix manually as well")
    end
end % end function
