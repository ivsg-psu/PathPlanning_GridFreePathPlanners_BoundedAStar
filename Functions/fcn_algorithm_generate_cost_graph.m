function [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode)
    % generate matrix of costs from each point to each other point
    % mode indicates how the cost should be expressed and can be "xyz or xyt", "time or z only", "xy spatial only"
    % visibility_weight is the relative weighting of the visibility term
    % reachability_weight is the relative weighting of the reachability term

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];
    xs = all_pts_plus_start_and_fin(:,1); % vector of all x coords
    ys = all_pts_plus_start_and_fin(:,2); % vector of all y coords

    if mode == "xyz or xyt"
        zs = all_pts_plus_start_and_fin(:,3); % vector of all y coords
        % make cost matrix, g - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
        cgraph = sqrt((xs - xs').^2 + (ys - ys').^2 + (zs - zs').^2)'; % distance of every pt from all other pts
        % make heuristic matrix, h - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
        % xs - finish(:,1)' gives a matrix where each row is a point and each
        % column is a finish point so the element in 3,4 is the difference of
        % point 3 and finish 4
        % then performing min(M,[],2) on this matrix gives a vector with the
        % minimum of each row, i.e. for each point the lowest heuristic cost to
        % a goal.  This is important for the multiple goal case as A* must have
        % a heuristic that underestimtes actual cost
        hvec = min(sqrt((xs - finish(:,1)').^2 + (ys - finish(:,2)').^2 + (zs - finish(:,3)').^2),[],2)';
    elseif mode == "time or z only"
        zs = all_pts_plus_start_and_fin(:,3); % vector of all y coords
        cgraph = sqrt((zs - zs').^2)'; % distance of every pt from all other pts
        hvec = min(sqrt((zs - finish(:,3)').^2),[],2)';
    elseif mode == "xy spatial only"
        cgraph = sqrt((xs - xs').^2 + (ys - ys').^2)'; % distance of every pt from all other pts
        hvec = min(sqrt((xs - finish(:,1)').^2 + (ys - finish(:,2)').^2),[],2)';
    else
        error('The mode argument must be a string with the value "xyz or xyt", or "time or z only", "xy spatial only". You may make a cost matrix manually and circumvent using this function by creating as well by making square N-dimensional matrix where N is the numebr of nodes including starts and goals.  Element i-j has a value corresponding to the cost of going form node i to node j.')
    end
end % end function
