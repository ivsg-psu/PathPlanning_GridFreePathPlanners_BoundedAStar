function [cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = fcn_algorithm_straight_planner(start,finish,all_pts,polytopes)
    cost = 0;
    distance_in_polys = 0;
    distance_outside_polys = 0;
    num_polys_traversed = 0;
    a_b = fcn_general_calculation_euclidean_point_to_point_distance(start(1:2),finish(1:2));
    % determine which polys are near the straight path
    close_polytopes = fcn_polytope_calculation_polytopes_near_the_line(start,finish,polytopes)
    figure(5);
    fig = 5;
    line_spec = "b-";
    line_width = 3; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square';% plot axes style
    hold on;
    box on;
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    line_spec = "r-";
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square';% plot axes style
    hold on;
    box on;
    fcn_plot_polytopes(close_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    plot(linspace(0,1,2),0.5*ones(2))

    % obtain equation for line from a (start) to b (finish)
    % for each poly:
    %     if poly has at least one vertex above and at least one below the equation for a-b:
    %         note poly as straddling the line
    % for each of those polytopes...
    for i = 1:length(close_polytopes)
        % determine if the straight path crosses the side
        wall_starts_xy = close_polytopes(i).vertices(1:end-1,1:2);
        wall_ends_xy = close_polytopes(i).vertices(2:end,1:2);
        % search using option 2 of findIntersectionOfSegments to find all intersections
        [dists_to_crossings,locations_of_crossings,~] = ...
            fcn_geometry_findIntersectionOfSegments(wall_starts_xy,wall_ends_xy,...
            start(1:2),finish(1:2),2);
        % there should always be 0 or 2 crossings
        assert(size(locations_of_crossings,1) == 2 || size(locations_of_crossings,1) == 0);
        % note the distance between intersection points
        if size(locations_of_crossings,1) == 2
            figure(fig); hold on;
            plot(locations_of_crossings(:,1),locations_of_crossings(:,2),"cx");
            distance_through_poly = fcn_general_calculation_euclidean_point_to_point_distance(locations_of_crossings(1,:),locations_of_crossings(2,:));
            % increment traversed polytope counter
            num_polys_traversed = num_polys_traversed + 1;
        elseif size(locations_of_crossings,1) == 0
            distance_through_poly = 0;
        end
        % add to total distance_in_polys
        distance_in_polys = distance_in_polys + distance_through_poly;
        % scale by poly.cost, add to total_cost
        cost_through_poly = (1+close_polytopes(i).cost)*distance_through_poly;
        cost = cost + cost_through_poly;
    end
    % subtract distance_in_polys from dist between a-b % this is outside of polytope distance
    distance_outside_polys = a_b - distance_in_polys;
    % add unscaled outside distance to total_cost
    cost = cost + distance_outside_polys;
end
