function [cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = ...
    fcn_BoundedAStar_straightPlanner(start,finish,all_pts,polytopes)
    % fcn_BoundedAStar_straightPlanner
    % plans a path straight through the obstacle field, traversing all encountered obstacles
    %
    % FORMAT:
    %
    % [cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = ...
    % fcn_BoundedAStar_straightPlanner(start,finish,all_pts,polytopes)
    %
    % INPUTS:
    %
    % START: 1-by-5 vector with the same info as route for the starting point
    % FINISH: same as start for the finish point
    % ALL_PTS: p-by-5 matrix of all the points except start and finish
    % POLYTOPES: a 1-by-n seven field structure of shrunken polytopes,
    % where n <= number of polytopes with fields:
    %   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
    %     the number of the individual polytope vertices
    %   xv: a 1-by-m vector of vertice x-coordinates
    %   yv: a 1-by-m vector of vertice y-coordinates
    %   distances: a 1-by-m vector of perimeter distances from one point to the
    %     next point, distances(i) = distance from vertices(i) to vertices(i+1)
    %   mean: centroid xy coordinate of the polytope
    %   area: area of the polytope
    %
    % OUTPUTS:
    %
    % cost - the cost to execute this path, scaled up by polytope traversal costs for distance
    %   spent in polytopes
    % distance_in_polys - the distance of the straight path inside polytope boundaries
    % distance_outside_polys - the distance of the straight path outside polytope boundaries, in free space
    % num_polys_traversed - the integer number of polytopes encountered and traversed
    %
    % DEPENDENCIES:
    %   fcn_BoundedAStar_polytopesNearLine
    %   fcn_BoundedAStar_plotPolytopes
    %   fcn_geometry_findIntersectionOfSegments
    %
    % EXAMPLES:
    %
    % For additional examples, see: script_planning_performed_at_multiple_costs.m
    %
    % This function was written in 2022_05 by Steve Harentt
    % Questions or comments? sjh6473@psu.edu
    %

    % Revision History:
    % 2025_07_08 - K. Hayes, kxh1031@psu.edu
    % -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
    %    with vector sum method 
    % 2025_07_17 - K. Hayes
    % -- copied to new function from fcn_algorithm_straight_planner to
    %    follow library conventions
    
    % TO DO

    %% Debugging and Input checks
    flag_check_inputs = 1; % Set equal to 1 to check the input arguments
    flag_do_plot = 0;      % Set equal to 1 for plotting
    flag_do_debug = 0;     % Set equal to 1 for debugging

    % initialize variables
    cost = 0;
    distance_in_polys = 0;
    distance_outside_polys = 0;
    num_polys_traversed = 0;
    a_b = sum((start(1:2) - finish(1:2)).^2,2).^0.5;
    % determine which polys are near the straight path
    close_polytopes = fcn_BoundedAStar_polytopesNearLine(start,finish,polytopes)
    if flag_do_plot
        figure(5);
        fig = 5;
        line_spec = "b-";
        line_width = 3; % linewidth of the edge
        axes_limits = [0 1 0 1]; % x and y axes limits
        axis_style = 'square';% plot axes style
        hold on;
        box on;
        fcn_BoundedAStar_plotPolytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
        line_spec = "r-";
        line_width = 2; % linewidth of the edge
        axes_limits = [0 1 0 1]; % x and y axes limits
        axis_style = 'square';% plot axes style
        hold on;
        box on;
        fcn_BoundedAStar_plotPolytopes(close_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
        plot(linspace(0,1,2),0.5*ones(2))
    end

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
            figure(111111); hold on;
            plot(locations_of_crossings(:,1),locations_of_crossings(:,2),"cx");
            distance_through_poly = sum((locations_of_crossings(1,:) - locations_of_crossings(2,:)).^2,2).^0.5;
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
