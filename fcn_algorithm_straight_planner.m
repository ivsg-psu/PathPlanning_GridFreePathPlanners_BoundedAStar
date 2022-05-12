function cost = fcn_algorithm_straight_planner(start,finish,all_pts,polytopes)
    cost = 0;
    distance_in_polys = 0;
    a_b = fcn_general_calculation_euclidean_point_to_point_distance(start,finish);
    % determine which polys are near the straight path
    close_polytopes = fcn_polytope_calculation_polytopes_near_the_line(start,finish,polytopes)
    % obtain equation for line from a (start) to b (finish)
    % for each poly:
    %     if poly has at least one vertex above and at least one below the equation for a-b:
    %         note poly as straddling the line
    % for each of those polytopes...
    for i = 1:length(close_polytopes)
        % look at each side on the polytope...
        for j = 1:length(close_polytopes(i).vertices)
            poly_crossings = [];
            % determine if the straight path crosses the side
            side_crossing = fcn_visibility_line_polytope_intersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes)
            % there should only be one or 0 crossings for a given side
            assert(length(poly_crossings) <=1);
            % if there was one crossing, store it
            if length(side_crossing) == 1
                poly_crossings = [poly_crossings; side_crossing(1).points];
                % there should never be more than 2 crossings for the poly
                assert(size(side_crossings,1) < 3);
            end
            % if this is the second crossing for this poly, break
            if size(side_crossings,1) == 2
                break;
            end
        end
        % note the distance between intersection points
        distance_through_poly = fcn_general_calculation_euclidean_point_to_point_distance(poly_crossings(1,:),poly_crossings(2,:));
        % add to total distance_in_polys
        distance_in_polys = distance_in_polys + distance_through_poly;
        % scale by poly.cost, add to total_cost
        cost_through_poly = (1+polytope(i).cost)*distance_through_poly;
        cost = cost + cost_through_poly;
    end
    % subtract distance_in_polys from dist between a-b % this is outside of polytope distance
    distance_outside_polys = a_b - distance_in_polys;
    % add unscaled outside distance to total_cost
    cost = cost + distance_outside_polys;
end
