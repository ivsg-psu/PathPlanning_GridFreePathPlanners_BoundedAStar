function route_dense = fcn_BoundedAStar_interpolateRouteSpatially(route, spacing)
% fcn_BoundedAStar_interpolateRouteSpatially
%
% Adds waypoints to the input route to increase the density of points according to the number of
% desired points. This is useful for threadpulling (replanning from waypoints along an initial route).
%
%
% FORMAT:
%
% route_dense = fcn_BoundedAStar_interpolateRouteSpatially(route,num_pts)
%
% INPUTS:
%
%     route: the matrix as produced by fcn_BoundedAStar_AStar consisting of waypoints.  Each row is a
%     waypoint, and each column is x, y, and point ID
%
%     spacing: the desired spacing between points to achieve by adding waypoints
%       when creating the dense route by interpolating the route waypoints
%
% OUTPUTS:
%     route_dense: the matrix representing the interpolated route consisting of waypoints.  Each row is a
%     waypoint, and each column is x, y, and point ID
%
% DEPENDENCIES:
% generally, the input for this function can be generated by
% fcn_BoundedAStar_AStar or fcn_BoundedAStar_AStarBounded
% but this is not a strictly necessary dependency
%
% EXAMPLES:
%
% See the script: script_test_fcn_interpolate_route_spatially
% for a full test suite.
%
% This function was written in Janyuary 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024, January by Steve Harnett
% -- first write of function
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_interpolate_route_spatially to follow
%    library convention
%
% TO DO:



    %% interpolation code for a route
    num_route_verts = size(route,1);
    route_dense = [];

    % loop through each pair of waypoints
    for i = 1:(num_route_verts-1)
        % get pair of adjascent waypoints and distance between them
        p1 = route(i,:);
        p2 = route(i+1,:);
        dist21 = sqrt((p2(2)-p1(2))^2+(p2(1)-p1(1))^2);
        needed_points = dist21/spacing;

        % if the x's aren't the same, interpolate normally BASED ON DISTANCE
        if p1(1) ~= p2(1)
            route_dense_x = linspace(p1(1),p2(1),needed_points);
            route_dense_y = interp1(route(i:(i+1),1),route(i:(i+1),2),route_dense_x);

        % if the x's are the same, interpolate in y instead
        elseif p1(1) == p2(1)
            route_dense_y = linspace(p1(2),p2(2),needed_points);
            route_dense_x = interp1(route(i:(i+1),2),route(i:(i+1),1),route_dense_y);

        % this condition should not be hit
        else
            error('Waypoints may be malformed')
        end
        % For a similar conditional block, see fcn_MapGen_increasePolytopeVertexCount

        % assemble new waypoints into dense route
        route_dense = [route_dense; route_dense_x', route_dense_y'];
    end
    % remove any duplicated points whilst preserving route order
    route_dense = unique(route_dense,'stable','rows');
end
