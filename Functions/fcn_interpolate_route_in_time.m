function route_dense = fcn_interpolate_route_in_time(route,dt)
    %% interpolation code for a route
    % columns of verts are x,y,t,id
    % for number of unique time values in verts...
    unique_times = unique(route(:,3));
    num_unique_times = length(unique(route(:,3)));

    num_route_verts = size(route,1);

    dense_times = [];
    for i = 2:1:num_unique_times
        new_times = unique_times(i-1):dt:unique_times(i);
        dense_times = [dense_times; new_times'];
    end
    dense_times = unique(dense_times);
    num_dense_times = length(dense_times);

    try
        route_dense_x = interp1(route(:,3),route(:,1),dense_times);
    catch
        route_dense_x = route(1,1)*ones(size(dense_times,1),size(dense_times,2));
    end
    try
        route_dense_y = interp1(route(:,3),route(:,2),dense_times);
    catch
        route_dense_y = route(1,2)*ones(size(dense_times,1),size(dense_times,2));
    end

    route_dense = [route_dense_x route_dense_y dense_times];
end
