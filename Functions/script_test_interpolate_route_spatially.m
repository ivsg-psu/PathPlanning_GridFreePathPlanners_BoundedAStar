clear; close all; clc
% script_test_interpolate_route_spatially
% a basic test of interpolating a route in XY space

%% initialize test fixtures
start = [0,0];
finish = start+10;
mid_route = [1, 0; 1, 1; 5, 4; 10, 4; 4, 6; 2,4];
init_route = [start; mid_route; finish];

%% test function
desired_spacing = 0.25;
route_dense = fcn_interpolate_route_spatially(init_route, desired_spacing);

% assert that the number of points in the dense route is greater than the requested number of points to add
assert(gt(size(route_dense,1),size(init_route,1)));
% assert that the points in the sparse route are all still in the dense route
assert(isequal(sum(ismember(init_route, route_dense, 'rows')),size(init_route,1)));

%% plot results
figure; hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
plot(start(1),start(2),'xg','MarkerSize',3);
plot(finish(1),finish(2),'xr','MarkerSize',3);
plot(init_route(:,1),init_route(:,2),'-ok', 'LineWidth', 3, 'MarkerSize',3);
plot(route_dense(:,1),route_dense(:,2),'--dm', 'LineWidth', 2,'MarkerSize',2)
title_string = sprintf('interpolating a %i point route \n to form a %i point route', size(init_route,1), size(route_dense,1));
title(title_string);
legend('start','finish','initial route','interpolated route','Location','northwest');
