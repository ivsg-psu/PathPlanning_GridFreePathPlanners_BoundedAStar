clear; close all; clc
% script_test_fcn_interpolate_route_spatially
% Tests: fcn_interpolate_route_spatially
%
% REVISION HISTORY:
%
% 2024_01 by S. Harnett
% -- first write of script
%%%%%%%%%%%%%%§

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

% compare the original route and interpolated route
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

% plotting in color order can help check for segments that go "backwards"
% a bug that can occur during interpolation
figure; hold on; box on;
for i = 1:size(route_dense,1)
    plot(route_dense(i,1),route_dense(i,2),'d', 'LineWidth', 2,'MarkerSize',2,'MarkerFaceColor',[1-(i/size(route_dense,1)), 0, (i/size(route_dense,1))],'MarkerEdgeColor',[1-(i/size(route_dense,1)), 0, (i/size(route_dense,1))],'Color',[1-(i/size(route_dense,1)), 0, (i/size(route_dense,1))])
end
