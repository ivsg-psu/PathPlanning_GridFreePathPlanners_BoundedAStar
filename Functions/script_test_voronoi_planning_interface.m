% script_test_voronoi_planning_interface
% tests fcn_MedialAxis_plannerWrapper and fcn_MedialAxis_replanWrapper
% test script of planning along voronoi diagram edges via a wrapper function that encompases the
% medial axis planning stack
clear; close all; clc

%% declair dependencies
addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% polytopes for testing
n_polys = 16; % number of polytopes
polytope_vertices = cell(1,n_polys); % initialize empty cell array
rng(10); % set rng seed for determinism
% random integer for num. verts of each poly, currently set between 3 and 3 to make triangles
num_verts = randi([3 3],n_polys);
poly_scale = ones(1,n_polys); % vector to scale size of polytopes if desired, currently set to 1
figure; hold on; box on;
for poly = 1:n_polys
    verts = rand(num_verts(poly),2); % create random nx2 matrix of vertices
    verts = verts - 0.5; % center random verts at 0
    verts = verts*poly_scale(poly); % scale size of polytopes
    [x_shift, y_shift] = ind2sub([4, 4], poly); % shift polytope to position in 4x4 obs. field
    verts = verts + [x_shift*ones(num_verts(poly),1) y_shift*ones(num_verts(poly),1)]; % shift vertices based on position in matrix
    fill(verts(:,1),verts(:,2),poly_scale(poly));
    polytope_vertices{poly} = verts;
end

%% other inputs for testing
start = [0.5 0.5];
finish = [4.5 4.5];
boundary_verts = [0.25 0.25;...
                  0.25 4.75;...
                  4.75 4.75;...
                  4.75 0.25;...
                  ];
min_corridor_width = 0;

length_cost_weight = 1;

%% wrapper function called here
[route, route_length, route_choke] = fcn_MedialAxis_plannerWrapper(polytope_vertices, start, finish, boundary_verts, min_corridor_width, length_cost_weight);
%% plot result
figure(1);
plot(route(:,1), route(:,2), '-k','LineWidth',2.5)
xlabel('x [m]')
ylabel('y [m]')
title(sprintf('path length: %.2f [m]\n narrowest corridor: %.2f [m]',route_length, route_choke));

%% replan wrapper called here
replan_point = [2, 2.5];
[new_route,~ ,~] = fcn_MedialAxis_replanWrapper(replan_point, finish, min_corridor_width, length_cost_weight);
plot(replan_point(1), replan_point(2), 'dm','MarkerFaceColor','m','MarkerSize',6)
plot(new_route(:,1), new_route(:,2), '--g','LineWidth',2.5)

replan_point = [1.5, 3];
[new_route,~ ,~] = fcn_MedialAxis_replanWrapper(replan_point, finish, min_corridor_width, length_cost_weight);
plot(replan_point(1), replan_point(2), 'dr','MarkerFaceColor','r','MarkerSize',6)
plot(new_route(:,1), new_route(:,2), '--r','LineWidth',2.5)
