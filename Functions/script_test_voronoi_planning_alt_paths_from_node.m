% script_test_voronoi_planning_alt_poaths_from_node
% test script of planning along voronoi diagram edges and replanning down all edges leaving an arbitrary node
clear; close all; clc

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% mission options
map_idx = 7;
flag_do_plot = 1;
% load map
[shrunk_polytopes, start_init, finish_init, resolution_scale] = fcn_util_load_test_map(map_idx, 1);

%% make medial axis graph
% constrained delaunay triangulation, adjacency_matrix, medial axis graph
[adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);

% prune graph
[adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, flag_do_plot);

% get costs for navigating each triangle chain
[triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, flag_do_plot);

%% planning through triangle graph
% add start and goal to medial axis graph
start_xy = start_init;
finish_xy = finish_init;
[adjacency_matrix, triangle_chains, nodes, start_closest_tri, start_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(start_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
[adjacency_matrix, triangle_chains, nodes, finish_closest_tri, finish_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(finish_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
% make cost graph
% cost is of the form: total cost = w*length + (1-w)*corridor_width
w = 1;
min_corridor_width = 0; % willing to tolerate any corridor with greater than 0 width
denylist_route_chain_ids = []; % no need to manually block edge
[adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);
% adjacency matrix is vgraph
vgraph = adjacency_matrix;
num_nodes = length(nodes);
vgraph(1:num_nodes+1:end) = 1;
% check reachability prior to planning
[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
if ~is_reachable
    % if this iteration is not possible, issue a warning and try again
    my_err = sprintf('initial route planning not possible');
    alternate_routes_nodes{end+1}  = nan;
    alternate_routes_chain_ids{end+1}  = nan;
    alternate_routes{end+1}  = nan;
    smallest_corridors = [smallest_corridors, nan];
    error(my_err)
end
% run Dijkstra's algorithm (no heuristic)
hvec = zeros(1,num_nodes);
% plan a path
[cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
% convert series of nodes to series of spatial points
[route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);

%% generating alterate routes
% pick a node from along the initial route
step = 7;
idx_of_start_node = route(step,3); % third column is node id
% generate routes using every departing edge from the node of interest
[alternate_routes, alternate_routes_nodes, alternate_routes_chain_ids, smallest_corridors, route_lengths] = fcn_MedialAxis_generateAltRoutesFromNode(idx_of_start_node, adjacency_matrix, triangle_chains, nodes, xcc, ycc, finish_xy, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);

%% plot alternate routes
figure; hold on; box on;
leg_str = {};
plot(start_xy(1),start_xy(2),'xg','MarkerSize',10);
plot(finish_xy(1),finish_xy(2),'xr','MarkerSize',10);
plot(start(1),start(2),'.g','MarkerSize',10);
plot(finish(1),finish(2),'.r','MarkerSize',10);
leg_str{end+1} = 'start';
leg_str{end+1} = 'finish';
leg_str{end+1} = 'start node';
leg_str{end+1} = 'finish node';
xlabel('x [km]');
ylabel('y [km]');
route_to_plot = route_full;
plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',length(alternate_routes)+1);
leg_str{end+1} = sprintf('route 1');
for i = 1:length(alternate_routes)
    route_to_plot = alternate_routes{i};
    if isnan(route_to_plot) % if the route wasn't calculated, just remove it
        continue
    end
    plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',length(alternate_routes)+1-i);
    leg_str{end+1} = sprintf('route %i, corridors > %.3f [km]',i,smallest_corridors(i));
end
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
leg_str{end+1} = 'obstacles';
for i = 1:length(shrunk_polytopes)-2
    leg_str{end+1} = '';
end
legend(leg_str,'Location','best');
title(strcat(sprintf('%i paths')));

%% recolor of the above plot
figure; hold on; box on;
leg_str = {};
plot(start_xy(1),start_xy(2),'xg','MarkerSize',10);
plot(finish_xy(1),finish_xy(2),'xr','MarkerSize',10);
plot(start(1),start(2),'.g','MarkerSize',10);
plot(finish(1),finish(2),'.r','MarkerSize',10);
leg_str{end+1} = 'start';
leg_str{end+1} = 'finish';
leg_str{end+1} = 'start node';
leg_str{end+1} = 'finish node';
xlabel('x [km]');
ylabel('y [km]');
route_to_plot = route_full;
plot(route_to_plot(:,1),route_to_plot(:,2),'-b','LineWidth',4);
leg_str{end+1} = sprintf('route 1');
for i = 1:length(alternate_routes)
    route_to_plot = alternate_routes{i};
    if isnan(route_to_plot) % if the route wasn't calculated, just remove it
        continue
    end
    plot(route_to_plot(:,1),route_to_plot(:,2),'k--','LineWidth',2);
    leg_str{end+1} = sprintf('route %i, corridors > %.3f [km]',i,smallest_corridors(i));
end
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
leg_str{end+1} = 'obstacles';
for i = 1:length(shrunk_polytopes)-2
    leg_str{end+1} = '';
end
legend(leg_str,'Location','best');
title(strcat(sprintf('%i paths')));
