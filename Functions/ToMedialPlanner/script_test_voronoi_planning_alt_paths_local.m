% script_test_voronoi_planning_alt_paths_local
% test script of planning along voronoi diagram edges
% this graph is then used to generate alternate paths

% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_util_load_test_map, replaced with fcn_BoundedAStar_loadTestMap
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability
% 2025_11_01 - S. Brennan
% -- removed calls to fcn_BoundedAStar_loadTestMap, replaced with fcn_MapGen_loadTestMap

% clear; close all; clc
% addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% mission options
alt_path_mode = 3; % alt_route_mode can be a 1, 2, or 3:
% 1 blocks only the next segment in the initial path,
% 2 blocks the entire initial path,
% 3 blocks the initial path and all previously calculated alternate paths
map_idx = 7;
flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;
[shrunk_polytopes, start_init, finish_init, resolution_scale] = fcn_MapGen_loadTestMap(map_idx, 1);

%% constrained delaunay triangulation
[adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);

%% prune graph
[adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, flag_do_plot);

%% get costs for navigating each triangle chain
% TODO add zcc as optional input
[triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, flag_do_plot);

%% planning through triangle graph
start_xy = start_init;
finish_xy = finish_init;
% add start and finish to nearest medial axis edge
% TODO add zcc as optional input
[adjacency_matrix, triangle_chains, nodes, start_closest_tri, start_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(start_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
[adjacency_matrix, triangle_chains, nodes, finish_closest_tri, finish_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(finish_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);

%% loop for generating alterate routes
% cost is of the form: total cost = w*length + (1-w)*corridor_width
w = 1;
route_choke = 0;
alternate_routes = {};
alternate_routes_nodes = {};
alternate_routes_chain_ids = {};
smallest_corridors = [];
route_lengths = [];
denylist_route_chain_ids = [];
replanning_times = [];

backstep = 1;
iterations = 1;
init_route_num_nodes = inf; % initialize to infinite until we know init route length
% want to iterate for as many route points as there are
% we walk backwards along the initial route, planning again from each node in the init route
while backstep < init_route_num_nodes - 1
    replanning_time = tic; % time each replanning attempt
    route_choke = 0;
    if iterations ~= 1
        % after the first iteration, we start from a node on the initial path.  Need to get that node's info.
        start_closest_node = my_start(3);
        start_closest_tri = nodes(start_closest_node);
        start_xy = [xcc(start_closest_tri) ycc(start_closest_tri)];
    end

    % make cgraph, this is where denylisted edges are removed
    [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, route_choke, denylist_route_chain_ids);

    % adjacency matrix is vgraph
    vgraph = adjacency_matrix;
    num_nodes = length(nodes);
    vgraph(1:num_nodes+1:end) = 1;
    % check reachability
    [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
    if ~is_reachable
        % if this iteration is not possible, issue a warning and try again
        my_warn = sprintf('alternate route %i planning not possible',iterations);
        warning(my_warn)
        %% update for next iteration of alt route
        alternate_routes_nodes{end+1}  = nan;
        alternate_routes_chain_ids{end+1}  = nan;
        alternate_routes{end+1}  = nan;
        smallest_corridors = [smallest_corridors, nan];
        route_lengths = [route_lengths, nan];
        init_route = alternate_routes_nodes{1};
        init_route_num_nodes = size(init_route,1);
        my_start = init_route(end-backstep,:);
        backstep = backstep + 1;
        iterations = iterations+ 1;
        continue
    end
    % run Dijkstra's algorithm (no heuristic)
    hvec = zeros(1,num_nodes);

    % plan a path
    [cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
    % turn route of nodes into route of x-y positions
    [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);

    %% denylist logic to create alternate routes
    % 1 blocks only the next segment in the initial path,
    % 2 blocks the entire initial path,
    % 3 blocks the initial path and all previously calculated alternate paths
    if alt_path_mode == 1
        if iterations ~= 1
            init_route_chain_ids = alternate_routes_chain_ids{1};
            % if backstep > init_route_num_nodes - 2
            %     continue
            % end
            denylist_route_chain_ids = init_route_chain_ids(end-backstep);
        end
    elseif alt_path_mode == 2
        if iterations == 1
            denylist_route_chain_ids = route_triangle_chain_ids(1:end-2);
        end
    elseif alt_path_mode == 3
        denylist_route_chain_ids = [denylist_route_chain_ids route_triangle_chain_ids(1:end-3)];
    end

    replanning_times = [replanning_times, toc(replanning_time)]

    % plot result
    figure; hold on; box on;
    xlabel('x [km]');
    ylabel('y [km]');
    leg_str = {};
    leg_str{end+1} = 'medial axis graph';
    not_first = 0;
    for i = 1:(size(triangle_chains,1))
        % pop off a triangle chain
        chain_of_note = triangle_chains{i,3};
        if isempty(chain_of_note)
            continue
        end
        % plot big markers for the start and end node
        beg_end = [chain_of_note(1) chain_of_note(end)];
        % plot a straight line between them (this is the adjacency graph connection)
        plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',0.6*ones(1,3));
        if not_first
            leg_str{end+1} = '';
        end
        not_first =1;
        % plot the medial axis path between them (this is the curved path from the triangle chain)
        plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',0.6*ones(1,3));
        leg_str{end+1} = '';
    end
    plot(start_xy(1),start_xy(2),'xg','MarkerSize',10);
    plot(finish_xy(1),finish_xy(2),'xr','MarkerSize',10);
    plot(start(1),start(2),'.g','MarkerSize',10);
    plot(finish(1),finish(2),'.r','MarkerSize',10);
    leg_str{end+1} = 'start';
    leg_str{end+1} = 'finish';
    leg_str{end+1} = 'start node';
    leg_str{end+1} = 'finish node';
    plot(route(:,1),route(:,2),'--r','MarkerSize',20,'LineWidth',1);
    leg_str{end+1} = sprintf('adjacency of route nodes');
    for j = 2:length(shrunk_polytopes)
        fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    end
    leg_str{end+1} = 'obstacles';
    for i = 1:length(shrunk_polytopes)-2
        leg_str{end+1} = '';
    end
    plot(route_full(:,1), route_full(:,2), '-k','LineWidth',2.5) % plot approx. medial axis
    leg_str{end+1} = 'medial axis route';
    legend(leg_str,'Location','best');
    tit_str = sprintf('length cost weight was: %.1f \n total length: %.2f km \n worst corridor: %.2f km',w, route_length, route_choke);
    title(tit_str)
    %% update for next iteration of alt route
    alternate_routes_nodes{end+1}  = route;
    alternate_routes_chain_ids{end+1}  = route_triangle_chain_ids;
    alternate_routes{end+1}  = route_full;
    smallest_corridors = [smallest_corridors, route_choke];
    route_lengths = [route_lengths, route_length];
    init_route = alternate_routes_nodes{1};
    init_route_num_nodes = size(init_route,1);
    my_start = init_route(end-backstep,:);
    backstep = backstep + 1;
    iterations = iterations+ 1;
end % end alternate route iterations loop

% plot alternate routes
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
route_to_plot = alternate_routes{1};
plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',length(alternate_routes)+1);
leg_str{end+1} = sprintf('route 1');
for i = 2:length(alternate_routes)
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
title(sprintf('%i paths, each with wider corridors',length(alternate_routes)));

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
route_to_plot = alternate_routes{1};
plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',length(alternate_routes)+1);
leg_str{end+1} = sprintf('route 1');
for i = 2:length(alternate_routes)
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
% 1 blocks only the next segment in the initial path,
% 2 blocks the entire initial path,
% 3 blocks the initial path and all previously calculated alternate paths
if alt_path_mode == 1
    blocking_mode_str = 'next segment of init. path blocked';
elseif alt_path_mode == 2
    blocking_mode_str = 'entire init. path blocked';
elseif alt_path_mode == 3
    blocking_mode_str = 'entire init. path and all prior alt. paths blocked';
end
title(strcat(sprintf('%i paths, ', length(alternate_routes)), blocking_mode_str));
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
route_to_plot = alternate_routes{1};
plot(route_to_plot(:,1),route_to_plot(:,2),'-b','LineWidth',4);
leg_str{end+1} = sprintf('route 1');
for i = 2:length(alternate_routes)
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
% 1 blocks only the next segment in the initial path,
% 2 blocks the entire initial path,
% 3 blocks the initial path and all previously calculated alternate paths
if alt_path_mode == 1
    blocking_mode_str = 'next segment of init. path blocked';
elseif alt_path_mode == 2
    blocking_mode_str = 'entire init. path blocked';
elseif alt_path_mode == 3
    blocking_mode_str = 'entire init. path and all prior alt. paths blocked';
end
title(strcat(sprintf('%i paths, ', length(alternate_routes)), blocking_mode_str));
