clear; close all; clc
% script_test_voronoi_planning
% test script of planning along voronoi diagram edges

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));


map_idx = 8
flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;
[shrunk_polytopes, start_init, finish_init, resolution_scale] = fcn_util_load_test_map(map_idx, 1)

%% constrained delaunay triangulation
[adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);

%% prune graph
[adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, flag_do_plot);

%% get costs for navigating each triangle chain
% TODO add zcc as optional input
[triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, flag_do_plot);

%% planning through triangle graph
start_xy = start_init; %[1031 -4717];
finish_xy = finish_init; %[1050 -4722];
% TODO add zcc as optional input
[adjacency_matrix, triangle_chains, nodes, start_closest_tri, start_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(start_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
[adjacency_matrix, triangle_chains, nodes, finish_closest_tri, finish_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(finish_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);

for w = 0.1:0.1:1
    % TODO for alt paths, instead of looping on weights you loop on route chokes:
    % w = 1;
    % route_choke = 0;
    % alternate_routes = {};
    % smallest_corridors = [];
    % route_lengths = [];
    % for iterations = 1:5
    % TODO for alt paths branching from primary path, need to take route_triangle_chains out of route unpacking function
    % and pass this into cgraph creation function as an argument
    % and need to start from midpoint on init route:
    % backstep = 1;
    % iterations = 1;
    % init_route_num_nodes = inf; % initialize to infinite until we know init route length
    % while backstep < init_route_num_nodes
    min_corridor_width = 0; % do not restrict corridor width
    denylist_route_chain_ids = []; % no need to denylist any triangle chains
    [adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);
    % adjacency matrix is vgraph
    vgraph = adjacency_matrix;
    num_nodes = length(nodes);
    vgraph(1:num_nodes+1:end) = 1;
    % check reachability
    [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
    if ~is_reachable
        error('initial mission, prior to edge deletion, is not possible')
    end
    % run Dijkstra's algorithm (no heuristic)
    hvec = zeros(1,num_nodes);

    % plan a path
    [cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
    % TODO for replanning from route need something like this here and after ~is_reachable
    % init_route = alternate_routes_nodes{1};
    % init_route_num_nodes = size(init_route,1);
    % start = init_route(end-backstep,:);
    % backstep = backstep + 1;
    % iterations = iterations+ 1;
    % continue

    % take route and tri chains data structure
    % also take best path structure
    [route_full, route_length, route_choke] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);
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
        % pot big markers for the start and end node
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
    % TODO add alt route plotting code
end % end weight loop
