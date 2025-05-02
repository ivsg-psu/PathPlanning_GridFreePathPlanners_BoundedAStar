% script_test_voronoi_planning
% test script of planning along voronoi diagram edges
clear; close all; clc

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));


map_idx = 7;
flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;
[shrunk_polytopes, start_init, finish_init, resolution_scale] = fcn_util_load_test_map(map_idx, 1);

%% for sake of comparisons, do this with a visibility graph as well
polytopes = shrunk_polytopes;
if flag_do_plot
    fig = 99; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [1025 1055 -4726 -4710]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes(2:end),fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
end
point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes
beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
curpt = 0;
for poly = 1:size(polytopes,2) % check each polytope
    verts = unique(polytopes(poly).vertices,'stable','rows');
    num_verts = size(verts,1);
    polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
    polytopes(poly).xv = verts(:,1)';
    polytopes(poly).yv = verts(:,2)';
    polytopes(poly).vertices = [verts; verts(1,:)];
    polytopes(poly).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly).vertices(1:end-1,:),polytopes(poly).vertices(2:end,:));
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points
all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
vgraph_times = [];
for i = 1:10 % get timing data for repeat iterations
    create_vgraph_timer = tic;
    [vgraph, visibility_results] = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts,all_pts);
    vgraph_time = toc(create_vgraph_timer)
    vgraph_times = [vgraph_times vgraph_time]
end
% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-r')
            end
        end
    end
end
med_ax_times = [];
for i = 1:10 % get timing data for repeat iterations
    t_medial_axis = tic;
    %% constrained delaunay triangulation
    [adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);
    medial_axis_time = toc(t_medial_axis)
    med_ax_times = [med_ax_times medial_axis_time]
end

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

% loop over cost function weights
for w = 0.1:0.1:1
    min_corridor_width = 0; % do not restrict corridor width
    denylist_route_chain_ids = []; % no need to denylist any triangle chains
    % make cost matrix
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

    % expand route nodes into actual path
    [route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);
    % plot result
    figure; hold on; box on;
    xlabel('x [km]');
    ylabel('y [km]');
    leg_str = {};
    leg_str{end+1} = 'medial axis graph';
    not_first = 0;
    % plot all triangle chains
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
end % end weight loop
