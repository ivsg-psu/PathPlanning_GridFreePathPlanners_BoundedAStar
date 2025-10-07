% script_test_alternate_path_generation
% example of using a cost matrix (such as corridor width) to create alternate paths
% with increasingly tight hard constraints rather than only as an incentive/cost or soft constriant

% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_util_load_test_map, replaced with fcn_BoundedAStar_loadTestMap
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal
% -- removed calls to fcn_polytopes_generate_all_pts_table,
%    % replaced with fcn_BoundedAStar_polytopesGenerateAllPtsTable
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability
% -- removed calls to fcn_algorithm_generate_cost_graph,
%    % replaced with fcn_BoundedAStar_generateCostGraph
% -- removed calls to fcn_algorithm_generate_dilation_robustness_matrix,
%    % replaced with fcn_BoundedAStar_generateDilationRobustnessMatrix


% clear; close all; clc
% addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% plotting flags
flag_do_plot = 1;
flag_do_plot_slow = 0;

%% mission options
map_idx = 7;
num_paths = 10; % number of alternate paths to generate
corridor_width_buffer = 1.1; % how much larger should the smallest corridor in route n+1 be relative to the smallest corridor in route n?
[shrunk_polytopes, start_inits, finish_inits] = fcn_BoundedAStar_loadTestMap(map_idx);

for mission_idx = 1:size(start_inits,1)
    start_init = start_inits(mission_idx,:);
    finish_init = finish_inits(mission_idx,:);

    % all_pts array creation
    [all_pts, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes, start_init, finish_init);

    %% plan the initial path
    % make vgraph
    finishes = [all_pts; start; finish];
    starts = [all_pts; start; finish];
    [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes,1);
    orig_vgraph = vgraph;
    % make rgraph
    [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
    if ~is_reachable
        error('initial mission, prior to edge deletion, is not possible')
    end
    % make cgraph
    mode = "xy spatial only";
    [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode);

    % make dilation robustness matrix
    mode = '2d';
    dilation_robustness_tensor = fcn_BoundedAStar_generateDilationRobustnessMatrix(all_pts, start, finish, vgraph, mode, shrunk_polytopes);
    dilation_robustness_matrix = max(dilation_robustness_tensor(:,:,1) , dilation_robustness_tensor(:,:,2)); % combine the left and right sides as a max
    dilation_robustness_matrix_for_variance = dilation_robustness_matrix(:)'; % extract vector of all values
    dilation_robustness_matrix_for_variance(dilation_robustness_matrix_for_variance == 0) = []; % remove 0s
    dilation_robustness_matrix_for_variance(isinf(dilation_robustness_matrix_for_variance)) = []; % remove infs
    variance_of_corridor_widths = var(dilation_robustness_matrix_for_variance); % find variance of corridor width/dilation robustness
    % plan init route
    [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
    routes{1} = init_route; % store in cell array of all alt routes
    % find init route length
    route_x = init_route(:,1);
    route_y = init_route(:,2);
    lengths = diff([route_x(:) route_y(:)]);
    init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
    route_lengths = [init_route_length]; % note the route lengths
    smallest_corridors = nan(1,num_paths); % note the smallest corridor of each route
    replanning_times = zeros(1,num_paths);
    for path_idx = 1:num_paths
        replanning_time = tic;
        %% find smallest corridor in last route and remove all corridors that small or smaller
        % plan next best path given tighter constraints
        route_segment_costs = nan(1,size(init_route,1)-1);
        route_segment_vis  = nan(1,size(init_route,1)-1);
        route_segment_corridor_widths = nan(1,size(init_route,1)-1);
        for waypoint_id = 1:(size(init_route,1)-1)
            % loop through each route segment...
            route_segment_start = init_route(waypoint_id,:);
            route_segment_end = init_route(waypoint_id+1,:);
            % store the cost, visibility, and corridor width of each route segment
            route_segment_costs(waypoint_id) = cgraph(route_segment_start(3), route_segment_end(3));
            route_segment_vis(waypoint_id) = vgraph(route_segment_start(3), route_segment_end(3));
            route_segment_corridor_widths(waypoint_id) = dilation_robustness_matrix(route_segment_start(3), route_segment_end(3));
        end
        smallest_corridor_in_init_path = corridor_width_buffer*min(route_segment_corridor_widths); % find smallest corridor in route, scaled by our buffer value
        smallest_corridors(path_idx) = smallest_corridor_in_init_path; % note this in our list of smallest corridors
        new_vgraph = vgraph; % copy vgraph
        % set all vgraph edges where corridor width is smaller or equal to 0
        new_vgraph(dilation_robustness_matrix <= smallest_corridor_in_init_path) = 0;
        % get vgraph stats on number of removed edges
        num_edges_initially = sum(sum(vgraph));
        num_edges_finally = sum(sum(new_vgraph));
        num_edges_removed = num_edges_initially - num_edges_finally;
        pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
        %% plan alternate route
        % find rgraph again
        [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(new_vgraph,start(3),finish(3));
        if ~is_reachable
            warning(sprintf('mission planning is impossible with > %3f size corridors',smallest_corridor_in_init_path));
            routes{end+1} = nan;
            continue
        end % end is_reachable condition for replanning
        % make cgraph again
        % plan alternate route
        [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts, start, finish);
        % store this in list of all routes
        routes{end+1} = replan_route;
        init_route = replan_route; % make current alt route new baseline route for next iteration of the loop
        % find alternate route length
        route_x = replan_route(:,1);
        route_y = replan_route(:,2);
        lengths = diff([route_x(:) route_y(:)]);
        replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));
        route_lengths = [route_lengths, replan_route_length]; % store this route length in list of all route lengths
        replanning_times(path_idx) = toc(replanning_time)
    end % end number of paths loop
    %% plot all alternate routes on one plot
    if flag_do_plot
        figure; hold on; box on;
        xlabel('x [km]');
        ylabel('y [km]');
        plot(start_init(1),start_init(2),'xg','MarkerSize',6);
        plot(finish_init(1),finish_init(2),'xr','MarkerSize',6);
        leg_str = {'start','finish'};
        route_to_plot = routes{1};
        plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',2);
        leg_str{end+1} = sprintf('route 1');
        for i = 1:num_paths
            route_to_plot = routes{i+1};
            if isnan(route_to_plot) % if the route wasn't calculated, just remove it
                continue
            end
            plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',2);
            leg_str{end+1} = sprintf('route %i, corridors > %.3f [km]',i+1,smallest_corridors(i));
        end
        for j = 1:length(shrunk_polytopes)
            fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
        end
        leg_str{end+1} = 'obstacles';
        for i = 1:length(shrunk_polytopes)-1
            leg_str{end+1} = '';
        end
        legend(leg_str,'Location','best');
        title(sprintf('%i paths, each with corridors %2.0f%% larger',num_paths,(corridor_width_buffer-1)*100));
    end % end flag_do_plot condition
end % end mission (i.e., start goal pair) loop

%% enlarge polytopes and see if the same path results
for enlarge_idx = 1:(num_paths)
    % enlarge polytopes by the distance, below which corridors were filtered out, halved (because you dilate polytopes on both side of the corridor)
    enlarged_polytopes = fcn_MapGen_polytopesExpandEvenlyForConcave(shrunk_polytopes,(smallest_corridors(enlarge_idx))/2);
    % generate all_pts array for enlarged polytopes
    [all_pts_new, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(enlarged_polytopes, start_init, finish_init);
    % make vgraph for enlarged map
    finishes = [all_pts_new; start; finish];
    starts = [all_pts_new; start; finish];
    [new_vgraph, visibility_results_all_pts_new] = fcn_Visibility_clearAndBlockedPointsGlobal(enlarged_polytopes, starts, finishes,1);
    reduced_vgraph = new_vgraph;
    % make rgraph for enlarged map
    [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(new_vgraph,start(3),finish(3));
    if ~is_reachable
        warning(sprintf('mission planning impossible at enlargement %.3f',smallest_corridors(enlarge_idx)))
        continue
    end % end is_reachable condition for replanning
    % make cgraph for enlarged map
    mode = "xy spatial only";
    [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts_new, start, finish, mode);
    % plan route for enlarged map
    [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts_new, start, finish);

    %% plot path for dilated obstacle field
    if flag_do_plot
        figure; hold on; box on;
        xlabel('x [km]');
        ylabel('y [km]');
        plot(start_init(1),start_init(2),'xg','MarkerSize',6);
        plot(finish(1),finish(2),'xr','MarkerSize',6);
        plot(replan_route(:,1),replan_route(:,2),'--g','LineWidth',2);
        for j = 1:length(enlarged_polytopes)
             fill(enlarged_polytopes(j).vertices(:,1)',enlarged_polytopes(j).vertices(:,2),[0 0 1],'FaceColor','r','FaceAlpha',0.3)
        end
        for j = 1:length(shrunk_polytopes)
             fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
        end
        leg_str = {'start','finish','route','enlarged obstacles'};
        for i = 1:length(shrunk_polytopes)-1
            leg_str{end+1} = '';
        end
        leg_str{end+1} = 'obstacles';
        for i = 1:length(shrunk_polytopes)-1
            leg_str{end+1} = '';
        end
        title_string = sprintf('enlarged polytopes by %.3f',smallest_corridors(enlarge_idx));
        title(title_string);
    end % end plot flag
end % end polytope enlarge loop

function INTERNAL_fcn_format_timespace_plot()
    box on
    % define figure properties
    opts.width      = 8;
    opts.height     = 6;
    opts.fontType   = 'Times';
    opts.fontSize   = 9;
    fig = gcf;
    % scaling
    fig.Units               = 'centimeters';
    fig.Position(3)         = opts.width;
    fig.Position(4)         = opts.height;

    % set text properties
    set(fig.Children, ...
        'FontName',     'Times', ...
        'FontSize',     9);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('t [s]')
    view([36 30])
end
