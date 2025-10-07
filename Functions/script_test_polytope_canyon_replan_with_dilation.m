% script_test__polytope_canyon_replan_with_dilation
% This script is very similar to the scrip_test_polytope_canyon_replan but it evaluates a corridor width cost function instead of
% a connectivity cost function. This script plans two paths: (1) a distance reducing path and (2) a
% path that routes down edges with more free space around them. Then obstacles are dilated to close
% off narrow corridors and replanning is triggered from midway down the initial path. The final paths
% are then compared to show that replanning from the path that uses wider corridors is less costly and
% more likely to be successful.

% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_util_load_test_map, replaced with fcn_BoundedAStar_loadTestMap
% -- fixed calls to fcn_MapGen_polytopesStatistics, replaced with fcn_MapGen_statsPolytopes
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
flag_do_plot_slow= 0;
flag_do_threadpulling = 1;
flag_save_plots = 1;

% map_idx nominal_or_width_based polytope_size_increases polytope_size_increases init_route_length navigated_distance replan_route_length
data = []; % initialize array for storing results
%% mission options
for map_idx = [7, 8, 9] % Halton maps
% for map_idx = [3, 5, 6] % flood plain maps
    [shrunk_polytopes, start_inits, finish_inits,~, length_cost_weights, navigated_portions] = fcn_BoundedAStar_loadTestMap(map_idx); % relative weighting of cost function, cost = w*length_cost + (1-w)*dilation_robustness_cost

    %% get stats for this map to find gap size
    poly_map_stats = fcn_MapGen_statsPolytopes(shrunk_polytopes);
    N_int = poly_map_stats.linear_density_mean;
    figure; hold on; box on;
    xlabel('x [km]')
    ylabel('y [km]')
    for j = 1:length(shrunk_polytopes)
        fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
    end
    all_xv = extractfield(shrunk_polytopes,'xv');
    L = max(all_xv) - min(all_xv);
    A_occ = poly_map_stats.occupied_area;
    A = poly_map_stats.total_area;
    A_unocc = A - A_occ;
    r_A_unocc = A_unocc/A;
    r_L_unocc_old = sqrt(r_A_unocc);
    G_bar_old = r_L_unocc_old*L/N_int;

    for mission_idx = 1:size(start_inits,1)
        w = length_cost_weights(mission_idx);
        start_init = start_inits(mission_idx,:);
        finish_init = finish_inits(mission_idx,:);
        navigated_portion = navigated_portions(mission_idx);

        % loop over dilation sizes
        for polytope_size_increases = [0.01 0.02 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9  0.95 1]
            % loop over the nominal cost function and feature cost function
            for nominal_or_width_based = [1,2]
                trial_identifier = sprintf('map idx: %i, nominal or corridor-width-based: %i,\npolytope size increase [km]: %.2f',str2num(strcat(num2str(map_idx),num2str(mission_idx))), nominal_or_width_based,polytope_size_increases)
                %% plan the initial path
                % all_pts array creation
                [all_pts, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes, start_init, finish_init);
                % find vgraph
                finishes = [all_pts; start; finish];
                starts = [all_pts; start; finish];
                [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes,1);
                orig_vgraph = vgraph; % note the original to compare it to the reduced vgraph

                % find rgraph
                [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
                if ~is_reachable
                    warning('initial mission, prior to edge deletion, is not possible')
                    continue
                end

                % make cgraph
                mode = "xy spatial only";
                [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode);

                % make dilation robustness matrix
                mode = '2d';
                dilation_robustness_tensor = fcn_BoundedAStar_generateDilationRobustnessMatrix(all_pts, start, finish, vgraph, mode, shrunk_polytopes);
                dilation_robustness_matrix = min(dilation_robustness_tensor(:,:,1), dilation_robustness_tensor(:,:,2)); % combine the left and right sides as a max
                dilation_robustness_matrix_for_variance = dilation_robustness_matrix(:)'; % extract vector of all values
                dilation_robustness_matrix_for_variance(dilation_robustness_matrix_for_variance == 0) = []; % remove 0s
                dilation_robustness_matrix_for_variance(isinf(dilation_robustness_matrix_for_variance)) = []; % remove infs
                variance_of_corridor_widths = var(dilation_robustness_matrix_for_variance); % find variance of corridor width/dilation robustness
                if nominal_or_width_based == 2
                    % make cost function
                    inv_corridor_width = 1./dilation_robustness_matrix; % invert such that large corridors cost less
                    infinite_idx = find(inv_corridor_width==inf); % find inf
                    inv_corridor_width(infinite_idx) = 10000; % set "infinity" to a large value so cost is finite
                    cgraph = w*cgraph + (1-w)*inv_corridor_width;
                end
                % plan initial route
                [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

                if flag_do_threadpulling && nominal_or_width_based==2
                    % backup initial route for comparison
                    init_route_original = init_route;
                    % find initial route length
                    route_x = init_route(:,1);
                    route_y = init_route(:,2);
                    lengths = diff([route_x(:) route_y(:)]);
                    init_route_length_original= sum(sqrt(sum(lengths.*lengths,2)));
                    % create all points and start/finish for threadpulling from initial route
                    all_pts_tp = init_route(2:(end-1),:);
                    start_tp = init_route(1,:);
                    finish_tp = init_route(end,:);
                    all_pts_tp(:,3) = [1:size(all_pts_tp,1)];
                    start_tp(3) = size(all_pts_tp,1) + 1;
                    finish_tp(3) = size(all_pts_tp,1) + 2;
                    % make vgraph again
                    finishes_tp = [all_pts_tp; start_tp; finish_tp];
                    starts_tp = [all_pts_tp; start_tp; finish_tp];
                    [vgraph_tp, visibility_results_tp] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts_tp, finishes_tp,1);
                    % make rgraph again
                    [is_reachable_tp, num_steps_tp, rgraph_tp] = fcn_BoundedAStar_checkReachability(vgraph_tp,start_tp(3),finish_tp(3));
                    if ~is_reachable_tp
                        % we don't want to break if replanning is impossible, we want to save the data for what caused this
                        warning('threadpulling is impossible')
                        continue
                    end % end is_reachable condition for replanning
                    % make cgraph again
                    mode = "xy spatial only";
                    [cgraph_tp, hvec_tp] = fcn_BoundedAStar_generateCostGraph(all_pts_tp, start_tp, finish_tp, mode);
                    % replan path
                    [cost_tp, route_tp] = fcn_algorithm_Astar(vgraph_tp, cgraph_tp, hvec_tp, all_pts_tp, start_tp, finish_tp);
                    % overwrite route and length with threadpulled versions of these
                    init_route = route_tp;
                    init_cost = cost_tp;
                    % find initial route length
                    route_x = init_route(:,1);
                    route_y = init_route(:,2);
                    lengths = diff([route_x(:) route_y(:)]);
                    init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
                end % end flag_do_threadpulling

                % find initial route length
                route_x = init_route(:,1);
                route_y = init_route(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
                % only need to calculate station distance for nominal case
                if nominal_or_width_based == 1
                    nominal_init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
                end

                %% find midpoint of route
                navigated_distance = nominal_init_route_length*navigated_portion; % distance along init path to place replanning point
                St_points_input = [navigated_distance 0]; % ST coordinates so station is the navigated distance and T is 0 (i.e. we're on the path)
                referencePath = init_route(:,1:2);
                flag_snap_type = 1;
                start_midway = fcn_Path_convertSt2XY(referencePath,St_points_input, flag_snap_type);

                %% enlarge polytopes
                enlarged_polytopes = fcn_MapGen_polytopesExpandEvenlyForConcave(shrunk_polytopes,polytope_size_increases);

                % enlarging polytopes may have put the midway start inside a polytope
                % for each polytope, check if this point is inside the polytope and snap to a nearby vertex if so
                pts_to_test = [start_midway; finish_init];
                output_pts = fcn_MapGen_snapInteriorPointToVertex(enlarged_polytopes, pts_to_test);
                start_midway = output_pts(1,:);
                finish_init = output_pts(2,:);

                %% plan the new path
                % generate updated all_pts array
                [all_pts_new, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(enlarged_polytopes, start_midway, finish_init);

                % make vgraph again
                finishes = [all_pts_new; start; finish];
                starts = [all_pts_new; start; finish];
                [new_vgraph, visibility_results_all_pts_new] = fcn_Visibility_clearAndBlockedPointsGlobal(enlarged_polytopes, starts, finishes,1);
                reduced_vgraph = new_vgraph;
                % get vgraph stats
                num_edges_initially = sum(sum(vgraph));
                num_edges_finally = sum(sum(new_vgraph));
                num_edges_removed = num_edges_initially - num_edges_finally;
                pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
                % make rgraph again
                [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(new_vgraph,start(3),finish(3));
                if ~is_reachable
                    % we don't want to break if replanning is impossible, we want to save the data for what caused this
                    warning('mission replanning is impossible')
                    replan_cost = NaN;
                    replan_route_length = NaN;
                    data = [data; str2num(strcat(num2str(map_idx),num2str(mission_idx))) nominal_or_width_based polytope_size_increases polytope_size_increases init_route_length navigated_distance replan_route_length G_bar_old];
                    continue
                end % end is_reachable condition for replanning

                % make cgraph again
                mode = "xy spatial only";
                [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts_new, start, finish, mode);
                % replan path
                [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts_new, start, finish);

                % find replan route length
                route_x = replan_route(:,1);
                route_y = replan_route(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));

                % save data from this trial
                data = [data; str2num(strcat(num2str(map_idx),num2str(mission_idx))) nominal_or_width_based polytope_size_increases polytope_size_increases init_route_length navigated_distance replan_route_length G_bar_old];
                %% plot single trial
                % plot field, initial path, replan path, and midway point
                if flag_do_plot
                    figure; hold on; box on;
                    xlabel('x [km]');
                    ylabel('y [km]');
                    plot(start_init(1),start_init(2),'xg','MarkerSize',6);
                    plot(finish(1),finish(2),'xr','MarkerSize',6);
                    plot(init_route(:,1),init_route(:,2),'k','LineWidth',2);
                    if flag_do_threadpulling && nominal_or_width_based==2
                        plot(init_route_original(:,1), init_route_original(:,2),'--','Color',[0.5 0.5 0.5],'LineWidth',2);
                    end
                    plot(start_midway(1),start_midway(2),'dm','MarkerSize',6,'MarkerFaceColor','m')
                    plot(replan_route(:,1),replan_route(:,2),'--g','LineWidth',2);
                    for j = 1:length(enlarged_polytopes)
                         fill(enlarged_polytopes(j).vertices(:,1)',enlarged_polytopes(j).vertices(:,2),[0 0 1],'FaceColor','r','FaceAlpha',0.3)
                    end
                    for j = 1:length(shrunk_polytopes)
                         fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
                    end
                    title_string = sprintf('map idx: %i, nominal or corridor-width-based: %i,\npolytope size increase [km]: %.2f',str2num(strcat(num2str(map_idx),num2str(mission_idx))), nominal_or_width_based,polytope_size_increases);
                    title(title_string);
                    leg_str = {'start','finish','initial route','replanning point','replanned route','enlarged obstacles'};
                    if flag_do_threadpulling && nominal_or_width_based==2
                        leg_str = {'start','finish','initial route, shortened','initial route','replanning point','replanned route','enlarged obstacles'};
                    end
                    for i = 1:length(shrunk_polytopes)-1
                        leg_str{end+1} = '';
                    end
                    leg_str{end+1} = 'obstacles';
                    for i = 1:length(shrunk_polytopes)-1
                        leg_str{end+1} = '';
                    end
                    leg_str{end+1} = 'blocked initial path segment';
                    if flag_do_plot_slow
                        for waypoint_id = 1:(size(init_route,1)-1)
                            route_segment_start = init_route(waypoint_id,:);
                            route_segment_end = init_route(waypoint_id+1,:);

                            route_start_idx = all_pts(:,1) == route_segment_start(1) & all_pts(:,2) == route_segment_start(2);
                            route_end_idx = all_pts(:,1) == route_segment_end(1) & all_pts(:,2) == route_segment_end(2);
                            if ~new_vgraph(route_start_idx,route_end_idx)
                                plot([route_segment_start(1),route_segment_end(1)],[route_segment_start(2),route_segment_end(2)],"Color",'r',"LineWidth",3)
                            end
                        end
                    end
                    legend(leg_str,'Location','best');
                    if flag_do_plot_slow
                        figure; hold on; box on;
                        blues = zeros(size(orig_vgraph));
                        num_orig_nodes = size(orig_vgraph,1);
                        reduced_vgraph_concat = reduced_vgraph(1:num_orig_nodes,1:num_orig_nodes);
                        reds = orig_vgraph & ~reduced_vgraph_concat;
                        greens = reduced_vgraph_concat;
                        vgraph_image(:,:,1) = reds;
                        vgraph_image(:,:,2) = greens;
                        vgraph_image(:,:,3) = blues;
                        imshow(vgraph_image*255);
                        num_edges_initially = sum(sum(orig_vgraph));
                        num_edges_ultimately = sum(sum(reduced_vgraph_concat));
                        pct_edges_removed = (num_edges_initially - num_edges_ultimately)/num_edges_initially;
                        title(sprintf("%.2f pct. of edges removed, obstacle dilation",pct_edges_removed));
                    end
                end % end flag_do_plot condition
            end % end nominal or corridor width cost function loop
        end % end edge dilation size loop
    end % end mission (i.e., start goal pair) loop
end % end map loop
%% plot multiple trials data
% sort data and define plot options
markers = {'x','d','o','+','s','x','^','v','pentagram'};
colors = {'r','b'};
% idx_nominal = data(:,1)== map_idx & data(:,2)==1 & ~isnan(data(:,6)) & ~isnan(data(:,7)); % filter on map and nominal and not NaN
idx_nominal = data(:,2)==1;% & ~isnan(data(:,6)) & ~isnan(data(:,7));
nominal_data = data(idx_nominal,:);
% idx_reachable = data(:,1)== map_idx & data(:,2)==2 & ~isnan(data(:,6)) & ~isnan(data(:,7)); % filter on map and feature and not NaN
idx_reachable = data(:,2)==2;% & ~isnan(data(:,6)) & ~isnan(data(:,7));
reachable_data = data(idx_reachable,:);

%% make a smooth curve ratio of feature to nominal to show cost savings
% [1 map_idx,mission_idx
% 2 nominal_or_width_based
% 3 polytope_size_increases
% 4 polytope_size_increases
% 5 init_route_length
% 6 navigated_distance
% 7 replan_route_length];
idx_nominal_data_including_fails = find(data(:,2)==1);
data_discount_ratio = nan(size(data,1)/2, 3);
for d = 1:size(idx_nominal_data_including_fails,1)
    nominal_datum = data(idx_nominal_data_including_fails(d),:);
    % grab the datum from the feature cost function with the same map+mission id and same dilation
    feature_datum = data((data(:,2)==2) & data(:,1) == nominal_datum(1) & data(:,3) == nominal_datum(3),:);
    nominal_length = nominal_datum(6)+nominal_datum(7);
    feature_length = feature_datum(6)+feature_datum(7);
    if isnan(feature_length)
        feature_length = inf;
    end
    if isnan(nominal_length)
        nominal_length = inf;
    end
    discount_ratio = feature_length/nominal_length;
    if isnan(discount_ratio)
        continue
    end
    if isinf(discount_ratio)
        continue
    end
    data_discount_ratio(d,:) = [feature_datum(1) feature_datum(4)./feature_datum(8) discount_ratio];
end
% plot this
unique_maps = unique(data_discount_ratio(:,1));
figure; hold on; box on;
xlabel('obstacle size increase ratio (relative to average gap size)')
ylabel('path length ratio (feature vs. nominal cost function)')
line_widths = 2;
for u_map_id = 1:length(unique_maps)
    unique_map = unique_maps(u_map_id);
    data_discount_ratio_this_map = data_discount_ratio(data_discount_ratio(:,1) == unique_map,:);
    plot(data_discount_ratio_this_map(:,2), data_discount_ratio_this_map(:,3),'--', 'LineWidth',line_widths,'MarkerSize',4);
end
plot([min(data_discount_ratio(:,2)) max(data_discount_ratio(:,2))], [1 1], 'k--')

%% make a smooth curve ratio of each cost after replanning to nominal initial path length
% [1 map_idx,mission_idx
% 2 nominal_or_width_based
% 3 polytope_size_increases
% 4 polytope_size_increases
% 5 init_route_length
% 6 navigated_distance
% 7 replan_route_length];
ratios_to_min_dist = nan(size(data,1), 4);
for d = 1:size(data,1)
    datum = data(d,:);
    % grab the reference datum from the nominal cost function with the same map+mission id and same dilation
    reference_datum = data((data(:,2)==1) & data(:,1) == datum(1) & data(:,3) == datum(3),:);
    % TODO are there ever nan nominals? waht si nan/n what is n/nan
    nominal_init_length = reference_datum(5);
    replan_length = datum(6)+datum(7);
    ratio_to_min_dist = replan_length/nominal_init_length;
    if isnan(ratio_to_min_dist)
        continue
    end
    ratios_to_min_dist(d,:) = [datum(1) datum(2) datum(4)./datum(8) ratio_to_min_dist];
end
% plot this
unique_maps = unique(ratios_to_min_dist(:,1));
figure; hold on; box on;
xlabel('obstacle size increase ratio (relative to average gap size)')
ylabel('path length ratio (relative to min. distance path)')
plot([min(data(:,4)./data(:,8)) max(data(:,4)./data(:,8))], [1 1], 'k--')
for u_map_id = 1:length(unique_maps)
    unique_map = unique_maps(u_map_id);
    min_dist_ratio_this_map_nominal = ratios_to_min_dist(ratios_to_min_dist(:,1) == unique_map & ratios_to_min_dist(:,2) == 1,:);
    min_dist_ratio_this_map_feature = ratios_to_min_dist(ratios_to_min_dist(:,1) == unique_map & ratios_to_min_dist(:,2) == 2,:);
    plot(min_dist_ratio_this_map_nominal(:,3), min_dist_ratio_this_map_nominal(:,4), '-x', 'Color', colors{1}, 'LineWidth',1,'MarkerSize',4);
    plot(min_dist_ratio_this_map_feature(:,3), min_dist_ratio_this_map_feature(:,4), '-x', 'Color', colors{2}, 'LineWidth',1,'MarkerSize',4);
end
legend({'optimal length','nominal cost function','corridor width function'},'Location','best');
%% make a smooth curve ratio of each cost after replanning to nominal initial path length AVERAGED
% [1 map_idx,mission_idx
% 2 nominal_or_width_based
% 3 polytope_size_increases
% 4 polytope_size_increases
% 5 init_route_length
% 6 navigated_distance
% 7 replan_route_length];
ratios_to_min_dist = nan(size(data,1), 4);
for d = 1:size(data,1)
    datum = data(d,:);
    % grab the reference datum from the nominal cost function with the same map+mission id and same dilation
    reference_datum = data((data(:,2)==1) & data(:,1) == datum(1) & data(:,3) == datum(3),:);
    % TODO are there ever nan nominals? waht si nan/n what is n/nan
    nominal_init_length = reference_datum(5);
    replan_length = datum(6)+datum(7);
    ratio_to_min_dist = replan_length/nominal_init_length;
    if isnan(ratio_to_min_dist)
        continue
    end
    %                          map id   nom or feat    gap (xaxis)   ratio  (yaxis)
    ratios_to_min_dist(d,:) = [datum(1) datum(2) datum(4)./datum(8) ratio_to_min_dist];
end
% plot this
unique_gaps = unique(ratios_to_min_dist(:,3));
figure; hold on; box on;
xlabel('obstacle size increase ratio (relative to average gap size)')
ylabel('path length ratio (relative to min. distance path)')
plot([min(data(:,4)./data(:,8)) max(data(:,4)./data(:,8))], [1 1], 'k--')
min_dist_ratio_all_maps_nominal = [];
min_dist_ratio_all_maps_feature = [];
min_dist_ratio_all_maps_nominal_std = [];
min_dist_ratio_all_maps_feature_std = [];
for u_gap_id = 1:length(unique_gaps)
    unique_gap = unique_gaps(u_gap_id);
    min_dist_ratio_this_gap_nominal = ratios_to_min_dist(ratios_to_min_dist(:,3) == unique_gap & ratios_to_min_dist(:,2) == 1,:);
    min_dist_ratio_this_gap_feature = ratios_to_min_dist(ratios_to_min_dist(:,3) == unique_gap & ratios_to_min_dist(:,2) == 2,:);
    min_dist_ratio_all_maps_nominal = [min_dist_ratio_all_maps_nominal; mean(min_dist_ratio_this_gap_nominal(:,4),'omitnan')];
    min_dist_ratio_all_maps_feature = [min_dist_ratio_all_maps_feature; mean(min_dist_ratio_this_gap_feature(:,4),'omitnan')];
    min_dist_ratio_all_maps_nominal_std = [min_dist_ratio_all_maps_nominal_std; std(min_dist_ratio_this_gap_nominal(:,4),'omitnan')];
    min_dist_ratio_all_maps_feature_std = [min_dist_ratio_all_maps_feature_std; std(min_dist_ratio_this_gap_feature(:,4),'omitnan')];
end
plot(unique_gaps, min_dist_ratio_all_maps_nominal, '-x', 'Color', colors{1}, 'LineWidth',1,'MarkerSize',4);
plot(unique_gaps, min_dist_ratio_all_maps_feature, '-x', 'Color', colors{2}, 'LineWidth',1,'MarkerSize',4);
errorbar(unique_gaps, min_dist_ratio_all_maps_nominal, min_dist_ratio_all_maps_nominal_std, '-x', 'Color', colors{1}, 'LineWidth',1,'MarkerSize',4);
errorbar(unique_gaps, min_dist_ratio_all_maps_feature, min_dist_ratio_all_maps_feature_std, '-x', 'Color', colors{2}, 'LineWidth',1,'MarkerSize',4);
legend({'optimal length','nominal cost function','corridor width function'},'Location','best');
%% same as above but with average of average ga
ratios_to_min_dist = nan(size(data,1), 4);
avg_avg_gap = mean(data(:,8),'omitnan');
for d = 1:size(data,1)
    datum = data(d,:);
    % grab the reference datum from the nominal cost function with the same map+mission id and same dilation
    reference_datum = data((data(:,2)==1) & data(:,1) == datum(1) & data(:,3) == datum(3),:);
    % TODO are there ever nan nominals? waht si nan/n what is n/nan
    nominal_init_length = reference_datum(5);
    replan_length = datum(6)+datum(7);
    ratio_to_min_dist = replan_length/nominal_init_length;
    if isnan(ratio_to_min_dist)
        continue
    end
    %                          map id   nom or feat    gap (xaxis)   ratio  (yaxis)
    ratios_to_min_dist(d,:) = [datum(1) datum(2) datum(4)./avg_avg_gap ratio_to_min_dist];
end
% plot this
unique_gaps = unique(ratios_to_min_dist(:,3));
figure; hold on; box on;
xlabel('obstacle size increase ratio (relative to average gap size)')
ylabel('path length ratio (relative to min. distance path)')
plot([min(data(:,4)./avg_avg_gap) max(data(:,4)./avg_avg_gap)], [1 1], 'k--')
min_dist_ratio_all_maps_nominal = [];
min_dist_ratio_all_maps_feature = [];
min_dist_ratio_all_maps_nominal_std = [];
min_dist_ratio_all_maps_feature_std = [];
for u_gap_id = 1:length(unique_gaps)
    unique_gap = unique_gaps(u_gap_id);
    min_dist_ratio_this_gap_nominal = ratios_to_min_dist(ratios_to_min_dist(:,3) == unique_gap & ratios_to_min_dist(:,2) == 1,:);
    min_dist_ratio_this_gap_feature = ratios_to_min_dist(ratios_to_min_dist(:,3) == unique_gap & ratios_to_min_dist(:,2) == 2,:);
    min_dist_ratio_all_maps_nominal = [min_dist_ratio_all_maps_nominal; mean(min_dist_ratio_this_gap_nominal(:,4),'omitnan')];
    min_dist_ratio_all_maps_feature = [min_dist_ratio_all_maps_feature; mean(min_dist_ratio_this_gap_feature(:,4),'omitnan')];
    min_dist_ratio_all_maps_nominal_std = [min_dist_ratio_all_maps_nominal_std; std(min_dist_ratio_this_gap_nominal(:,4),'omitnan')];
    min_dist_ratio_all_maps_feature_std = [min_dist_ratio_all_maps_feature_std; std(min_dist_ratio_this_gap_feature(:,4),'omitnan')];
end
plot(unique_gaps, min_dist_ratio_all_maps_nominal, '-x', 'Color', colors{1}, 'LineWidth',1,'MarkerSize',4);
plot(unique_gaps, min_dist_ratio_all_maps_feature, '-x', 'Color', colors{2}, 'LineWidth',1,'MarkerSize',4);

legend({'optimal length','nominal cost function','corridor width function'},'Location','best');


%% plot scatter of ratio of each path relative to its own initial path
idx_nominal = data(:,2)==1;% & ~isnan(data(:,6)) & ~isnan(data(:,7));
nominal_data = data(idx_nominal,:);
% idx_reachable = data(:,1)== map_idx & data(:,2)==2 & ~isnan(data(:,6)) & ~isnan(data(:,7)); % filter on map and feature and not NaN
idx_reachable = data(:,2)==2;% & ~isnan(data(:,6)) & ~isnan(data(:,7));
reachable_data = data(idx_reachable,:);
idx_nominal = data(:,2)==1 & ~isnan(data(:,6)) & ~isnan(data(:,7));
nominal_data = data(idx_nominal,:);
idx_reachable = data(:,2)==2 & ~isnan(data(:,6)) & ~isnan(data(:,7));
reachable_data = data(idx_reachable,:);
figure; hold on; box on;
% loop over each mission idx to plot different markers for different missions
figure; hold on; box on;
for u_map_id = 1:length(unique_maps)
    this_mission = unique_maps(u_map_id);
    idx_nominal_this_mission = nominal_data(:,1) == this_mission;
    nominal_data_this_mission = nominal_data(idx_nominal_this_mission,:);
    idx_reachable_this_mission = reachable_data(:,1) == this_mission;
    reachable_data_this_mission = reachable_data(idx_reachable_this_mission,:);
    if isempty(nominal_data_this_mission)
        nominal_data_this_mission = nan(1,7);
        nominal_data_this_mission(1,1) = this_mission;
    end
    if isempty(reachable_data_this_mission)
        reachable_data_this_mission= nan(1,7);
        reachable_data_this_mission(1,1) = this_mission;
    end
    plot(nominal_data_this_mission(:,4),(nominal_data_this_mission(:,6)+nominal_data_this_mission(:,7))./nominal_data_this_mission(:,5),"Color",colors{1},"Marker",markers{mod(u_map_id,length(markers))+1},'LineStyle','none');
    plot(reachable_data_this_mission(:,4),(reachable_data_this_mission(:,6)+reachable_data_this_mission(:,7))./reachable_data_this_mission(:,5),"Color",colors{2},"Marker",markers{mod(u_map_id,length(markers))+1},'LineStyle','none');
end
% add polynominal fit
fit_order = 3;
p_nominal = polyfit(nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))./nominal_data(:,5),fit_order);
p_reachable = polyfit(reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))./reachable_data(:,5),fit_order);
x_for_poly = linspace(min(nominal_data(:,4)),max(nominal_data(:,4)),100);
% plot(x_for_poly,polyval(p_nominal,x_for_poly),'Color',colors{nominal_data(1,2)},'LineWidth',2);
% plot(x_for_poly,polyval(p_reachable,x_for_poly),'Color',colors{reachable_data(1,2)},'LineWidth',2);
% add convex hull
P_nominal = [nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))./nominal_data(:,5)];
P_reachable = [reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))./reachable_data(:,5)];
k_nominal = convhull(P_nominal);
k_reachable = convhull(P_reachable);
fill(P_nominal(k_nominal,1),P_nominal(k_nominal,2),colors{nominal_data(1,2)},'FaceAlpha',0.3);
fill(P_reachable(k_reachable,1),P_reachable(k_reachable,2),colors{reachable_data(1,2)},'FaceAlpha',0.3);
% add tight non-convex boundary
k_nominal_nonconvex = boundary(P_nominal);
k_reachable_nonconvex = boundary(P_reachable);
% fill(P_nominal(k_nominal_nonconvex,1),P_nominal(k_nominal_nonconvex,2),colors{nominal_data(1,2)},'FaceAlpha',0.5);
% fill(P_reachable(k_reachable_nonconvex,1),P_reachable(k_reachable_nonconvex,2),colors{reachable_data(1,2)},'FaceAlpha',0.5);
% legends and labels
ylabel(sprintf('ratio of replanned path length to width incentive \nor nominal initial path length'))
xlabel('obstacle size increase [km]')
legend({'nominal cost function','corridor width function'},'Location','best');

%% plot scatter ratio of each path relative to nominal initial path
idx_nominal = data(:,2)==1 & ~isnan(data(:,6))% & ~isnan(data(:,7));
nominal_data = data(idx_nominal,:);
idx_reachable = data(:,2)==2 & ~isnan(data(:,6))% & ~isnan(data(:,7));
reachable_data = data(idx_reachable,:);
figure; hold on; box on;
for u_map_id = 1:length(unique_maps)
    this_mission = unique_maps(u_map_id);
    idx_nominal_this_mission = nominal_data(:,1) == this_mission;
    nominal_data_this_mission = nominal_data(idx_nominal_this_mission,:);
    idx_reachable_this_mission = reachable_data(:,1) == this_mission;
    reachable_data_this_mission = reachable_data(idx_reachable_this_mission,:);
    if isempty(nominal_data_this_mission)
        nominal_data_this_mission = nan(1,7);
        nominal_data_this_mission(1,1) = this_mission;
    end
    if isempty(reachable_data_this_mission)
        reachable_data_this_mission= nan(1,7);
        reachable_data_this_mission(1,1) = this_mission;
    end
    plot(nominal_data_this_mission(:,4),(nominal_data_this_mission(:,6)+nominal_data_this_mission(:,7))./nominal_data_this_mission(:,5),"Color",colors{1},"Marker",markers{mod(u_map_id,length(markers))+1},'LineStyle','none');
    plot(reachable_data_this_mission(:,4),(reachable_data_this_mission(:,6)+reachable_data_this_mission(:,7))./nominal_data_this_mission(:,5),"Color",colors{2},"Marker",markers{mod(u_map_id,length(markers))+1},'LineStyle','none');
end
% legends and labels
ylabel('ratio of replanned path length to nominal initial path length')
xlabel('obstacle size increase [km]')
legend({'nominal cost function','corridor width cost function'},'Location','best');

%% plot scatter of absolute length
idx_nominal = data(:,2)==1 & ~isnan(data(:,6)) & ~isnan(data(:,7));
nominal_data = data(idx_nominal,:);
idx_reachable = data(:,2)==2 & ~isnan(data(:,6)) & ~isnan(data(:,7));
reachable_data = data(idx_reachable,:);
figure; hold on; box on;
for u_map_id = 1:length(unique_maps)
    this_mission = unique_maps(u_map_id);
    idx_nominal_this_mission = nominal_data(:,1) == this_mission;
    nominal_data_this_mission = nominal_data(idx_nominal_this_mission,:);
    idx_reachable_this_mission = reachable_data(:,1) == this_mission;
    reachable_data_this_mission = reachable_data(idx_reachable_this_mission,:);
    if isempty(nominal_data_this_mission)
        nominal_data_this_mission = nan(1,7);
        nominal_data_this_mission(1,1) = this_mission;
    end
    if isempty(reachable_data_this_mission)
        reachable_data_this_mission= nan(1,7);
        reachable_data_this_mission(1,1) = this_mission;
    end
    plot(nominal_data_this_mission(:,4),(nominal_data_this_mission(:,6)+nominal_data_this_mission(:,7)),"Color",colors{1},"Marker",markers{mod(u_map_id,length(markers))+1},'LineStyle','none');
    plot(reachable_data_this_mission(:,4),(reachable_data_this_mission(:,6)+reachable_data_this_mission(:,7)),"Color",colors{2},"Marker",markers{mod(u_map_id,length(markers))+1},'LineStyle','none');
end
% add polynominal fit
p_nominal = polyfit(nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7)),fit_order);
p_reachable = polyfit(reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7)),fit_order);
x_for_poly = linspace(min(nominal_data(:,4)),max(nominal_data(:,4)),100);
% plot(x_for_poly,polyval(p_nominal,x_for_poly),'Color',colors{nominal_data(1,2)},'LineWidth',2);
% plot(x_for_poly,polyval(p_reachable,x_for_poly),'Color',colors{reachable_data(1,2)},'LineWidth',2);
% add convex hull
P_nominal = [nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))];
P_reachable = [reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))];
k_nominal = convhull(P_nominal);
k_reachable = convhull(P_reachable);
fill(P_nominal(k_nominal,1),P_nominal(k_nominal,2),colors{nominal_data(1,2)},'FaceAlpha',0.3);
fill(P_reachable(k_reachable,1),P_reachable(k_reachable,2),colors{reachable_data(1,2)},'FaceAlpha',0.3);
% add tight non-convex boundary
k_nominal_nonconvex = boundary(P_nominal);
k_reachable_nonconvex = boundary(P_reachable);
% fill(P_nominal(k_nominal_nonconvex,1),P_nominal(k_nominal_nonconvex,2),colors{nominal_data(1,2)},'FaceAlpha',0.5);
% fill(P_reachable(k_reachable_nonconvex,1),P_reachable(k_reachable_nonconvex,2),colors{reachable_data(1,2)},'FaceAlpha',0.5);
% legends and labels
xlabel('obstacle size increase [km]')
ylabel('total path length after replanning [km]')
legend({'nominal cost function','corridor width function'},'Location','best');

%% plot histogram of failed trials
% find data for failed replanning
nandata = data(find(isnan(data(:,7))),:); % find nan replan cost rows
nandata_nominal = nandata(nandata(:,2)==1,:); % of those, find nominal ones
nandata_reachable = nandata(nandata(:,2)==2,:); % of those, find reachable ones
% find data for successful initial planning to normalize the above data
init_success_data = data(find(~isnan(data(:,5))),:); % find non-nan initial path cost rows
init_success_data_nominal = init_success_data(init_success_data(:,2)==1,:); % of those, find nominal ones
init_success_data_reachable = init_success_data(init_success_data(:,2)==2,:); % of those, find reachable ones
% polytope_size_bins = [0.005 0.015 0.03 0.075 0.15 0.25 0.35 0.45 0.55];
sizes = [0.01 0.02 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9  0.95 1];
polytope_size_bins = [0 diff(sizes)./2] + sizes;

figure; hold on; box on;
n_bins = 18;
x_bins = 0.02:0.02:0.2;
h1 = histogram(nandata_nominal(:,4)./nandata_nominal(:,8),n_bins); % polytope dilations for nan data
h2 = histogram(nandata_reachable(:,4)./nandata_reachable(:,8),n_bins); % polytope dilations for nan data
% h1 = histogram(nandata_nominal(:,4),13); % polytope dilations for nan data
% h2 = histogram(nandata_reachable(:,4),13); % polytope dilations for nan data
% h1 = histogram(nandata_nominal(:,4),22); % polytope dilations for nan data
% h2 = histogram(nandata_reachable(:,4),22); % polytope dilations for nan data
h1.FaceColor = 'r';
h2.FaceColor = 'b';
xlabel('obstacle size increase [km]')
ylabel('count of failed replanning attempts');
legend({'nominal cost function','corridor width function'},'Location','best');

% count normilized dilations for failed attempts
h1data = hist(nandata_nominal(:,4)./nandata_nominal(:,8),x_bins); % normilized polytope dilations for nan data
h2data = hist(nandata_reachable(:,4)./nandata_reachable(:,8),x_bins); % normilized polytope dilations for nan data
% count normilized dilations for initial successes
h1data_denom = hist(init_success_data_nominal(:,4)./init_success_data_nominal(:,8),x_bins); % normilized polytope dilations for nan data
h2data_denom = hist(init_success_data_reachable(:,4)./init_success_data_reachable(:,8),x_bins); % normilized polytope dilations for nan data
h1_norm = h1data./h1data_denom;
h2_norm = h2data./h2data_denom;

figure; hold on; box on;
bar(x_bins, h1_norm,'FaceColor','r','FaceAlpha',0.5)
bar(x_bins, h2_norm,'FaceColor','b','FaceAlpha',0.5)
xlabel('obstacle size increase [km]')
ylabel('count of failed replanning attempts');
legend({'nominal cost function','corridor width function'},'Location','best');

%% save all figs from this bag and the workspace, then close them
if flag_save_plots
    FolderName = './data_output';
    mkdir(FolderName)  % Your destination folder
    FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
    cd(FolderName)
    for iFig = 1:length(FigList)
      FigHandle = FigList(iFig);
      FigName   = get(FigHandle, 'Number');
      savefig(FigHandle, strcat(num2str(FigName), '.fig'));
      saveas(FigHandle, strcat(num2str(FigName), '.png'));
    end
    save('data.mat','data');
    cd ..
end

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
