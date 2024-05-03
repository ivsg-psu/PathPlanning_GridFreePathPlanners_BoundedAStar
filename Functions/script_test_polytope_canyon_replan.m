% script_test__polytope_canyon_replan
% this script plans two paths: (1) a distance reducing path and (2) a path that visits more connected
% nodes. Then random edges are deleted from the vgraph and replanning is triggered from midway down the
% initial path. The final paths are then compared to show that replanning from the more connected path
% is less costly and more likely to be successful.

clear; close all; clc
addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% plotting flags
flag_do_plot = 1;
flag_do_plot_slow = 0;

%% mission options
map_idx =5;
navigated_portion = 0.5; % portion of initial path to be completed prior to triggering replanning
random_delete = 0; % toggle on random edge deletion vs length based deletion where long edges are deleted first (assumed to be more likely to be blocked as they cover more ground)
num_repeats = 1; % since there is random edge deletion, the user may want to run each trial multiple times
w = 10000; % relative weighting of heuristic cost function, heuristic_cost = dist_to_goal + w*num_visible_nodes + w*num_reachable_nodes
[shrunk_polytopes, start_inits, finish_inits] = fcn_util_load_test_map(map_idx);

% map_ID nominal_or_reachable edge_deletion(edge_deletion_idx) pct_edges_removed init_route_length navigated_distance replan_route_length
data = []; % initialize array for storing results
for mission_idx = 1:size(start_inits,1)
    start_init = start_inits(mission_idx,:);
    finish_init = finish_inits(mission_idx,:);
    % all_pts array creation
    % TODO @sjharnett make a function for all pts creation
    point_tot = length([shrunk_polytopes.xv]); % total number of vertices in the polytopes
    beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    for poly = 1:size(shrunk_polytopes,2) % check each polytope
        verts = length(shrunk_polytopes(poly).xv);
        shrunk_polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
        beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
        curpt = curpt+verts;
    end
    obs_id = [shrunk_polytopes.obs_id];
    all_pts = [[shrunk_polytopes.xv];[shrunk_polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]

    % loop over multiple trials as there is randomness in edge deletion so we may wish to repeat the experiment
    for repeats = 1:num_repeats
        % loop over portions of vgraph edges to delete
        edge_deletion = 0:0.05:0.9;
        for edge_deletion_idx = 1:length(edge_deletion)
            % loop over the nominal cost function and feature cost function
            for nominal_or_reachable = [1,2]
                %% plan the initial path
                % make vgraph
                start = [start_init size(all_pts,1)+1 -1 1];
                finish = [finish_init size(all_pts,1)+2 -1 1];
                finishes = [all_pts; start; finish];
                starts = [all_pts; start; finish];
                [vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);
                orig_vgraph = vgraph; % store to compare to the replanning vgrpah
                % make rgraph
                [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
                if ~is_reachable
                    error('initial mission, prior to edge deletion, is not possible')
                end

                % make cgraph
                % feature cost function prioritizing reachability
                reachable_nodes_from_each_node = sum(rgraph,2);
                inv_reach_cost = w*(1./(reachable_nodes_from_each_node))'; % want inverse so more reachable nodes cost less
                % feature cost function prioritizing visibility
                visible_nodes_from_each_node = sum(vgraph,2);
                inv_vis_cost = w*(1./(visible_nodes_from_each_node))'; % want inverse so more visible nodes cost less
                mode = "xy spatial only";
                [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
                if nominal_or_reachable == 2
                    hvec = hvec + inv_reach_cost + inv_vis_cost; % makes more sense to combine visibility metrics with heuristic rather then cgraph because there is a visibility/reachability value per node not per edge
                end
                % plan initial path
                [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

                % find init route length
                route_x = init_route(:,1);
                route_y = init_route(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                init_route_length = sum(sqrt(sum(lengths.*lengths,2)));

                %% find midpoint of route
                navigated_distance = init_route_length*navigated_portion; % distance along init path to place replanning point
                St_points_input = [navigated_distance 0]; % ST coordinates so station is the navigated distance and T is 0 (i.e. we're on the path)
                referencePath = init_route(:,1:2);
                flag_snap_type = 1;
                start_midway = fcn_Path_convertSt2XY(referencePath,St_points_input, flag_snap_type);

                %% delete vgraph edges
                % make vgraph again because we have a new start point
                start = [start_midway size(all_pts,1)+1 -1 1];
                finish = [finish_init size(all_pts,1)+2 -1 1];
                finishes = [all_pts; start; finish];
                starts = [all_pts; start; finish];
                [vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);
                % we only need to delete edges on the first go through a given portion, on the second go round when we run the feature cost function...
                % we want to keep the same deletion as when we ran the nominal so we have a fair comparison of the cost functions
                if nominal_or_reachable == 1
                    desired_portion_edge_deletion = edge_deletion(edge_deletion_idx);
                    % never want to disconnect start and finish from vgraph so leave them
                    vgraph_without_start_and_fin = vgraph(1:end-2,1:end-2);
                    valid_edges_initially = find(vgraph_without_start_and_fin==1); % number of edges prior to modification
                    num_edges_initially = length(valid_edges_initially);
                    if random_delete
                        % if we want every edge to have an equal cahnce of being deleted...
                        edge_lottery_draw = rand(num_edges_initially,1); % draw a random number per edge
                        edges_for_removal = (edge_lottery_draw <= desired_portion_edge_deletion); % if we want 70% of edges gone, you have to draw over 0.7 from a uniform dist to survive the cut
                        idx_of_edges_for_removal = valid_edges_initially(edges_for_removal); % note which edges failed the draw
                    else
                        % it's possible that longer edges are more likely to be deleted since they cover more ground...
                        % thus we may want to delete edges in length order
                        % get the length based cgraph
                        mode = "xy spatial only";
                        [cgraph, ~] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
                        cgraph_without_start_and_fin = cgraph(1:end-2,1:end-2);
                        costs_of_valid_edges = cgraph_without_start_and_fin(valid_edges_initially); % cost of every 1 in vgraph
                        vgraph_edge_idx_to_cost_table = [valid_edges_initially, costs_of_valid_edges]; % associate costs to edges in a table
                        vgraph_edge_idx_sorted = sortrows(vgraph_edge_idx_to_cost_table,2,'descend'); % sort the table by edge length
                        num_edges_for_removal = num_edges_initially*edge_deletion(edge_deletion_idx); % find out how many edges to remove
                        idx_of_edges_for_removal = vgraph_edge_idx_sorted(1:num_edges_for_removal,1); % take that many edges from top of sorted list
                    end % end condition for random delete vs length based delete
                    [rows_for_removal, cols_for_removal] = ind2sub(size(vgraph_without_start_and_fin),idx_of_edges_for_removal);
                    % note how many edges we actually removed
                    num_edges_removed = length(rows_for_removal);
                    pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
                end % end edge deletion condition (for nominal pass only)
                % even though we only identify edges for removal on the first pass through (for the nomianl cost function)
                % we execute the edge removal on every iteration, now that we've identified edges for removal
                % this means the nominal and feature iterations each have their own vgraph (necessary since they have different midpoints)
                % but they will have the same edges deleted
                new_vgraph = vgraph;
                % vgraph stats
                num_edges_initally_updated = sum(sum(vgraph));
                idx_of_edges_for_removal_updated = sub2ind(size(new_vgraph),rows_for_removal,cols_for_removal);
                new_vgraph(idx_of_edges_for_removal_updated) = 0;
                num_edges_after = sum(sum(new_vgraph));
                pct_edges_removed_updated = (num_edges_initally_updated - num_edges_after)/num_edges_initally_updated*100;
                reduced_vgraph = new_vgraph;

                %% plan the new path
                % make rgraph again
                [is_reachable, num_steps, rgraph] = fcn_check_reachability(new_vgraph,start(3),finish(3));
                if ~is_reachable
                    % we don't want to break if replanning is impossible, we want to save the data for what caused this
                    warning('mission replanning is impossible')
                    replan_cost = NaN;
                    replan_route_length = NaN;
                    data = [data; map_idx nominal_or_reachable edge_deletion(edge_deletion_idx) pct_edges_removed init_route_length navigated_distance replan_route_length];
                    continue
                end % end is_reachable condition for replanning

                % make cgraph again
                mode = "xy spatial only";
                % mode = 'time or z only';
                % mode = "xyz or xyt";
                [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
                % replan route
                [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts, start, finish);

                % find replan route length
                route_x = replan_route(:,1);
                route_y = replan_route(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));
                % store the data
                % map_ID nominal_or_reachable edge_deletion initial_distance navigated_distance replan_route_length
                data = [data; map_idx nominal_or_reachable edge_deletion(edge_deletion_idx) pct_edges_removed init_route_length navigated_distance replan_route_length];
                %% plot a single trial
                % plot field, initial path, replan path, and midway point
                if flag_do_plot
                    figure; hold on; box on;
                    xlabel('x [km]');
                    ylabel('y [km]');
                    plot(start_init(1),start_init(2),'xg','MarkerSize',6);
                    plot(finish(1),finish(2),'xr','MarkerSize',6);
                    plot(init_route(:,1),init_route(:,2),'k','LineWidth',2);
                    plot(start_midway(1),start_midway(2),'dm','MarkerSize',6)
                    plot(replan_route(:,1),replan_route(:,2),'--g','LineWidth',2);
                    for j = 1:length(shrunk_polytopes)
                         fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
                    end
                    title_string = sprintf('map idx: %i, nominal or reachable: %i, pct edges removed: %.1f',map_idx, nominal_or_reachable,pct_edges_removed);
                    title(title_string);
                    leg_str = {'start','finish','initial route','replanning point','replanned route','obstacles'};
                    for i = 1:length(shrunk_polytopes)-1
                        leg_str{end+1} = '';
                    end
                    leg_str{end+1} = 'blocked initial path segment';
                    legend(leg_str,'Location','best');
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
                    figure; hold on; box on;
                    blues = zeros(size(vgraph));
                    reds = orig_vgraph & ~reduced_vgraph;
                    greens = reduced_vgraph;
                    vgraph_image(:,:,1) = reds;
                    vgraph_image(:,:,2) = greens;
                    vgraph_image(:,:,3) = blues;
                    imshow(vgraph_image*255);
                    title(sprintf("%.2f pct. of edges removed, random edge blocking",pct_edges_removed));
                end % end flag_do_plot condition
            end % end nominal or reachable cost function loop
        end % end edge deletion portion loop
    end % end repeats loop
end % end mission loop

%% plot multiple trials data
% sort data and define plot options
markers = {'x','d','o','+','s','x','d'};
colors = {'r','b'};
idx_nominal = data(:,1)== map_idx & data(:,2)==1 & ~isnan(data(:,6)) & ~isnan(data(:,7));
nominal_data = data(idx_nominal,:);
idx_reachable = data(:,1)== map_idx & data(:,2)==2 & ~isnan(data(:,6)) & ~isnan(data(:,7));
reachable_data = data(idx_reachable,:);

% plot ratio of each path relative to its own initial path
figure; hold on; box on;
box on; hold on;
plot(nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))./nominal_data(:,5),"Color",colors{nominal_data(1,2)},"Marker",markers{nominal_data(1,1)},'LineStyle','none');
plot(reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))./reachable_data(:,5),"Color",colors{reachable_data(1,2)},"Marker",markers{reachable_data(1,1)},'LineStyle','none');
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
ylabel(sprintf('ratio of replanned path length to reachable \nor nominal initial path length'))
xlabel('percentage of visibility graph edges blocked [%]')
legend({'nominal cost function','reachable cost function'},'Location','best');

% plot ratio of each path relative to nominal initial path
figure; hold on; box on;
box on; hold on;
plot(nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))./nominal_data(1,5),"Color",colors{nominal_data(1,2)},"Marker",markers{nominal_data(1,1)},'LineStyle','none');
plot(reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))./nominal_data(1,5),"Color",colors{reachable_data(1,2)},"Marker",markers{reachable_data(1,1)},'LineStyle','none');
% add polynominal fit
p_nominal = polyfit(nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))./nominal_data(1,5),fit_order);
p_reachable = polyfit(reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))./nominal_data(1,5),fit_order);
x_for_poly = linspace(min(nominal_data(:,4)),max(nominal_data(:,4)),100);
% plot(x_for_poly,polyval(p_nominal,x_for_poly),'Color',colors{nominal_data(1,2)},'LineWidth',2);
% plot(x_for_poly,polyval(p_reachable,x_for_poly),'Color',colors{reachable_data(1,2)},'LineWidth',2);
% add convex hull
P_nominal = [nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7))./nominal_data(1,5)];
P_reachable = [reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7))./nominal_data(1,5)];
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
ylabel('ratio of replanned path length to nominal initial path length')
xlabel('percentage of visibility graph edges blocked [%]')
legend({'nominal cost function','reachable cost function'},'Location','best');

% plot absolute length
figure; hold on; box on;
box on; hold on;
plot(nominal_data(:,4),(nominal_data(:,6)+nominal_data(:,7)),"Color",colors{nominal_data(1,2)},"Marker",markers{nominal_data(1,1)},'LineStyle','none');
plot(reachable_data(:,4),(reachable_data(:,6)+reachable_data(:,7)),"Color",colors{reachable_data(1,2)},"Marker",markers{reachable_data(1,1)},'LineStyle','none');
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
xlabel('percentage of visibility graph edges blocked [%]')
ylabel('total path length after replanning [km]')
legend({'nominal cost function','reachable cost function'},'Location','best');

% plot histogram of failed trials
nandata = data(find(isnan(data(:,7))),:); % find nan replan cost rows
nandata_nominal = nandata(nandata(:,2)==1,:); % of those, find nominal ones
nandata_reachable = nandata(nandata(:,2)==2,:); % of those, find reachable ones
edge_deletion_delta = edge_deletion(2) - edge_deletion(1);
edge_deletion_bins = 100*[edge_deletion - edge_deletion_delta, edge_deletion(end)+ edge_deletion_delta];

figure; hold on; box on;
h1 = histogram(100*nandata_nominal(:,3), edge_deletion_bins); % polytope dilations for nan data
h2 = histogram(100*nandata_reachable(:,3), edge_deletion_bins); % polytope dilations for nan data
h1.FaceColor = 'r';
h2.FaceColor = 'b';
xlabel('percentage of visibility graph edges blocked [%]')
ylabel('count of failed replanning attempts');
legend({'nominal cost function','reachable cost function'},'Location','best');


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
