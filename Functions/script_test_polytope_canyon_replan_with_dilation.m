% script_test__polytope_canyon_replan_with_dilation
% This script is very similar to the scrip_test_polytope_canyon_replan but it evaluates a corridor width cost function instead of
% a connectivity cost function. This script plans two paths: (1) a distance reducing path and (2) a
% path that routes down edges with more free space around them. Then obstacles are dilated to close
% off narrow corridors and replanning is triggered from midway down the initial path. The final paths
% are then compared to show that replanning from the path that uses wider corridors is less costly and
% more likely to be successful.

clear; close all; clc
addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% plotting flags
flag_do_plot = 1;
flag_do_plot_slow= 0;

%% mission options
map_idx =5;
navigated_portion = 0.4; % portion of initial path to be completed prior to triggering replanning
w = 1/6; % relative weighting of cost function, cost = w*length_cost + (1-w)*dilation_robustness_cost
[shrunk_polytopes, start_inits, finish_inits] = fcn_util_load_test_map(map_idx);

% map_idx nominal_or_width_based polytope_size_increases polytope_size_increases init_route_length navigated_distance replan_route_length
data = []; % initialize array for storing results
for mission_idx = 1:size(start_inits,1)
    start_init = start_inits(mission_idx,:);
    finish_init = finish_inits(mission_idx,:);

    % loop over dilation sizes
    for polytope_size_increases = [0.01 0.02 0.05 0.1 0.2 0.3 0.4 0.5]
        % loop over the nominal cost function and feature cost function
        for nominal_or_width_based = [1,2]
            %% plan the initial path
            % all_pts array creation
            [all_pts, start, finish] = fcn_polytopes_generate_all_pts_table(shrunk_polytopes, start_init, finish_init);
            % find vgraph
            finishes = [all_pts; start; finish];
            starts = [all_pts; start; finish];
            [vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);
            orig_vgraph = vgraph; % note the original to compare it to the reduced vgraph

            % find rgraph
            [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
            if ~is_reachable
                error('initial mission, prior to edge deletion, is not possible')
            end

            % make cgraph
            mode = "xy spatial only";
            [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);

            % make dilation robustness matrix
            mode = '2d';
            dilation_robustness_tensor = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode, shrunk_polytopes);
            dilation_robustness_matrix = max(dilation_robustness_tensor(:,:,1) , dilation_robustness_tensor(:,:,2)); % combine the left and right sides as a max
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

            % find initial route length
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
            [all_pts_new, start, finish] = fcn_polytopes_generate_all_pts_table(enlarged_polytopes, start_midway, finish_init);

            % make vgraph again
            finishes = [all_pts_new; start; finish];
            starts = [all_pts_new; start; finish];
            [new_vgraph, visibility_results_all_pts_new] = fcn_visibility_clear_and_blocked_points_global(enlarged_polytopes, starts, finishes,1);
            reduced_vgraph = new_vgraph;
            % get vgraph stats
            num_edges_initially = sum(sum(vgraph));
            num_edges_finally = sum(sum(new_vgraph));
            num_edges_removed = num_edges_initially - num_edges_finally;
            pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
            % make rgraph again
            [is_reachable, num_steps, rgraph] = fcn_check_reachability(new_vgraph,start(3),finish(3));
            if ~is_reachable
                % we don't want to break if replanning is impossible, we want to save the data for what caused this
                warning('mission replanning is impossible')
                replan_cost = NaN;
                replan_route_length = NaN;
                data = [data; map_idx nominal_or_width_based polytope_size_increases polytope_size_increases init_route_length navigated_distance replan_route_length];
                continue
            end % end is_reachable condition for replanning

            % make cgraph again
            mode = "xy spatial only";
            [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts_new, start, finish, mode);
            % replan path
            [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts_new, start, finish);

            % find replan route length
            route_x = replan_route(:,1);
            route_y = replan_route(:,2);
            lengths = diff([route_x(:) route_y(:)]);
            replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));

            % save data from this trial
            data = [data; map_idx nominal_or_width_based polytope_size_increases polytope_size_increases init_route_length navigated_distance replan_route_length];
            %% plot single trial
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
                for j = 1:length(enlarged_polytopes)
                     fill(enlarged_polytopes(j).vertices(:,1)',enlarged_polytopes(j).vertices(:,2),[0 0 1],'FaceColor','r','FaceAlpha',0.3)
                end
                for j = 1:length(shrunk_polytopes)
                     fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
                end
                title_string = sprintf('map idx: %i, nominal or corridor-width-based: %i,\npolytope size increase [km]: %.2f',map_idx, nominal_or_width_based,polytope_size_increases);
                title(title_string);
                leg_str = {'start','finish','initial route','replanning point','replanned route','enlarged obstacles'};
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
ylabel(sprintf('ratio of replanned path length to width incentive \nor nominal initial path length'))
xlabel('obstacle size increase [km]')
legend({'nominal cost function','corridor width function'},'Location','best');

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
xlabel('obstacle size increase [km]')
legend({'nominal cost function','corridor width cost function'},'Location','best');

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
xlabel('obstacle size increase [km]')
ylabel('total path length after replanning [km]')
legend({'nominal cost function','corridor width function'},'Location','best');

% plot histogram of failed trials
nandata = data(find(isnan(data(:,7))),:); % find nan replan cost rows
nandata_nominal = nandata(nandata(:,2)==1,:); % of those, find nominal ones
nandata_reachable = nandata(nandata(:,2)==2,:); % of those, find reachable ones
polytope_size_bins = [0.005 0.015 0.03 0.075 0.15 0.25 0.35 0.45 0.55];

figure; hold on; box on;
h1 = histogram(nandata_nominal(:,4), polytope_size_bins); % polytope dilations for nan data
h2 = histogram(nandata_reachable(:,4), polytope_size_bins); % polytope dilations for nan data
h1.FaceColor = 'r';
h2.FaceColor = 'b';
xlabel('obstacle size increase [km]')
ylabel('count of failed replanning attempts');
legend({'nominal cost function','corridor width function'},'Location','best');

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
