clear; close all; clc
% script_test_3d_polytope_canyon
% example of routing through a field of polytopes with a large chokepoint in the middle
% reachability and visibilty incentive cost functions can be used to route around the choke point

addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'

addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\Desktop\gif\gif'

addpath 'C:\Users\sjh6473\Desktop\gif\gif'
addpath 'C:\Users\sjh6473\Desktop\TriangleRayIntersection'

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;

%%%
% map_ID nominal_or_reachable edge_deletion initial_distance navigated_distance replan_route_length
%%%
data = []; % initialize array for storing results
for map_idx = 2%2:5
    if map_idx == 1
        %% load test fixtures for polytope map rather than creating it here
        % load distribution north of canyon
        load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes1.mat'));
        % this test fixture was made with the following block of code using functions from the MapGen repo
        % tiled_polytopes1 = fcn_MapGen_haltonVoronoiTiling([1,20],[2 1]);
        % % remove the edge polytope that extend past the high and low points
        % % shink the polytopes so that they are no longer tiled
        % des_radius = 0.05; % desired average maximum radius
        % sigma_radius = 0.002; % desired standard deviation in maximum radii
        % min_rad = 0.0001; % minimum possible maximum radius for any obstacle
        % [shrunk_polytopes1,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes1,des_radius,sigma_radius,min_rad);

        % load polytopes representing canyon
        load(strcat(pwd,'\..\Test_Fixtures\canyon_polys_without_exterior.mat'));
        % these polytopes were manually defined

        % load distribution south of canyon
        load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes2.mat'));
        % this test fixture was made with the following block of code using functions from the MapGen repo
        % tiled_polytopes2 = fcn_MapGen_haltonVoronoiTiling([1, 20],[2 1]);
        % % remove the edge polytope that extend past the high and low points
        % % shink the polytopes so that they are no longer tiled
        % [shrunk_polytopes2,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes2,des_radius,sigma_radius,min_rad);
        %% move second polytope field north of canyon
        second_field_vertical_translation = 1.5;
        for i = 1:length(shrunk_polytopes2)
            num_verts_this_poly = length(shrunk_polytopes2(i).yv);
            shrunk_polytopes2(i).yv = shrunk_polytopes2(i).yv + second_field_vertical_translation;
            shrunk_polytopes2(i).vertices = shrunk_polytopes2(i).vertices + [zeros(num_verts_this_poly+1,1) second_field_vertical_translation*ones(num_verts_this_poly+1,1)];
        end

        %% combine two polytope fields and canyon choke point into one field
        shrunk_polytopes = [shrunk_polytopes1, shrunk_polytopes2, polytopes_manual_canyon];
        %% define start and finish
        start_init = [0 1.25];
        finish_init = [2 1.25];
    elseif map_idx == 2
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_1.mat'));
        shrunk_polytopes = flood_plain_1;
        start_init = [-78.3 40.88];
        % finish_init = [-78.1 40.9];
        finish_init = [-78.07 40.82];
    elseif map_idx == 3
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_2.mat'));
        shrunk_polytopes = flood_plain_2;
        start_init = [-78.02 40.96];
        % finish_init = [-77.86 40.93];
        finish_init = [-77.82 40.97];
    elseif map_idx == 4
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_3.mat'));
        shrunk_polytopes = flood_plain_3;
        start_init = [-77.49 40.84];
        % finish_init = [-77.58 40.845];
        finish_init = [-77.68 40.85];
    elseif map_idx == 5
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_4.mat'));
        shrunk_polytopes = flood_plain_4;
        start_init = [-77.68 40.9];
        finish_init = [-77.5 40.8];
    end % if conditions for different map test fixtures

    %% all_pts array creation
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

    for repeat_trials = 1;
        %% plan the initial path
        start = [start_init size(all_pts,1)+1 -1 1];
        finish = [finish_init size(all_pts,1)+2 -1 1];
        finishes = [all_pts; start; finish];
        starts = [all_pts; start; finish];
        [vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);

        start_for_reachability = start;
        start_for_reachability(4) = start(3);
        finish_for_reachability = finish;
        finish_for_reachability(4) = finish(3);

        [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start_for_reachability,finish_for_reachability);
        if ~is_reachable
            error('initial mission, prior to edge deletion, is not possible')
        end

        % new experimental cost function prioritizing reachability
        reachable_nodes_from_each_node = sum(rgraph,2);
        inv_reach_cost = 100*(1./(reachable_nodes_from_each_node))';

        % new experimental cost function prioritizing visibility
        visible_nodes_from_each_node = sum(vgraph,2);
        inv_vis_cost = 100*(1./(visible_nodes_from_each_node))';

        %% make cgraph
        mode = "xy spatial only";
        % mode = 'time or z only';
        % mode = "xyz or xyt";
        [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
        hvec_with_reachability = hvec + inv_reach_cost + inv_vis_cost;

        [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
        [init_cost_with_reachability, init_route_with_reachability] = fcn_algorithm_Astar(vgraph, cgraph, hvec_with_reachability, all_pts, start, finish);

        % find route length
        route_x = init_route(:,1);
        route_y = init_route(:,2);
        lengths = diff([route_x(:) route_y(:)]);
        init_route_length = sum(sqrt(sum(lengths.*lengths,2)));

        route_x = init_route_with_reachability(:,1);
        route_y = init_route_with_reachability(:,2);
        lengths = diff([route_x(:) route_y(:)]);
        init_route_length_with_reachability = sum(sqrt(sum(lengths.*lengths,2)));

        %% find midpoint of route
        % mid_pt_x = 1;
        % mid_pt_y = interp1(route(:,1),route(:,2),mid_pt_x);
        % start_midway = [1 1.008]; % from_mid_pt_of_reachable_path
        % start_midway = [1 1.276]; % from_mid_pt_of_nominal_path
        % start_midway = [0.6 1.276]; % from_mid_pt_of_nominal_path
        % start_midway = [start_init]; % start at the original start

        navigated_distance = init_route_length/2;
        navigated_distance_with_reachability = init_route_length_with_reachability/2;

        % assume you get halfway
        St_points_input = [navigated_distance 0];
        referencePath = init_route(:,1:2);
        flag_snap_type = 1;
        start_midway = fcn_Path_convertSt2XY(referencePath,St_points_input, flag_snap_type);

        St_points_input = [navigated_distance_with_reachability 0];
        referencePath = init_route_with_reachability(:,1:2);
        start_midway_with_reachability = fcn_Path_convertSt2XY(referencePath,St_points_input, flag_snap_type);

        %% delete vgraph edges randomly
        edge_deletion = 0:0.05:0.9;
        for i = 1:13%length(edge_deletion)

            %% plan the new path
            start = [start_midway size(all_pts,1)+1 -1 1];
            start_with_reachability = [start_midway_with_reachability size(all_pts,1)+2 -1 1];
            finish = [finish_init size(all_pts,1)+3 -1 1];
            finishes = [all_pts; start; start_with_reachability; finish];
            starts = finishes;
            [vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);

            desired_portion_edge_deletion = edge_deletion(i);
            valid_edges_initially = find(vgraph==1);
            num_edges_initially = length(valid_edges_initially);
            edge_lottery_draw = rand(num_edges_initially,1);
            edges_for_removal = (edge_lottery_draw <= desired_portion_edge_deletion);
            idx_of_edges_for_removal = valid_edges_initially(edges_for_removal);
            new_vgraph = vgraph;
            new_vgraph(idx_of_edges_for_removal) = 0;
            num_edges_after = sum(sum(new_vgraph));
            pct_edges_removed = (num_edges_initially - num_edges_after)/num_edges_initially*100;

            start_for_reachability = start;
            start_for_reachability(4) = start(3);
            finish_for_reachability = finish;
            finish_for_reachability(4) = finish(3);

            [is_reachable, num_steps, rgraph] = fcn_check_reachability(new_vgraph,start_for_reachability,finish_for_reachability);

            start_for_reachability = start_with_reachability;
            start_for_reachability(4) = start(3);
            finish_for_reachability = finish;
            finish_for_reachability(4) = finish(3);

            [is_reachable_with_reachability, num_steps, rgraph] = fcn_check_reachability(new_vgraph,start_for_reachability,finish_for_reachability);

            if ~is_reachable
                warning('mission replanning is impossible in nominal case')
                replan_cost = NaN;
                replan_route_length = NaN;
                nominal_or_reachable = 1;
                data = [data; map_idx nominal_or_reachable edge_deletion(i) pct_edges_removed init_route_length navigated_distance replan_route_length];
            end
            if ~is_reachable_with_reachability
                warning('mission replanning is impossible in reachability case')
                replan_cost_with_reachability = NaN;
                replan_route_length_with_reachability = NaN;
                nominal_or_reachable = 2;
                data = [data; map_idx nominal_or_reachable edge_deletion(i) pct_edges_removed init_route_length_with_reachability navigated_distance_with_reachability replan_route_length_with_reachability];
            end

            % new experimental cost function prioritizing reachability
            reachable_nodes_from_each_node = sum(rgraph,2);
            inv_reach_cost = 100*(1./(reachable_nodes_from_each_node))';

            % new experimental cost function prioritizing visibility
            visible_nodes_from_each_node = sum(new_vgraph,2);
            inv_vis_cost = 100*(1./(visible_nodes_from_each_node))';

            %% make cgraph
            mode = "xy spatial only";
            % mode = 'time or z only';
            % mode = "xyz or xyt";

            if is_reachable
                [cgraph, hvec] = fcn_algorithm_generate_cost_graph([all_pts; start_with_reachability], start, finish, mode);
                [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, [all_pts; start_with_reachability], start, finish);

                % find route length
                route_x = replan_route(:,1);
                route_y = replan_route(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));
                nominal_or_reachable = 1;
                % map_ID nominal_or_reachable edge_deletion initial_distance navigated_distance replan_route_length
                data = [data; map_idx nominal_or_reachable edge_deletion(i) pct_edges_removed init_route_length navigated_distance replan_route_length];
                % plot field, initial path, replan path, and midway point
                if flag_do_plot
                    figure; hold on; box on;
                    xlabel('x [km]');
                    ylabel('y [km]');
                    plot(start(1),start(2),'xg','MarkerSize',3);
                    plot(finish(1),finish(2),'xr');
                    plot(init_route(:,1),init_route(:,2),'k','LineWidth',2);
                    plot(start_midway(1),start_midway(2),'dm','MarkerSize',3)
                    plot(replan_route(:,1),replan_route(:,2),'--g','LineWidth',2);
                    for j = 1:length(shrunk_polytopes)
                         fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
                    end
                    title_string = sprintf('map idx: %i, nominal or reachable: %i, pct edges removed: %.1f',map_idx, nominal_or_reachable,pct_edges_removed);
                    title(title_string);
                    legend('start','finish','initial route','replanning point','replanned route','obstacles');
                end % end flag_do_plot
            end % is_reachable for nominal case
            if is_reachable_with_reachability
                [cgraph, hvec] = fcn_algorithm_generate_cost_graph([all_pts; start], start_with_reachability, finish, mode);
                hvec_with_reachability = hvec;
                [replan_cost_with_reachability, replan_route_with_reachability] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec_with_reachability, [all_pts;start], start_with_reachability, finish);

                % find route length
                route_x = replan_route_with_reachability(:,1);
                route_y = replan_route_with_reachability(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                replan_route_length_with_reachability = sum(sqrt(sum(lengths.*lengths,2)));
                nominal_or_reachable = 2;
                % map_ID nominal_or_reachable edge_deletion initial_distance navigated_distance replan_route_length
                data = [data; map_idx nominal_or_reachable edge_deletion(i) pct_edges_removed init_route_length_with_reachability navigated_distance_with_reachability replan_route_length_with_reachability];
                % plot field, initial path, replan path, and midway point
                if flag_do_plot
                    figure; hold on; box on;
                    xlabel('x [km]');
                    ylabel('y [km]');
                    plot(start(1),start(2),'xg','MarkerSize',3);
                    plot(finish(1),finish(2),'xr');
                    plot(init_route_with_reachability(:,1),init_route_with_reachability(:,2),'k','LineWidth',2);
                    plot(start_midway_with_reachability(1),start_midway_with_reachability(2),'dm','MarkerSize',3)
                    plot(replan_route_with_reachability(:,1),replan_route_with_reachability(:,2),'--g','LineWidth',2);
                    for j = 1:length(shrunk_polytopes)
                         fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
                    end
                    title_string = sprintf('map idx: %i, nominal or reachable: %i, pct edges removed: %.1f',map_idx, nominal_or_reachable,pct_edges_removed);
                    title(title_string);
                    legend('start','finish','initial route','replanning point','replanned route','obstacles');
                end % end flag_do_plot
            end % is reachable for replan case
        end % end edge deletion portion loop
    end % end repeat attempts
end % end map loop

figure; hold on; box on;
box on; hold on;
markers = {'x','d','o','+','s'};
colors = {'r','b'};
for data_idx = 1:size(data,1)
    datum = data(data_idx,:);
    if isnan(datum(7))
        continue
    end
    if datum(1) == 5
        plot(datum(4),(datum(6)+datum(7))/datum(5),"Color",colors{datum(2)},"Marker",markers{datum(1)});
    end
end
ylabel('ratio of replanned path length to initial path length')
xlabel('percentage of visibility graph edges blocked')

figure; hold on; box on;
box on; hold on;
for data_idx = 1:size(data,1)
    datum = data(data_idx,:);
    if isnan(datum(7))
        continue
    end
    if datum(1) == 5
        plot(datum(4),(datum(6)+datum(7)),"Color",colors{datum(2)},"Marker",markers{datum(1)});
    end
end
ylabel('total path length after replanning')
xlabel('percentage of visibility graph edges blocked')

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