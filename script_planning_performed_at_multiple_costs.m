clear
clc
close all

%% add necessary directories
addpath([pwd '\Example_Map_Generation_Code'])
addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])

%% initialize loop params and storage arrays for plotting
des_radii = linspace(0.001,0.081,20);
colors = [1 0 0; 0 1 0; 0 0 1; 0 1 1;1 0 1;1 1 0; 0 0 0;0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250; 0.4940 0.1840 0.5560; 0.4660 0.6740 0.1880; 0.3010 0.7450 0.9330; 0.6350 0.0780 0.1840; 1 0 0; 0 1 0; 0 0 1; 0 1 1;1 0 1;1 1 0];
all_rd = [];
fig = 99;

%% begin loop of departure ratios
for radii_idx = 1:length(des_radii)
    % generate Voronoi tiling from Halton points
    low_pt = 1; high_pt = 100; % range of Halton points to use to generate the tiling
    tiled_polytopes = fcn_polytope_generation_halton_voronoi_tiling(low_pt,high_pt);
    % remove the edge polytope that extend past the high and low points
    xlow = 0; xhigh = 1; ylow = 0; yhigh = 1;
    trim_polytopes = fcn_polytope_editing_remove_edge_polytopes(tiled_polytopes,xlow,xhigh,ylow,yhigh);
    % shink the polytopes so that they are no longer tiled
    des_radius = des_radii(radii_idx); % desired average maximum radius
    sigma_radius = 0.002; % desired standard deviation in maximum radii
    min_rad = 0.0001; % minimum possible maximum radius for any obstacle
    shrink_seed = 1111; % seed used for randomizing the shrinking process
    rng(shrink_seed) % set the random number generator with the shrink seed
    try % if shrink isn't possible, just skip this r_D
        shrunk_polytopes = fcn_polytope_editing_shrink_to_average_max_radius_with_variance(trim_polytopes,des_radius,sigma_radius,min_rad);
    catch
        continue
    end

    %% polytope stats to create inputs for predictor code
    field_stats = fcn_MapGen_polytopesStatistics(shrunk_polytopes);
    field_stats_pre_shrink = fcn_MapGen_polytopesStatistics(trim_polytopes);
    % extract parameters of interest
    gap_size = field_stats.average_gap_size_G_bar;
    field_avg_r_D = field_stats.avg_r_D;
    field_avg_r_D_pre_shrink = field_stats_pre_shrink.avg_r_D;
    shrunk_distance = field_stats_pre_shrink.average_max_radius - field_stats.average_max_radius;
    shrink_ang = field_stats_pre_shrink.average_vertex_angle;
    R_bar_initial = field_stats_pre_shrink.average_max_radius;

    %% get some basic parameters for plotting
    area = (xhigh-xlow)*(yhigh-ylow);
    rho = length(trim_polytopes)/area;
    rd = des_radius*sqrt(rho);
    all_rd = [all_rd, rd];

    %% initialize loop params and storage arrays for plotting
    des_costs = linspace(0,0.5,13);
    total_lengths = [];
    obs_around_all = [];
    obs_through_all = [];
    predicted_obs_through_all = [];
    r_lc_sparse_average_actuals = [];
    num_predicted_obs_traversals_this_rd = [];
    predicted_N_int = [];
    actual_N_int = [];

    %% begin loop of costs
    for cost_idx=1:length(des_costs)
        des_cost = des_costs(cost_idx);
        predicted_N_int = [predicted_N_int, field_stats.linear_density_mean];
        shrunk_polytopes = fcn_polytope_editing_set_all_costs(shrunk_polytopes,des_cost);

        % run predictor, could run outside of cost loop except now depends on costs
        [field_small_choice_angles,field_big_choice_angles,r_lc_max,r_lc_avg,r_lc_iterative,r_lc_max_effective,r_lc_avg_effective,r_lc_iterative_effective,r_lc_sparse_worst,r_lc_sparse_average,r_lc_sparse_std,r_lc_sparse_worst_new,r_lc_sparse_average_new,r_lc_sparse_std_new,r_lc_sparse_worst_actual,r_lc_sparse_average_actual,r_lc_sparse_std_actual,r_lc_sparse_worst_linear,r_lc_sparse_average_linear,r_lc_sparse_std_linear,num_predicted_obs_traversals] = ...
        fcn_MapGen_polytopesPredictLengthCostRatio(shrunk_polytopes,gap_size,...
            shrunk_distance,shrink_ang,R_bar_initial);

        % plot the map
        line_spec = 'b-'; % edge line plotting
        line_width = 2; % linewidth of the edge
        axes_limits = [0 1 0 1]; % x and y axes limits
        axis_style = 'square'; % plot axes style
        fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
        % fig = fig + 1;


        %% plan path
        % starting (A) and finish (B) coordinates
        A.x = 0; A.y = 0.5; B.x = 1; B.y = 0.5;

        [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B);
        % path: series of points [x y point_id obs_id beg_end]
        % cost: path length
        % err: marker indicating if there was an error in setup (1) or not (0)

        % critical cost is based on when side length is longer than scaled dist
        cost_c = 1./cos(field_small_choice_angles') - 1;
        % current polys below crit cost will be traversed
        cost_logical = cost_c > des_cost;
        % count number of polys where cost was less than crit cost
        predicted_vert_through = sum(cost_logical);
        % TODO(@sjharnett) is this the right thing to normalize by?
        portion_vert_through = predicted_vert_through/length(field_small_choice_angles');
        % if the cost is 0, we should assume that we pass through everything
        if des_cost == 0
%             assert(portion_vert_through == 1);
        end
        % assume the percent of obstacles we pass through is the same as the percent of vertices...
        % where we would pass through, applied to the number of obstacles blocking the path
        predicted_obs_through = field_stats.linear_density_mean*portion_vert_through;
        predicted_obs_through_all = [predicted_obs_through_all, predicted_obs_through];
        % plot path
        plot(path(:,1),path(:,2),'k-','linewidth',2)
        plot(A.x, A.y, 'gx','linewidth',2)
        plot(B.x, B.y, 'rx','linewidth',2)
        x = path(:,1);
        y = path(:,2);
        d = diff([x(:) y(:)]);
        total_length = sum(sqrt(sum(d.*d,2)));
        total_lengths = [total_lengths, total_length];
        r_lc_sparse_average_actuals = [r_lc_sparse_average_actuals, r_lc_sparse_average_actual];
        num_predicted_obs_traversals_this_rd = [num_predicted_obs_traversals_this_rd, num_predicted_obs_traversals];

        %% info needed for further work
        % gather data on all the points
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

        appex_x = zeros(size(path,1)-2,3);
        appex_y = appex_x;
        prev_pt = [A.x A.y];
        for app = 1:size(appex_x,1)
            pt = path(app+1,:);
            appex_x(app,1) = pt(1);
            appex_y(app,1) = pt(2);
            if pt(5) == 1
                obstacle = pt(4);
                other_beg_end_pt = all_pts(((all_pts(:,4)==obstacle).*(all_pts(:,5)==1).*(all_pts(:,3)~=pt(3)))==1,:);
                if other_beg_end_pt(3) > pt(3)
                    other_pt = all_pts(pt(3)+1,1:2);
                else % pt(3) > other_beg_end_pt(3)
                    other_pt = all_pts(pt(3)-1,1:2);
                end
                dist = fcn_general_calculation_euclidean_point_to_point_distance(ones(2,1)*prev_pt,[other_beg_end_pt(1:2); other_pt]);
                if dist(1) < dist(2) % other_pt farther
                    appex_x(app,2:3) = [other_beg_end_pt(1), other_pt(1)];
                    appex_y(app,2:3) = [other_beg_end_pt(2), other_pt(2)];
                else % other_pt closer
                    appex_x(app,2:3) = [other_pt(1), other_beg_end_pt(1)];
                    appex_y(app,2:3) = [other_pt(2), other_beg_end_pt(2)];
                end
            else
                pt1 = all_pts(pt(3)-1,1:2);
                pt2 = all_pts(pt(3)+1,1:2);
                dist = fcn_general_calculation_euclidean_point_to_point_distance(ones(2,1)*prev_pt,[pt1; pt2]);
                if dist(1) < dist(2) % pt1 closer
                    appex_x(app,2:3) = [pt1(1), pt2(1)];
                    appex_y(app,2:3) = [pt1(2), pt2(2)];
                else % pt1 farther
                    appex_x(app,2:3) = [pt2(1), pt1(1)];
                    appex_y(app,2:3) = [pt2(2), pt1(2)];
                end
            end
            prev_pt = pt(1:2);
        end
        [obs_around, obs_through] =...
            fcn_general_calculation_count_obs_in_path(path);
        obs_around_all = [obs_around_all, obs_around];
        obs_through_all = [obs_through_all, obs_through];
        figure(fig)
        hold on
        plot(appex_x,appex_y,'o','linewidth',2)
        box on;
        xlabel('x [km]');
        ylabel('y [km]');
        title_string = sprintf('Dep. rat %.2f, obs. cost %.2f, path length %.3f, \n obs. around %.0f, obs. through %.0f',...
            rd, des_cost, total_length, obs_around, obs_through);
        title(title_string);
        xlim([0.02,0.27])
        ylim([0.5-0.25/2,0.625])
        % appex_x = [appex_x1 closer_x1 farther_x1; appex_x2 closer_x2 farther_x2; .... appex_xn closer_xn farther_xn]
        % appex_y = [appex_y1 closer_y1 farther_y1; appex_y2 closer_y2 farther_y2; .... appex_yn closer_yn farther_yn]

        %% Final Info
        % A, B, appex_x, appex_y
    end
    figure(747474)
    hold on
    plot(des_costs,total_lengths,'Color', colors(radii_idx,1:3));
    plot(des_costs, r_lc_sparse_average_actuals,'x','Color', colors(radii_idx,1:3));
    figure(747475)
    hold on
    plot(des_costs,obs_through_all./(obs_through_all+obs_around_all), 'Color',colors(radii_idx,1:3))
    plot(des_costs,num_predicted_obs_traversals_this_rd./length(field_small_choice_angles'),'x', 'Color',colors(radii_idx,1:3))
    figure(77577)
    hold on
    plot(des_costs,(obs_through_all+obs_around_all),'Color',colors(radii_idx,1:3));
    plot(des_costs,predicted_N_int,'x','Color', colors(radii_idx,1:3));
end
    figure(747474)
    hold on
    legends = {};
    k = 1;
    for i = 1:length(all_rd)
        legends(k) = {sprintf('departure ratio = %.3f',all_rd(i))};
        k = k + 1;
        legends(k) = {sprintf('predicted for departure ratio = %.3f',all_rd(i))};
        k = k + 1;
    end
    legend(legends);
    box on
    xlabel('Obstacle cost')
    ylabel('Total planned path length [km]')
    figure(747475)
    hold on
    box on
    legend(legends);
    xlabel('Obstacle cost')
    ylabel('Ratio of obs. traversed to total obs.')
    figure(77577)
    hold on
    box on
    xlabel('Obstacle cost')
    ylabel('Number of encountered obstacles, N_{int}')
    legend(legends);
