% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- fixed calls to fcn_MapGen_polytopesStatistics, replaced with fcn_MapGen_statsPolytopes


% clear
% clc
% close all
%
% add necessary directories
% addpath([pwd '\Example_Map_Generation_Code'])
% addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
% addpath([pwd '\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

%% initialize loop params and storage arrays for plotting
des_gap_size = linspace(0.0001,0.08,20);
colors = [1 0 0; 0 1 0; 0 0 1; 0 1 1;1 0 1;1 1 0; 0 0 0;0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250; 0.4940 0.1840 0.5560; 0.4660 0.6740 0.1880; 0.3010 0.7450 0.9330; 0.6350 0.0780 0.1840; 1 0 0; 0 1 0; 0 0 1; 0 1 1;1 0 1;1 1 0];
all_rd = [];
fig = 99;

flag_do_plot = 1;

predicted_straight_path_costs = [];
straight_path_costs = [];

%% begin loop of departure ratios
for Halton_seed = 1%:20:101
    for gap_idx = 5%1:length(des_gap_size)
        try
            % generate Voronoi tiling from Halton points
            low_pt = 1+Halton_seed; high_pt = 4000+Halton_seed; % range of Halton points to use to generate the tiling
            trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
            % shink the polytopes so that they are no longer tiled
            gap_size = des_gap_size(gap_idx); % desired average maximum radius
            shrunk_polytopes = fcn_MapGen_polytopesShrinkFromEdges(trim_polytopes,gap_size);

            %% polytope stats to create inputs for predictor code
            field_stats = fcn_MapGen_statsPolytopes(shrunk_polytopes);
            field_stats_pre_shrink = fcn_MapGen_statsPolytopes(trim_polytopes);
            % extract parameters of interest
            field_avg_r_D = field_stats.avg_r_D;
            field_avg_r_D_pre_shrink = field_stats_pre_shrink.avg_r_D;
            shrunk_distance = field_stats_pre_shrink.average_max_radius - field_stats.average_max_radius;
            shrink_ang = field_stats_pre_shrink.average_vertex_angle;
            R_bar_initial = field_stats_pre_shrink.average_max_radius;

            %% get some basic parameters for plotting
            xlow = 0; xhigh = 1; ylow = 0; yhigh = 1;
            area = (xhigh-xlow)*(yhigh-ylow);
            rho = length(trim_polytopes)/area;

            %% initialize loop params and storage arrays for plotting
            des_costs = linspace(0,0.2*pi,9);
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
                shrunk_polytopes = fcn_MapGen_polytopesSetCosts(shrunk_polytopes, des_cost, (-1));


                if flag_do_plot
                    % plot the map
                    line_spec = 'b-'; % edge line plotting
                    line_width = 2; % linewidth of the edge
                    axes_limits = [0 1 0 1]; % x and y axes limits
                    axis_style = 'square'; % plot axes style
                    fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
                end


                %% plan path
                % starting (A) and finish (B) coordinates
                A.x = 0; A.y = 0.5; B.x = 1; B.y = 0.5;

                [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B,'through or around');
                % array is gap_idx, cost_idx, r_lc_straight_through_predicted
                straight_path_costs = [straight_path_costs; Halton_seed, gap_idx, cost_idx, field_avg_r_D, cost];
                % predict straight path cost
                r_lc_straight_through = ...
                    fcn_MapGen_polytopesPredictLengthCostRatioStraightPath(trim_polytopes,shrunk_polytopes,gap_size,A.x,A.y,B.x,B.y);
                % array is gap_idx, cost_idx, r_lc_straight_through_predicted
                predicted_straight_path_costs = [predicted_straight_path_costs; Halton_seed, gap_idx, cost_idx, field_avg_r_D, r_lc_straight_through];

                figure(fig)
                hold on
                x = path(:,1);
                y = path(:,2);
                d = diff([x(:) y(:)]);
                total_length = sum(sqrt(sum(d.*d,2)));
                total_lengths = [total_lengths, total_length];
                % num_predicted_obs_traversals_this_rd = [num_predicted_obs_traversals_this_rd, num_predicted_obs_traversals];
                [obs_around, obs_through] =...
                    fcn_BoundedAStar_countObstaclesInPath(path);
                obs_around_all = [obs_around_all, obs_around];
                obs_through_all = [obs_through_all, obs_through];

            end
            figure(747474)
            hold on
%             plot(des_costs,total_lengths,'Color', colors(gap_idx,1:3));
%             plot(des_costs, r_lc_sparse_average_actuals,'x','Color', colors(gap_idx,1:3));
            figure(747475)
            hold on
%             plot(des_costs,obs_through_all./(obs_through_all+obs_around_all), 'Color',colors(gap_idx,1:3))
            % plot(des_costs,num_predicted_obs_traversals_this_rd./length(field_small_choice_angles'),'x', 'Color',colors(gap_idx,1:3))
            figure(77577)
            hold on
%             plot(des_costs,(obs_through_all+obs_around_all),'Color',colors(gap_idx,1:3));
%             plot(des_costs,predicted_N_int,'x','Color', colors(gap_idx,1:3));
            close all;
        end
    end
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
figure(77588)
clf
hold on
% for point_idx = 1:1:size(straight_path_costs,1)
%     plot(all_rd(straight_path_costs(point_idx,1)),straight_path_costs(point_idx,3),'o','Color',colors(straight_path_costs(point_idx,2),1:3));
%     hold on;
%     plot(all_rd(predicted_straight_path_costs(point_idx,1)),predicted_straight_path_costs(point_idx,3),'x','Color',colors(predicted_straight_path_costs(point_idx,2),1:3));
% end
hold on
box on
xlabel('Departure ratio [r_D]')
ylabel('Length Cost Ratio for Routing Straight Through [r_{LC}]')
legends = {};
k = 1;
% for i = 1:length(des_costs)
%     legends(k) = {sprintf('cost = %.3f',des_costs(i))};
%     k = k + 1;
%     legends(k) = {sprintf('predicted for cost = %.3f',des_costs(i))};
%     k = k + 1;
% end
legend(legends);

save('C:\Users\sjhar\OneDrive\Desktop\sive_iters_per_point')

% figure(8787)
% clf
% hold on
% for point_idx = 1:1:size(straight_path_costs,1)
%     plot(all_rd(straight_path_costs(point_idx,1)),straight_path_costs(point_idx,3),'o','Color',colors(straight_path_costs(point_idx,2),1:3));
%     hold on;
%     plot(all_rd(predicted_straight_path_costs(point_idx,1)),predicted_straight_path_costs(point_idx,3),'x','Color',colors(predicted_straight_path_costs(point_idx,2),1:3));
% end
% hold on
% box on
% xlabel('Departure ratio [r_D]')
% ylabel('Critical Cost [traversal cost as % of free space cost]')
% legend('predicted','actual')
%
figure(86868686)
clf
hold on
box on
xlabel('Departure ratio [r_D]')
ylabel('Length Cost Ratio for Routing Straight Through [r_{LC}]')

% sort dataframes by traversal costs
straight_path_costs_by_poly_cost = sortrows(straight_path_costs,3);
predicted_straight_path_costs_by_poly_cost = sortrows(predicted_straight_path_costs,3);

for i = 1:120:961
    data_this_poly_cost = straight_path_costs_by_poly_cost(i:i+119,:);
    predicted_data_this_poly_cost = predicted_straight_path_costs_by_poly_cost(i:i+119,:);
    plot(data_this_poly_cost(:,4),data_this_poly_cost(:,5),'o','Color',colors(data_this_poly_cost(1,3),:));
    plot(predicted_data_this_poly_cost(:,4),predicted_data_this_poly_cost(:,5),'x','Color',colors(predicted_data_this_poly_cost(1,3),:));
end

legends = {};
k = 1;
for i = 1:length(des_costs)
    legends(k) = {sprintf('cost = %.3f',des_costs(i))};
    k = k + 1;
    legends(k) = {sprintf('predicted for cost = %.3f',des_costs(i))};
    k = k + 1;
end

legend(legends);

figure(86868687)
clf
hold on
box on
xlabel('Departure ratio [r_D]')
ylabel('Length Cost Ratio for Routing Straight Through [r_{LC}]')

% sort dataframes by traversal costs
straight_path_costs_by_poly_cost = sortrows(straight_path_costs,3);
predicted_straight_path_costs_by_poly_cost = sortrows(predicted_straight_path_costs,3);

for i = 1:120:961
    data_this_poly_cost = straight_path_costs_by_poly_cost(i:i+119,:);
    predicted_data_this_poly_cost = predicted_straight_path_costs_by_poly_cost(i:i+119,:);
    plot(data_this_poly_cost(:,4),data_this_poly_cost(:,5),'o','Color',colors(data_this_poly_cost(1,3),:));
    % plot a poly fit to the prediction data instead of a scatter
    coefficients = polyfit(predicted_data_this_poly_cost(:,4), predicted_data_this_poly_cost(:,5), 2);
    xFit = linspace(min(predicted_data_this_poly_cost(:,4)), max(predicted_data_this_poly_cost(:,4)), 1000);
    yFit = polyval(coefficients , xFit);
    plot(xFit,yFit,'Color',colors(predicted_data_this_poly_cost(1,3),:));
end

legend(legends);
