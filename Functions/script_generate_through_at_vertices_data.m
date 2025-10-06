% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- fixed calls to fcn_MapGen_polytopesStatistics, replaced with fcn_MapGen_statsPolytopes


% clear
% clc
% close all
% 
% %% add necessary directories
% addpath([pwd '\Example_Map_Generation_Code'])
% addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
% addpath([pwd '\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

%% initialize loop params and storage arrays for plotting
des_gap_size = linspace(0.0001,0.0125,30);
% colors = [1 0 0; 0 1 0; 0 0 1; 0 1 1;1 0 1;1 1 0; 0 0 0;0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250; 0.4940 0.1840 0.5560; 0.4660 0.6740 0.1880; 0.3010 0.7450 0.9330; 0.6350 0.0780 0.1840; 1 0 0; 0 1 0; 0 0 1; 0 1 1;1 0 1;1 1 0];
des_costs = [0.5];%,0.15,0.2,0.25,0];
all_rd = [];
all_rlc = [];
all_obs_through = [];
all_obs_around = [];
fig = 77777777;
figure(fig);
box on;
ylabel('Length Cost Ratio [r_{LC}]');
xlabel('Departure Ratio [r_D]');

flag_do_plot = 1;

%% begin loop of departure ratios
for Halton_seed = 1:20:101
    % generate Voronoi tiling from Halton points
    low_pt = 1+Halton_seed; high_pt = 1000+Halton_seed; % range of Halton points to use to generate the tiling
    trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
    field_stats_pre_shrink = fcn_MapGen_statsPolytopes(trim_polytopes);
    for gap_idx = 1:length(des_gap_size)

        
        % shink the polytopes so that they are no longer tiled
        gap_size = des_gap_size(gap_idx); % desired average maximum radius
        shrunk_polytopes = fcn_MapGen_polytopesShrinkFromEdges(trim_polytopes,gap_size);

        %% polytope stats to create inputs for predictor code
        field_stats = fcn_MapGen_statsPolytopes(shrunk_polytopes);
        
        % extract parameters of interest
        field_avg_r_D = field_stats.avg_r_D;
        all_rd = [all_rd, field_avg_r_D];
        field_avg_r_D_pre_shrink = field_stats_pre_shrink.avg_r_D;


        %% get some basic parameters for plotting
        xlow = 0; xhigh = 1; ylow = 0; yhigh = 1;
        area = (xhigh-xlow)*(yhigh-ylow);
        rho = length(trim_polytopes)/area;


        %% begin loop of costs
        for cost_idx=1%:length(des_costs)
            des_cost = des_costs(cost_idx);            
            shrunk_polytopes = fcn_MapGen_polytopesSetCosts(shrunk_polytopes, des_cost, (-1));


            %% plan path
            % starting (A) and finish (B) coordinates
            A.x = 0; A.y = 0.5; B.x = 1; B.y = 0.5;

            [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B,'through at vertices');

            x = path(:,1);
            y = path(:,2);
            d = diff([x(:) y(:)]);
            total_length = sum(sqrt(sum(d.*d,2)));
            all_rlc = [all_rlc, cost];
            % num_predicted_obs_traversals_this_rd = [num_predicted_obs_traversals_this_rd, num_predicted_obs_traversals];
            [obs_around, obs_through] =...
                fcn_general_calculation_count_obs_in_path(path);
            all_obs_through = [all_obs_through, obs_through];
            all_obs_around = [all_obs_around, obs_around];

        end

    end
end
for i = 1:length(all_rd)
    figure(fig)
    clf
    hold on
    plot(all_rd(i),all_rlc(i),'Marker','d','Color',[all_obs_around(i)/(all_obs_around(i)+all_obs_through(i)),0,all_obs_through(i)/(all_obs_around(i)+all_obs_through(i))]);
end
