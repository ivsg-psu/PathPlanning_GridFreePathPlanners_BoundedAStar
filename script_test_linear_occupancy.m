clear
clc
close all

%% add necessary directories
addpath([pwd '\Example_Map_Generation_Code'])
addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

%% initialize loop params and storage arrays for plotting
des_gap_size = linspace(0.0001,0.08,30);%linspace(0.001,0.081,10);
all_rd = [];

flag_do_plot = 0;

measured_unoccupancy = [];
% generate Voronoi tiling from Halton points
low_pt = 1; high_pt = 1000; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);

%% begin loop of departure ratios
for gap_idx = 1:length(des_gap_size)
    % shink the polytopes so that they are no longer tiled
    gap_size = des_gap_size(gap_idx); % desired average maximum radius
    shrunk_polytopes = fcn_MapGen_polytopesShrinkFromEdges(trim_polytopes,gap_size);

    %% polytope stats to create inputs for predictor code
    field_stats = fcn_MapGen_polytopesStatistics(shrunk_polytopes);
    field_stats_pre_shrink = fcn_MapGen_polytopesStatistics(trim_polytopes);
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
    all_rd = [all_rd, field_avg_r_D];

    %% initialize loop params and storage arrays for plotting
    % plot the map
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    % fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);


    %% plan path
    % starting (A) and finish (B) coordinates
    A.x = 0; A.y = 0.5; B.x = 1; B.y = 0.5;
    % TODO fix this to not be the straight cost but just the planned cost
    [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B);
    measured_unoccupancy = [measured_unoccupancy, cost];
end

figure(74)
hold on
plot(all_rd,measured_unoccupancy)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear unoccupancy ratio');
