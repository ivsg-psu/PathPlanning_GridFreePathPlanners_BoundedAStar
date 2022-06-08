% clear
% clc
% close all

%% add necessary directories
addpath([pwd '\Example_Map_Generation_Code'])
addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

%% initialize loop params and storage arrays for plotting
% des_gap_size = linspace(0.0001,0.08,30);%linspace(0.001,0.081,10);
% all_rd = [];

% flag_do_plot = 0;

% measured_unoccupancy = [];
est_from_sqrt_area_ratio_all = [];
est_from_gap_size_all = [];
% est_from_AABB_all = [];
% est_from_slant_AABB_all = [];
% est_from_gap_size_normal_all = [];
% est_from_poly_fit_all = [];

% generate Voronoi tiling from Halton points
% low_pt = 1; high_pt = 1000; % range of Halton points to use to generate the tiling
% trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);

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
%     all_rd = [all_rd, field_avg_r_D];

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
%     [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B);
%     measured_unoccupancy = [measured_unoccupancy, cost];

    %% find estimated values
    unocc_ests = fcn_MapGen_polytopesPredictUnoccupancyRatio(trim_polytopes,shrunk_polytopes,gap_size);
    est_from_sqrt_area_ratio_all = [est_from_sqrt_area_ratio_all; (unocc_ests.A_unocc_meas).^0.5]
    est_from_gap_size_all = [est_from_gap_size_all, unocc_ests.L_unocc_est_gap_size];
    % est_from_AABB_all = [est_from_AABB_all, unocc_ests.L_unocc_est_AABB_width];
    % est_from_slant_AABB_all = [est_from_slant_AABB_all, unocc_ests.L_unocc_est_slant_AABB_width];
    % est_from_gap_size_normal_all = [est_from_gap_size_normal_all, unocc_ests.L_unocc_est_gap_size_normal];
    % est_from_poly_fit_all = [est_from_poly_fit_all, unocc_ests.L_unocc_est_poly_fit];
end
% Defaults for this blog post
width = 3;     % Width in inches
height = 3;    % Height in inches
alw = 0.75;    % AxesLineWidth
fsz = 11;      % Fontsize
lw = 1.5;      % LineWidth
msz = 8;       % MarkerSize

% The new defaults will not take effect if there are any open figures. To
% use them, we close all figures, and then repeat the first example.
close all;

% The properties we've been using in the figures
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultAxesFontSize',fsz);
set(0,'defaultLegendFontSize',fsz);
set(0,'defaultAxesLineWidth',alw);
% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
set(0,'defaultAxesBox','on')

figure(2)
box on
plot(all_rd,measured_unoccupancy)
hold on
plot(all_rd,est_from_sqrt_area_ratio_all)
plot(all_rd,est_from_gap_size_all)
plot(all_rd,est_from_gap_size_normal_all)
plot(all_rd,est_from_AABB_all)
plot(all_rd,est_from_slant_AABB_all)
plot(all_rd,est_from_poly_fit_all)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear unoccupancy ratio');
legend('measured from planner',...
    'square root of measured area unoccupancy',...
    'estimate from angled gap size',...
    'estimate from normal gap size',...
    'estimate from AABB width',...
    'estimate from center 50% width',...
    'estimate from quadratic fit');

figure(1)
box on
plot(all_rd,1-measured_unoccupancy)
hold on
plot(all_rd,1-est_from_sqrt_area_ratio_all)
plot(all_rd,1-est_from_gap_size_all)
plot(all_rd,1-est_from_gap_size_normal_all)
plot(all_rd,1-est_from_AABB_all)
plot(all_rd,1-est_from_slant_AABB_all)
plot(all_rd,1-est_from_poly_fit_all)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear occupancy ratio');
legend('measured from planner',...
    'square root of measured area unoccupancy',...
    'estimate from angled gap size',...
    'estimate from normal gap size',...
    'estimate from AABB width',...
    'estimate from center 50% width',...
    'estimate from quadratic fit');
