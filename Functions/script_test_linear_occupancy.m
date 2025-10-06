% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- fixed calls to fcn_MapGen_polytopesStatistics, replaced with fcn_MapGen_statsPolytopes

% % add necessary directories
% addpath([pwd '\..\Example_Map_Generation_Code'])
% addpath([pwd '\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
% addpath([pwd '\..\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

% %% initialize loop params and storage arrays for plotting
des_gap_size = linspace(0.0001,0.08,30);
all_rd = [];
%
flag_do_plot = 0;
%
% %% initialize loop params and storage arrays for plotting
measured_unoccupancy = [];
est_from_sqrt_area_ratio_all = [];
est_from_gap_size_all = [];
est_from_AABB_all = [];
est_from_slant_AABB_all = [];
est_from_gap_size_normal_all = [];
est_from_poly_fit_all = [];
est_avg_circ_min_rad = [];
est_avg_circ_min_rad_est1 = [];
est_avg_circ_min_rad_est2 = [];
r_D_for_meas = [];
straight_path_costs = [];
est_d_eff = [];
est_d_eff2 = [];
est_d_eff3 = [];
est_d_eff4 = [];
est_d_eff5 = [];
est_d_eff6 = [];
est_d_eff7 = [];
%
% generate Voronoi tiling from Halton points

% for halton_seeds = 0:2000:10000
    low_pt = 1; high_pt = 100%1000+halton_seeds; % range of Halton points to use to generate the tiling
    trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
    for gap_idx = 1:14%length(des_gap_size)
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

        %% find measured occupancy
        % set polytope traversal cost. it doesn't matter what this is, as long as it's known.
        des_cost = 0.2;        
        shrunk_polytopes = fcn_MapGen_polytopesSetCosts(shrunk_polytopes, des_cost, (-1));

        % find measured occupancy at different heights
        for height_of_path = 0.1:0.2:0.9
            %% plan path
            A.x = 0; A.y = height_of_path; B.x = 1; B.y = height_of_path;
            [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes_known_cost,A,B,"straight through");
            % total_path_cost = dist_outside + dist_inside * (1+traversal_cost)
            % => total_path_cost = (path_length - dist_inside) + dist_inside * (1+traversal_cost)
            % => if path_length = 1: (total_path_cost - 1)/traversal_cost = dist_inside
            dist_inside = (cost-1)/des_cost;
            % because the path length is 1, dist_inside is linear occupancy
            measured_unoccupancy = [measured_unoccupancy, (1-dist_inside)];
            %% polytope stats to create inputs for predictor code
            field_stats = fcn_MapGen_statsPolytopes(shrunk_polytopes_known_cost);
            % extract parameters of interest
            field_avg_r_D = field_stats.avg_r_D;
            r_D_for_meas = [r_D_for_meas, field_avg_r_D];
            straight_path_costs = [straight_path_costs, cost];
        end
    end
% end

low_pt = 1; high_pt = 100; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);

%% begin loop of departure ratios
for gap_idx = 1:1:14%1:length(des_gap_size)
    est_avg_circ_min_rad_est1_this_rd = [];
    est_d_eff_this_rd = [];
    est_d_eff_this_rd2 = [];
    est_d_eff_this_rd3 = [];
    est_d_eff_this_rd4 = [];
%     est_d_eff_this_rd6 = [];
    est_d_eff_this_rd5 = [];
%     est_d_eff_this_rd7 = [];
    rd_this_rd = [];
    % for halton_seeds = 0:2000:10000
        low_pt = 1+halton_seeds; high_pt = 1000+halton_seeds; % range of Halton points to use to generate the tiling
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
        rd_this_rd = [rd_this_rd, field_avg_r_D];

        if flag_do_plot
            % plot the map
            line_spec = 'b-'; % edge line plotting
            line_width = 2; % linewidth of the edge
            axes_limits = [0 1 0 1]; % x and y axes limits
            axis_style = 'square'; % plot axes style
            fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
        end
        unocc_ests = fcn_MapGen_polytopesPredictUnoccupancyRatio(trim_polytopes,shrunk_polytopes,gap_size);
        est_avg_circ_min_rad_est1_this_rd = [est_avg_circ_min_rad_est1_this_rd, unocc_ests.L_unocc_est_avg_circle_min_rad_est_1];
        est_d_eff_this_rd = [est_d_eff_this_rd, unocc_ests.L_unocc_est_d_eff];
        est_d_eff_this_rd2 = [est_d_eff_this_rd2, unocc_ests.L_unocc_est_d_eff2];
        est_d_eff_this_rd3 = [est_d_eff_this_rd3, unocc_ests.L_unocc_est_d_eff3];
        est_d_eff_this_rd4 = [est_d_eff_this_rd4, unocc_ests.L_unocc_est_d_eff4];
        est_d_eff_this_rd5 = [est_d_eff_this_rd5, unocc_ests.L_unocc_est_d_eff5];
%         est_d_eff_this_rd6 = [est_d_eff_this_rd3, unocc_ests.L_unocc_est_d_eff6];
%         est_d_eff_this_rd7 = [est_d_eff_this_rd7, unocc_ests.L_unocc_est_d_eff7];
    % end
    %% find estimated values

%     est_from_sqrt_area_ratio_all = [est_from_sqrt_area_ratio_all, (unocc_ests.A_unocc_meas).^0.5];
%     est_from_gap_size_all = [est_from_gap_size_all, unocc_ests.L_unocc_est_gap_size];
%     est_from_AABB_all = [est_from_AABB_all, unocc_ests.L_unocc_est_AABB_width];
%     est_from_slant_AABB_all = [est_from_slant_AABB_all, unocc_ests.L_unocc_est_slant_AABB_width];
%     est_from_gap_size_normal_all = [est_from_gap_size_normal_all, unocc_ests.L_unocc_est_gap_size_normal];
%     est_from_poly_fit_all = [est_from_poly_fit_all, unocc_ests.L_unocc_est_poly_fit];
%     est_avg_circ_min_rad = [est_avg_circ_min_rad, unocc_ests.L_unocc_est_avg_circle_min_rad];
    est_avg_circ_min_rad_est1 = [est_avg_circ_min_rad_est1, nanmean(est_avg_circ_min_rad_est1_this_rd)];
%     est_avg_circ_min_rad_est2 = [est_avg_circ_min_rad_est2, unocc_ests.L_unocc_est_avg_circle_min_rad_est_2];
    est_d_eff = [est_d_eff, nanmean(est_d_eff_this_rd)];
    est_d_eff2 = [est_d_eff2, nanmean(est_d_eff_this_rd2)];
    est_d_eff3 = [est_d_eff3, nanmean(est_d_eff_this_rd3)];
    est_d_eff4 = [est_d_eff4, nanmean(est_d_eff_this_rd4)];
    est_d_eff5 = [est_d_eff5, nanmean(est_d_eff_this_rd5)];
%     est_d_eff6 = [est_d_eff6, mean(est_d_eff_this_rd6)];
%     est_d_eff7 = [est_d_eff7, mean(est_d_eff_this_rd7)];
    all_rd = [all_rd,nanmean(rd_this_rd)];
end

figure(2)
box on
plot(r_D_for_meas, measured_unoccupancy, 'kd')
hold on
% plot(all_rd, est_from_sqrt_area_ratio_all)
% plot(all_rd, est_from_gap_size_all)
% plot(all_rd, est_from_gap_size_normal_all)
% plot(all_rd, est_from_AABB_all)
% plot(all_rd, est_from_slant_AABB_all)
% plot(all_rd, est_from_poly_fit_all)
% plot(all_rd, est_avg_circ_min_rad)
plot(all_rd, est_avg_circ_min_rad_est1)
% plot(all_rd, est_avg_circ_min_rad_est2)
plot(all_rd, est_d_eff)
plot(all_rd, est_d_eff2)
plot(all_rd, est_d_eff3)
plot(all_rd, est_d_eff4)
plot(all_rd, est_d_eff5)
% plot(all_rd, est_d_eff6)
% plot(all_rd, est_d_eff7)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear unoccupancy ratio');
legend('measured from planner',...
    'estimate from avg. circ. value and avg. min radius',...
    'estimate from d_{eff}',...
    'estimate from d_{eff}2',...
    'estimate from d_{eff}3',...
    'estimate from d_{eff}4',...
    'estimate from d_{eff}5',...
    'estimate from d_{eff}6',...
    'estimate from d_{eff}7');

figure(1)
box on
plot(r_D_for_meas, 1-measured_unoccupancy,'kd')
hold on
% plot(all_rd,1-est_from_sqrt_area_ratio_all)
% plot(all_rd,1-est_from_gap_size_all)
% plot(all_rd,1-est_from_gap_size_normal_all)
% plot(all_rd,1-est_from_AABB_all)
% plot(all_rd,1-est_from_slant_AABB_all)
% plot(all_rd,1-est_from_poly_fit_all)
% plot(all_rd, 1-est_avg_circ_min_rad)
plot(all_rd, 1-est_avg_circ_min_rad_est1)
% plot(all_rd, 1-est_avg_circ_min_rad_est2)
plot(all_rd, 1-est_d_eff)
plot(all_rd, 1-est_d_eff2)
plot(all_rd, 1-est_d_eff3)
plot(all_rd, 1-est_d_eff4)
plot(all_rd, 1-est_d_eff5)
% plot(all_rd, 1-est_d_eff6)
% plot(all_rd, 1-est_d_eff7)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear occupancy ratio');
legend('measured from planner',...
    'estimate from avg. circ. value and avg. min radius',...
    'estimate from d_{eff}',...
    'estimate from d_{eff}2',...
    'estimate from d_{eff}3',...
    'estimate from d_{eff}4',...
    'estimate from d_{eff}5',...
    'estimate from d_{eff}6',...
    'estimate from d_{eff}7');


% figure(3)
% clf
% box on
% % plot(all_rd,measured_unoccupancy)
% hold on
% plot(all_rd, (est_from_sqrt_area_ratio_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_from_gap_size_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_from_gap_size_normal_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_from_AABB_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_from_slant_AABB_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_from_poly_fit_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_avg_circ_min_rad-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_avg_circ_min_rad_est1-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_avg_circ_min_rad_est2-measured_unoccupancy)./measured_unoccupancy)
% xlabel('departure ratio [r_D]');
% % measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
% ylabel('linear occupancy ratio percent error');
% legend('square root of measured area unoccupancy',...
%     'estimate from angled gap size',...
%     'estimate from normal gap size',...
%     'estimate from AABB width',...
%     'estimate from vertex IQR width',...
%     'estimate from quadratic fit',...
%     'estimate from avg. circ. value and avg. min radius',...
%     'estimate from avg. circ. value and est. min radius',...
%     'estimate from avg. circ. value and alt. est. min radius');
%


figure(4)
box on
hold on
plot(r_D_for_meas, measured_unoccupancy+(1.1*(1-measured_unoccupancy)),'rd')
plot(r_D_for_meas, measured_unoccupancy+(1.2*(1-measured_unoccupancy)),'gd')
plot(r_D_for_meas, measured_unoccupancy+(1.3*(1-measured_unoccupancy)),'bd')
plot(all_rd, est_avg_circ_min_rad_est1+(1.1*(1-est_avg_circ_min_rad_est1)),'r')
plot(all_rd, est_avg_circ_min_rad_est1+(1.2*(1-est_avg_circ_min_rad_est1)),'g')
plot(all_rd, est_avg_circ_min_rad_est1+(1.3*(1-est_avg_circ_min_rad_est1)),'b')


xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('length cost ratio [r_{LC}]');
legend('measured for polytope traversal cost of 110%',...
    'measured for polytope traversal cost of 120%',...
    'measured for polytope traversal cost of 130%',...
    'estimated for polytope traversal cost of 110%',...
    'estimated for polytope traversal cost of 120%',...
    'estimated for polytope traversal cost of 130%');
