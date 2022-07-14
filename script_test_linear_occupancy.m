%% add necessary directories
addpath([pwd '\Example_Map_Generation_Code'])
addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

%% initialize loop params and storage arrays for plotting
des_gap_size = linspace(0.0001,0.08,30);
all_rd = [];

flag_do_plot = 0;

%% initialize loop params and storage arrays for plotting
measured_unoccupancy = [];
est_from_sqrt_area_ratio_all = [];
est_from_gap_size_all = [];
est_from_AABB_all = [];
est_from_slant_AABB_all = [];
est_from_gap_size_normal_all = [];
est_from_poly_fit_all = [];
est_mean_all_rad_all = [];
est_mean_mean_rad_all = [];
est_med_all_rad_all = [];
est_med_mean_rad_all = [];
est_25th_all_rad_all = [];
est_25th_mean_rad_all = [];
r_D_for_meas = [];

% generate Voronoi tiling from Halton points
low_pt = 1; high_pt = 1000; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);

% use radial shrinking to get non-zero standard deviations of obstacle size
% only for measured occupancy, predicted occupancy will still come from edge shrinking
% because gap size is a required input
sd_radius_values = [0, 0.01, 0.02, 0.04, 0.08, 0.16];%, 0.32];
% loop through radius distributions
for sd_radius_index = 1:1:length(sd_radius_values)
    sd_radius = sd_radius_values(sd_radius_index);
    % loop through radius goals
    for radii_goals = 0.001:0.0025:0.081
        des_rad = radii_goals; sigma_radius = sd_radius; min_rad = 0.001;
        try
            [shrunk_field,mu_final,sigma_final] = ...
                fcn_MapGen_polytopesShrinkToRadius(...
                    trim_polytopes,des_rad,sigma_radius,min_rad...
            );
        catch
            sprintf('shrink failed at des. rad. %2f and sig. rad. %2f',des_rad,sigma_radius);
            continue
        end
        % set polytope traversal cost. it doesn't matter what this is, as long as it's known.
        des_cost = 0.2;
        shrunk_polytopes_known_cost = fcn_polytope_editing_set_all_costs(shrunk_field,des_cost);
        % find measured occupancy at different heights
        for height_of_path = 0.1:0.2:0.9
            %% plan path
            A.x = 0; A.y = height_of_path; B.x = 1; B.y = height_of_path;
            try
                [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes_known_cost,A,B,"straight through");
                % total_path_cost = dist_outside + dist_inside * (1+traversal_cost)
                % => total_path_cost = (path_length - dist_inside) + dist_inside * (1+traversal_cost)
                % => if path_length = 1: (total_path_cost - 1)/traversal_cost = dist_inside
                dist_inside = (cost-1)/des_cost;
                % because the path length is 1, dist_inside is linear occupancy
                measured_unoccupancy = [measured_unoccupancy, (1-dist_inside)];
                %% polytope stats to create inputs for predictor code
                field_stats = fcn_MapGen_polytopesStatistics(shrunk_polytopes_known_cost);
                field_stats_pre_shrink = fcn_MapGen_polytopesStatistics(trim_polytopes);
                % extract parameters of interest
                field_avg_r_D = field_stats.avg_r_D;
                r_D_for_meas = [r_D_for_meas, field_avg_r_D];
                straight_path_costs = [straight_path_costs, cost];
            catch
                sprintf('meas. cost failed for des. rad. %2f and sig. rad. %2f',des_rad,sigma_radius);
            end
        end
    end
end


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

    if flag_do_plot
        % plot the map
        line_spec = 'b-'; % edge line plotting
        line_width = 2; % linewidth of the edge
        axes_limits = [0 1 0 1]; % x and y axes limits
        axis_style = 'square'; % plot axes style
        fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    end



    %% find estimated values
    unocc_ests = fcn_MapGen_polytopesPredictUnoccupancyRatio(trim_polytopes,shrunk_polytopes,gap_size);
    est_from_sqrt_area_ratio_all = [est_from_sqrt_area_ratio_all, (unocc_ests.A_unocc_meas).^0.5];
    est_from_gap_size_all = [est_from_gap_size_all, unocc_ests.L_unocc_est_gap_size];
    est_from_AABB_all = [est_from_AABB_all, unocc_ests.L_unocc_est_AABB_width];
    est_from_slant_AABB_all = [est_from_slant_AABB_all, unocc_ests.L_unocc_est_slant_AABB_width];
    est_from_gap_size_normal_all = [est_from_gap_size_normal_all, unocc_ests.L_unocc_est_gap_size_normal];
    est_from_poly_fit_all = [est_from_poly_fit_all, unocc_ests.L_unocc_est_poly_fit];
    est_mean_all_rad_all = [est_mean_all_rad_all, unocc_ests.L_unocc_est_mean_all_rad];
    est_mean_mean_rad_all = [est_mean_mean_rad_all, unocc_ests.L_unocc_est_mean_mean_rad];
    est_med_all_rad_all = [est_med_all_rad_all, unocc_ests.L_unocc_est_med_all_rad];
    est_med_mean_rad_all = [est_med_mean_rad_all, unocc_ests.L_unocc_est_med_mean_rad];
    est_25th_all_rad_all = [est_25th_all_rad_all, unocc_ests.L_unocc_est_25th_all_rad];
    est_25th_mean_rad_all = [est_25th_mean_rad_all, unocc_ests.L_unocc_est_25th_mean_rad];
end

figure(2)
box on
plot(r_D_for_meas, measured_unoccupancy, 'kd')
hold on
plot(all_rd, est_from_sqrt_area_ratio_all)
plot(all_rd, est_from_gap_size_all)
plot(all_rd, est_from_gap_size_normal_all)
plot(all_rd, est_from_AABB_all)
plot(all_rd, est_from_slant_AABB_all)
plot(all_rd, est_from_poly_fit_all)
plot(all_rd, est_mean_all_rad_all)
plot(all_rd, est_mean_mean_rad_all)
plot(all_rd, est_med_all_rad_all)
plot(all_rd, est_med_mean_rad_all)
plot(all_rd, est_25th_all_rad_all)
plot(all_rd, est_25th_mean_rad_all)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear unoccupancy ratio');
legend('measured from planner',...
    'square root of measured area unoccupancy',...
    'estimate from angled gap size',...
    'estimate from normal gap size',...
    'estimate from AABB width',...
    'estimate from center 50% width',...
    'estimate from quadratic fit',...
    'estimate from mean of all radii',...
    'estiamte from mean of mean radii',...
    'estimate from median of all radii',...
    'estimate from median of mean radii',...
    'estimate from 25th pct of all radii',...
    'estimate from 25th pct of mean radii');

figure(1)
box on
plot(r_D_for_meas, 1-measured_unoccupancy,'kd')
hold on
plot(all_rd,1-est_from_sqrt_area_ratio_all)
plot(all_rd,1-est_from_gap_size_all)
plot(all_rd,1-est_from_gap_size_normal_all)
plot(all_rd,1-est_from_AABB_all)
plot(all_rd,1-est_from_slant_AABB_all)
plot(all_rd,1-est_from_poly_fit_all)
plot(all_rd, 1-est_mean_all_rad_all)
plot(all_rd, 1-est_mean_mean_rad_all)
plot(all_rd, 1-est_med_all_rad_all)
plot(all_rd, 1-est_med_mean_rad_all)
plot(all_rd, 1-est_25th_all_rad_all)
plot(all_rd, 1-est_25th_mean_rad_all)
xlabel('departure ratio [r_D]');
% measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
ylabel('linear occupancy ratio');
legend('measured from planner',...
    'square root of measured area unoccupancy',...
    'estimate from angled gap size',...
    'estimate from normal gap size',...
    'estimate from AABB width',...
    'estimate from vertex IQR width',...
    'estimate from quadratic fit',...
    'estimate from avg. circ. value and avg. min radius',...
    'estiamte from half avg. circ. value and avg. min radius',...
    'estimate from avg. circ. value and est. min radius',...
    'estiamte from half avg. circ. value and est. min radius',...
    'estimate from avg. circ. value and alt. est. min radius',...
    'estiamte from half avg. circ. value and alt. est. min radius');


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
% plot(all_rd, (est_mean_all_rad_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_mean_mean_rad_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_med_all_rad_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_med_mean_rad_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_25th_all_rad_all-measured_unoccupancy)./measured_unoccupancy)
% plot(all_rd, (est_25th_mean_rad_all-measured_unoccupancy)./measured_unoccupancy)
% xlabel('departure ratio [r_D]');
% % measuring distance outside of polytopes for 1 km travel i.e. unoccupancy
% ylabel('linear occupancy ratio percent error');
% legend(...
%     'square root of measured area unoccupancy',...
%     'estimate from angled gap size',...
%     'estimate from normal gap size',...
%     'estimate from AABB width',...
%     'estimate from vertex IQR width',...
%     'estimate from quadratic fit',...
%     'estimate from avg. circ. value and avg. min radius',...
%     'estiamte from half avg. circ. value and avg. min radius',...
%     'estimate from avg. circ. value and est. min radius',...
%     'estiamte from half avg. circ. value and est. min radius',...
%     'estimate from avg. circ. value and alt. est. min radius',...
%     'estiamte from half avg. circ. value and alt. est. min radius');
%
