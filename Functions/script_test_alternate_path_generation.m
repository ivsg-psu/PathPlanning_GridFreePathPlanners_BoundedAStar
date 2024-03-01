clear; close all; clc
% script_test_alternate_path_generation
% example of using a cost matrix (such as corridor width) to create alternate paths
% with increasingly tight hard constraints rather than only as an incentive/cost or soft constriant

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;
flag_do_plot_slow = 0;

%%%
% map_ID nominal_or_reachable edge_deletion initial_distance navigated_distance replan_route_length
%%%
data = []; % initialize array for storing results
for map_idx = 6%2:6
    if map_idx == 1 % generic canyon map
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
    elseif map_idx == 2 % the lower triangular flood plain
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_1.mat'));
        shrunk_polytopes = flood_plain_1;
        start_init = [-78.3 40.88];
        % finish_init = [-78.1 40.9];
        finish_init = [-78.07 40.82];
    elseif map_idx == 3 % the mustafar mining rig map (the comb)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_2.mat'));
        shrunk_polytopes = flood_plain_2;
        start_init = [-78.02 40.96];
        % finish_init = [-77.86 40.93];
        finish_init = [-77.82 40.97];
    elseif map_idx == 4 % also good for edge deletion case (the long river valleys)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_3.mat'));
        shrunk_polytopes = flood_plain_3;
        start_init = [-77.49 40.84];
        % finish_init = [-77.58 40.845];
        finish_init = [-77.68 40.85];
    elseif map_idx == 5 % bridge map, good for random edge deletion case
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_4.mat'));
        shrunk_polytopes = flood_plain_4;
        start_init = [-77.68 40.9];
        finish_init = [-77.5 40.8];
    elseif map_idx == 6 % large map, good for dilation case, nearly fully tiled
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_5.mat'));
        shrunk_polytopes = flood_plain_5;
        start_init = [-78.01 41.06];
        finish_init = [-77.75 40.93];
    elseif map_idx == 7 % generic polytope map
        % pull halton set
        halton_points = haltonset(2);
        points_scrambled = scramble(halton_points,'RR2'); % scramble values

        % pick values from halton set
        Halton_range = [1801 1851];
        low_pt = Halton_range(1,1);
        high_pt = Halton_range(1,2);
        seed_points = points_scrambled(low_pt:high_pt,:);

        % fill polytopes from tiling
        AABB = [0 0 1 1];
        stretch = AABB(3:4);
        tiled_polytopes = fcn_MapGen_generatePolysFromVoronoiAABBWithTiling(seed_points,AABB, stretch);

        % stretch polytopes to cover more area
        new_stretch = [30 40];
        stretched_polytopes = [];
        for poly = 1:length(tiled_polytopes) % pull each cell from the voronoi diagram
            stretched_polytopes(poly).vertices  = tiled_polytopes(poly).vertices.*new_stretch;
        end % Ends for loop for stretch
        stretched_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(stretched_polytopes);

        % shrink polytopes to desired radius
        des_rad = 2; sigma_radius = 0.4; min_rad = 0.1;
        [shrunk_polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(stretched_polytopes,des_rad,sigma_radius,min_rad);

        clear Halton_range
        clear halton_points
        clear points_scrambled

        start_init = [-2 20];
        finish_init = [32 20];
        % tile field to hedgerow by making a set above and a set below
    end % if conditions for different map test fixtures
    if map_idx <=6 && map_idx >= 2 % for the floodplain maps we have to convert from LLA to km
        %% convert from LLA to QGS84
        centre_co_avg_alt = 351.7392;
        start_init = INTERNAL_WGSLLA2xyz(start_init(2),start_init(1),centre_co_avg_alt);
        start_init = start_init(1:2)';
        start_init = start_init/1000;
        finish_init = INTERNAL_WGSLLA2xyz(finish_init(2),finish_init(1),centre_co_avg_alt);
        finish_init = finish_init(1:2)';
        finish_init = finish_init/1000;
        new_polytopes = [];
        for i = 1:length(shrunk_polytopes)
            poly = shrunk_polytopes(i);
            lats = poly.vertices(:,2);
            longs = poly.vertices(:,1);
            alts = centre_co_avg_alt*ones(size(lats));
            wgs_verts = [];
            for j = 1:length(lats)
                xyz = INTERNAL_WGSLLA2xyz(lats(j),longs(j),alts(j));
                xyz = xyz/1000;
                wgs_verts(j,:) = [xyz(1),xyz(2)];
            end
            new_polytopes(i).vertices = wgs_verts;
        end
        shrunk_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(new_polytopes);
    end
    if map_idx == 7 % for map 6 we can loop over many start goal pairs
        start_inits = [start_init; 1015,-4704; 1000,-4722; 1017 -4721; 995, -4714; 1025, -4704; 1030, -4708];
        finish_inits = [finish_init; 1010, -4722 ; 1027, -4704; 1007 -4707; 1030, -4712; 1005, -4722; 995 -4722];
    else % if we only have one start goal pair
        start_inits = start_init;
        finish_inits = finish_init;
    end
    for mission_idx = 1:size(start_inits,1)
        start_init = start_inits(mission_idx,:);
        finish_init = finish_inits(mission_idx,:);
    %% all_pts array creation
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
    if map_idx == 10
        % small polytopes in this map so need smaller polytope size increase
        polytope_size_increases_values = [0.01 0.02 0.05 0.1 0.2];% 0.3 0.4 0.5]%[20, 50, 100, 200]
    else
        % flood plain maps are large so can tolerate large dilation
        polytope_size_increases_values = [0.01 0.02 0.05 0.1 0.2 0.3 0.4 0.5];%[20, 50, 100, 200]
    end
    for polytope_size_increases = 1%polytope_size_increases_values
            %% plan the initial path
            start = [start_init size(all_pts,1)+1 -1 1];
            finish = [finish_init size(all_pts,1)+2 -1 1];
            finishes = [all_pts; start; finish];
            starts = [all_pts; start; finish];
            [vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);
            orig_vgraph = vgraph;
            start_for_reachability = start;
            start_for_reachability(4) = start(3);
            finish_for_reachability = finish;
            finish_for_reachability(4) = finish(3);

            [is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start_for_reachability,finish_for_reachability);
            if ~is_reachable
                error('initial mission, prior to edge deletion, is not possible')
            end

            %% make cgraph
            mode = "xy spatial only";
            % mode = 'time or z only';
            % mode = "xyz or xyt";
            [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);

            mode = '2d';
            dilationtimer = tic;
            dilation_robustness_tensor = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode, shrunk_polytopes);
            dilation_robustness_matrix = max(dilation_robustness_tensor(:,:,1), dilation_robustness_tensor(:,:,2));
            dilation_robustness_matrix_for_variance = dilation_robustness_matrix(:)';
            dilation_robustness_matrix_for_variance(dilation_robustness_matrix_for_variance == 0) = [];
            dilation_robustness_matrix_for_variance(isinf(dilation_robustness_matrix_for_variance)) = [];
            variance_of_corridor_widths = var(dilation_robustness_matrix_for_variance);
            my_time = toc(dilationtimer)

            [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
            routes{1} = init_route;
            % find route length
            route_x = init_route(:,1);
            route_y = init_route(:,2);
            lengths = diff([route_x(:) route_y(:)]);
            init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
            for i = 1:4
                %% plan next best path given tighter constraints
                route_segment_costs = nan(1,size(init_route,1)-1);
                route_segment_vis  = nan(1,size(init_route,1)-1);
                route_segment_corridor_widths = nan(1,size(init_route,1)-1);
                for waypoint_id = 1:(size(init_route,1)-1)
                    route_segment_start = init_route(waypoint_id,:);
                    route_segment_end = init_route(waypoint_id+1,:);
                    route_segment_costs(waypoint_id) = cgraph(route_segment_start(3), route_segment_end(3));
                    route_segment_vis(waypoint_id) = vgraph(route_segment_start(3), route_segment_end(3));
                    route_segment_corridor_widths(waypoint_id) = dilation_robustness_matrix(route_segment_start(3), route_segment_end(3));
                end
                smallest_corridor_in_init_path = 1.5*min(route_segment_corridor_widths) % find smallest corridor in route
                new_vgraph = vgraph; % copy vgraph
                % set all vgraph edges where corridor width is smaller or equal to 0
                new_vgraph(dilation_robustness_matrix <= smallest_corridor_in_init_path) = 0;

                num_edges_initially = sum(sum(vgraph));
                num_edges_finally = sum(sum(new_vgraph));
                num_edges_removed = num_edges_initially - num_edges_finally;
                pct_edges_removed = (num_edges_removed)/num_edges_initially*100;

                start_for_reachability = start;
                start_for_reachability(4) = start(3);
                finish_for_reachability = finish;
                finish_for_reachability(4) = finish(3);

                [is_reachable, num_steps, rgraph] = fcn_check_reachability(new_vgraph,start_for_reachability,finish_for_reachability);
                if ~is_reachable
                    warning('mission replanning is impossible')
                    continue
                end % end is_reachable condition for replanning

                %% make cgraph

                [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts, start, finish);
                routes{end+1} = replan_route;
                init_route = replan_route
                % find route length
                route_x = replan_route(:,1);
                route_y = replan_route(:,2);
                lengths = diff([route_x(:) route_y(:)]);
                replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));
            end
            % plot field, initial path, replan path, and midway point
            if flag_do_plot
                figure; hold on; box on;
                xlabel('x [km]');
                ylabel('y [km]');
                plot(start_init(1),start_init(2),'xg','MarkerSize',6);
                plot(finish_init(1),finish_init(2),'xr','MarkerSize',6);
                leg_str = {'start','finish'};
                for i = 1:length(routes)
                    route_to_plot = routes{i};
                    plot(route_to_plot(:,1),route_to_plot(:,2),'LineWidth',2);
                    leg_str{end+1} = sprintf('route choice %i',i);
                end
                for j = 1:length(shrunk_polytopes)
                     fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
                end
                leg_str{end+1} = 'obstacles';
                for i = 1:length(shrunk_polytopes)-1
                    leg_str{end+1} = '';
                end
                legend(leg_str,'Location','best');
            end % end flag_do_plot condition
    end % end edge deletion portion loop
end % end mission (i.e., start goal pair) loop
end % end map loop
return
% plot histogram of failed trials
nandata = data(find(isnan(data(:,7))),:); % find nan replan cost rows
nandata_nominal = nandata(nandata(:,2)==1,:); % of those, find nominal ones
nandata_reachable = nandata(nandata(:,2)==2,:); % of those, find reachable ones
polytope_size_bins = [0.005 0.015 0.03 0.075 0.15 0.25 0.35 0.45 0.55];
% polytope_size_increases = [0.01 0.02 0.05 0.1 0.2 0.3 0.4 0.5]%[20, 50, 100, 200]

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

% function LL2KM(lat1, lon1, lat2, lon2){  // generally used geo measurement function
%     var R = 6378.137; // Radius of earth in KM
%     var dLat = lat2 * Math.PI / 180 - lat1 * Math.PI / 180;
%     var dLon = lon2 * Math.PI / 180 - lon1 * Math.PI / 180;
%     var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
%     Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
%     Math.sin(dLon/2) * Math.sin(dLon/2);
%     var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
%     var d = R * c;
%     return d * 1000; // meters
% }
function xyz = INTERNAL_WGSLLA2xyz(wlat, wlon, walt)
    %Function xyz = wgslla2xyz(lat, lon, alt) returns the
    %equivalent WGS84 XYZ coordinates (in meters) for a
    %given geodetic latitude "lat" (degrees), longitude "lon"
    %(degrees), and altitude above the WGS84 ellipsoid
    %in meters.  Note: N latitude is positive, S latitude
    %is negative, E longitude is positive, W longitude is
    %negative.
    %
    %Ref: Decker, B. L., World Geodetic System 1984,
    %Defense Mapping Agency Aerospace Center.

    A_EARTH = 6378137;
    flattening = 1/298.257223563;
    NAV_E2 = (2-flattening)*flattening; % also e^2
    deg2rad = pi/180;

    slat = sin(wlat*deg2rad);
    clat = cos(wlat*deg2rad);
    r_n = A_EARTH/sqrt(1 - NAV_E2*slat*slat);
    xyz = [ (r_n + walt)*clat*cos(wlon*deg2rad);
            (r_n + walt)*clat*sin(wlon*deg2rad);
            (r_n*(1 - NAV_E2) + walt)*slat ];

    if ((wlat < -90.0) | (wlat > +90.0) | (wlon < -180.0) | (wlon > +360.0))
        error('WGS lat or WGS lon out of range');
    end
end
