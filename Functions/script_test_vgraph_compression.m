
% REVISION HISTORY:
% 2025_10_05 by S. Brennan
% -- removed call to fcn_MapGen_fillPolytopeFieldsFromVertices 
%    % replaced with fcn_MapGen_polytopesFillFieldsFromVertices

clear; close all; clc
% script_test_vgraph_compression
% experiment of removing "unecessary" edges from the vgraph by performing a singular value decomposition
% and subsequent low rank matrix approximation using
% Eckhart-Young theorem
% see: https://en.wikipedia.org/wiki/Singular_value_decomposition#Low-rank_matrix_approximation
% then plan through the compressed visibility graph to see how impacted the path is compared
% to the original visibility graph


addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;
flag_do_plot_slow = 1;

%% load map
load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_5.mat'));
shrunk_polytopes = flood_plain_5;
start_init = [-78.01 41.06];
finish_init = [-77.75 40.93];

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
shrunk_polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(new_polytopes);

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

%% make original vgraph
start = [start_init size(all_pts,1)+1 -1 1];
finish = [finish_init size(all_pts,1)+2 -1 1];
finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
[vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes,1);
orig_vgraph_without_eye = vgraph - eye(size(vgraph));

%% plan original path
mode = "xy spatial only";
% mode = 'time or z only';
% mode = "xyz or xyt";
[cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
init_planning_time = tic;
[init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);
toc(init_planning_time)

%% compress vgraph
[U,S,V] = svd(vgraph); % perform singular value decomposition
critical_singular_value = 7; % apply eckhart young to remove SVs below some SV
compressed_vgraph_S = S;
compressed_vgraph_S(compressed_vgraph_S<7) = 0;
compressed_vgraph = round(U*compressed_vgraph_S*V');
% add back start and finish rows and columns
compressed_vgraph(:,end) = vgraph(:,end);
compressed_vgraph(:,end-1) = vgraph(:,end-1);
compressed_vgraph(end,:) = vgraph(end,:);
compressed_vgraph(end-1,:) = vgraph(end-1,:);
% keep cgraph the same as it just contains costs for all  possible node pairs
compressed_cgraph = cgraph;

[is_reachable, num_steps, rgraph] = fcn_check_reachability(compressed_vgraph,start(3),finish(3));
if ~is_reachable
    error('compressed map does not contain path from start to goal')
end
compressed_planning_time = tic;
[compressed_cost, compressed_route] = fcn_algorithm_Astar(compressed_vgraph, compressed_cgraph, hvec, all_pts, start, finish);
toc(compressed_planning_time)

sprintf('initial route IDs \n')
init_route(:,3)
sprintf('compressed route IDs \n')
compressed_route(:,3)
vgraph_node_count_init = sum(sum(vgraph));
vgraph_node_count_compressed = sum(sum(compressed_vgraph));
if flag_do_plot
    figure; hold on; box on;
    xlabel('x [km]');
    ylabel('y [km]');
    plot(start_init(1),start_init(2),'xg','MarkerSize',6);
    plot(finish(1),finish(2),'xr','MarkerSize',6);
    plot(init_route(:,1),init_route(:,2),'k','LineWidth',2);
    plot(compressed_route(:,1),compressed_route(:,2),'--r','LineWidth',2);
    for j = 1:length(shrunk_polytopes)
         fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    end
    leg_str = {'start','finish','route','compressed route','obstacles'};
    for i = 1:length(shrunk_polytopes)-1
        leg_str{end+1} = '';
    end
    legend(leg_str,'Location','best');
    title_str = sprintf('Initial vgraph contained %i nodes. \n Compressed vgraph contained %i nodes.',vgraph_node_count_init,vgraph_node_count_compressed);
    title(title_str);
end % end flag_do_plot condition

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
