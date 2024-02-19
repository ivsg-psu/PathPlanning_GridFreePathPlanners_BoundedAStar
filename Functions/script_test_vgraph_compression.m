clear; close all; clc
% script_test_vgraph_compression
% create a visibility graph and dilate it to destroy edges
% create a visibiltiy graph and delete edges based on edge length
% overlay them to create a robust visibility graph
% compress this with singular value decomposition and Eckhart-Young theorem
% then plan through the compressed visibility graph to see what happens

addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'

addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\Desktop\gif\gif'

addpath 'C:\Users\sjh6473\Desktop\gif\gif'
addpath 'C:\Users\sjh6473\Desktop\TriangleRayIntersection'

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
shrunk_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(new_polytopes);

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
orig_vgraph = vgraph - eye(size(vgraph));

%% dilation based delete
% polytope_size_increase = 0;
% enlarged_polytopes = fcn_MapGen_polytopesExpandEvenlyForConcave(shrunk_polytopes,polytope_size_increase);
% % find all pts table
% point_tot = length([enlarged_polytopes.xv]); % total number of vertices in the polytopes
% beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
% curpt = 0;
% for poly = 1:size(enlarged_polytopes,2) % check each polytope
%     verts = length(enlarged_polytopes(poly).xv);
%     enlarged_polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
%     beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
%     curpt = curpt+verts;
% end
% obs_id = [enlarged_polytopes.obs_id];
% all_pts_new = [[enlarged_polytopes.xv];[enlarged_polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
% start = [start_init size(all_pts_new,1)+1 -1 1];
% finish = [finish_init size(all_pts_new,1)+2 -1 1];
% finishes = [all_pts_new; start; finish];
% starts = [all_pts_new; start; finish];
% [new_vgraph_dilation, visibility_results_all_pts_new] = fcn_visibility_clear_and_blocked_points_global(enlarged_polytopes, starts, finishes,1);
% reduced_vgraph = new_vgraph_dilation;
% num_edges_initially = sum(sum(orig_vgraph));
% num_edges_finally = sum(sum(new_vgraph_dilation));
% num_edges_removed = num_edges_initially - num_edges_finally;
% pct_edges_removed_dilation = (num_edges_removed)/num_edges_initially*100;

% %% plot result
% figure; hold on; box on;
% blues = zeros(size(orig_vgraph));
% num_orig_nodes = size(orig_vgraph,1);
% reduced_vgraph_concat = reduced_vgraph(1:num_orig_nodes,1:num_orig_nodes);
% reds = orig_vgraph & ~reduced_vgraph_concat;
% greens = reduced_vgraph_concat;
% vgraph_image(:,:,1) = reds;
% vgraph_image(:,:,2) = greens;
% vgraph_image(:,:,3) = blues;
% imshow(vgraph_image*255);
% num_edges_initially = sum(sum(orig_vgraph));
% num_edges_ultimately = sum(sum(reduced_vgraph_concat));
% pct_edges_removed = (num_edges_initially - num_edges_ultimately)/num_edges_initially;
% title(sprintf("%.2f pct. of edges removed, obstacle dilation",pct_edges_removed));

%% length based edge blocking
desired_portion_edge_deletion = 0.2;
vgraph_without_start_and_fin = orig_vgraph(1:end-2,1:end-2);
valid_edges_initially = find(vgraph_without_start_and_fin==1);
num_edges_initially = length(valid_edges_initially);
mode = "xy spatial only";
[cgraph, ~] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
cgraph_without_start_and_fin = cgraph(1:end-2,1:end-2);
costs_of_valid_edges = cgraph_without_start_and_fin(valid_edges_initially); % cost of every 1 in vgraph
vgraph_edge_idx_to_cost_table = [valid_edges_initially, costs_of_valid_edges]; % associate in a table
vgraph_edge_idx_sorted = sortrows(vgraph_edge_idx_to_cost_table,2,'descend'); % sort the table by edge length
num_edges_for_removal = round(num_edges_initially*desired_portion_edge_deletion); % find out how many edges to remove
idx_of_edges_for_removal = vgraph_edge_idx_sorted(1:num_edges_for_removal,1); % take that many edges from top of sorted list
[rows_for_removal, cols_for_removal] = ind2sub(size(vgraph_without_start_and_fin),idx_of_edges_for_removal);
num_edges_removed = length(rows_for_removal);
pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
new_vgraph_random = orig_vgraph;
idx_of_edges_for_removal_updated = sub2ind(size(new_vgraph_random),rows_for_removal,cols_for_removal);
new_vgraph_random(idx_of_edges_for_removal_updated) = 0;
num_edges_after = sum(sum(new_vgraph_random));
pct_edges_removed_random = (num_edges_initially- num_edges_after)/num_edges_initially*100;
%% plot result
figure; hold on; box on;
blues = zeros(size(vgraph));
reds = orig_vgraph & ~new_vgraph_random;
greens = new_vgraph_random;
vgraph_image(:,:,1) = reds;
vgraph_image(:,:,2) = greens;
vgraph_image(:,:,3) = blues;
imshow(vgraph_image*255);
title(sprintf("%.2f pct. of edges removed, random edge blocking",pct_edges_removed_random));

%% short edge first
desired_portion_edge_deletion = 0.2;
vgraph_without_start_and_fin = orig_vgraph(1:end-2,1:end-2);
valid_edges_initially = find(vgraph_without_start_and_fin==1);
num_edges_initially = length(valid_edges_initially);
mode = "xy spatial only";
[cgraph, ~] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);
cgraph_without_start_and_fin = cgraph(1:end-2,1:end-2);
costs_of_valid_edges = cgraph_without_start_and_fin(valid_edges_initially); % cost of every 1 in vgraph
vgraph_edge_idx_to_cost_table = [valid_edges_initially, costs_of_valid_edges]; % associate in a table
vgraph_edge_idx_sorted = sortrows(vgraph_edge_idx_to_cost_table,2,'ascend'); % sort the table by edge length
num_edges_for_removal = round(num_edges_initially*desired_portion_edge_deletion); % find out how many edges to remove
idx_of_edges_for_removal = vgraph_edge_idx_sorted(1:num_edges_for_removal,1); % take that many edges from top of sorted list
[rows_for_removal, cols_for_removal] = ind2sub(size(vgraph_without_start_and_fin),idx_of_edges_for_removal);
num_edges_removed = length(rows_for_removal);
pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
new_vgraph_random = orig_vgraph;
idx_of_edges_for_removal_updated = sub2ind(size(new_vgraph_random),rows_for_removal,cols_for_removal);
new_vgraph_random(idx_of_edges_for_removal_updated) = 0;
num_edges_after = sum(sum(new_vgraph_random));
pct_edges_removed_random = (num_edges_initially- num_edges_after)/num_edges_initially*100;
%% plot result
figure; hold on; box on;
blues = zeros(size(vgraph));
reds = orig_vgraph & ~new_vgraph_random;
greens = new_vgraph_random;
vgraph_image(:,:,1) = reds;
vgraph_image(:,:,2) = greens;
vgraph_image(:,:,3) = blues;
imshow(vgraph_image*255);
title(sprintf("%.2f pct. of edges removed, random edge blocking",pct_edges_removed_random));
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
