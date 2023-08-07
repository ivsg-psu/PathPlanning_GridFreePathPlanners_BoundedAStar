clear; close all; clc

addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'

addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\Desktop\gif\gif'

addpath 'C:\Users\sjh6473\Desktop\gif\gif'
addpath 'C:\Users\sjh6473\Desktop\TriangleRayIntersection'

addpath 'C:\Users\sjhar\Documents\GitHub\Errata_Tutorials_DebugTools\Functions\'
addpath 'C:\Users\sjhar\Documents\GitHub\PathPlanning_MapTools_MapGenClassLibrary\Functions'

load('C:\Users\sjhar\Documents\GitHub\sjharnett\figures\3d_planning_canyon_example\canyon_polys_without_exterior.mat')
tic

% make distribution north of canyon
tiled_polytopes1 = fcn_MapGen_haltonVoronoiTiling([1,20],[2 1]);
% remove the edge polytope that extend past the high and low points
% shink the polytopes so that they are no longer tiled
des_radius = 0.05; % desired average maximum radius
sigma_radius = 0.002; % desired standard deviation in maximum radii
min_rad = 0.0001; % minimum possible maximum radius for any obstacle
[shrunk_polytopes1,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes1,des_radius,sigma_radius,min_rad);

% make distribution south of canyon
tiled_polytopes2 = fcn_MapGen_haltonVoronoiTiling([1, 20],[2 1]);
% remove the edge polytope that extend past the high and low points
% shink the polytopes so that they are no longer tiled
[shrunk_polytopes2,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes2,des_radius,sigma_radius,min_rad);

second_field_vertical_translation = 1.5;

for i = 1:length(shrunk_polytopes2)
    num_verts_this_poly = length(shrunk_polytopes2(i).yv);
    shrunk_polytopes2(i).yv = shrunk_polytopes2(i).yv + second_field_vertical_translation;
    shrunk_polytopes2(i).vertices = shrunk_polytopes2(i).vertices + [zeros(num_verts_this_poly+1,1) second_field_vertical_translation*ones(num_verts_this_poly+1,1)];
end

shrunk_polytopes = [shrunk_polytopes1, shrunk_polytopes2, polytopes_manual_canyon];
shrunk_polytopes = fcn_polytope_editing_set_all_costs(shrunk_polytopes,0);

% plot the map
fig = 99; % figure to plot on
line_spec = 'b-'; % edge line plotting
line_width = 2; % linewidth of the edge
axes_limits = [0 2 0 2.5]; % x and y axes limits
axis_style = 'square'; % plot axes style
fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
INTERNAL_fcn_format_timespace_plot()
view([0 90])
title('time 0 of a bottleneck map')

start = [0 1.25];
finish = [2 1.25];


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

%% plan path
start = [start size(all_pts,1)+1 -1 1]
finish = [finish size(all_pts,1)+2 -1 1]
finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
[vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(shrunk_polytopes, starts, finishes);

start_for_reachability = start;
start_for_reachability(4) = start(3);
finish_for_reachability = finish;
finish_for_reachability(4) = finish(3);

[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start_for_reachability,finish_for_reachability)

[cost, route] = fcn_algorithm_Astar(vgraph, all_pts, start, finish, rgraph)

hold on
plot(route(:,1),route(:,2),'k','LineWidth',2);
plot(start(1),start(2),'xg');
plot(finish(1),finish(2),'xr');

nodes_vis_from_route = 0;
for i = 1:size(route,1)
    way_pt = route(i,3);
    vgraph_row = vgraph(way_pt,:);
    nodes_vis_from_way_pt = sum(vgraph_row);
    nodes_vis_from_route = nodes_vis_from_route + nodes_vis_from_way_pt;
end
nodes_reach_from_route = 0;
for i = 1:size(route,1)
    way_pt = route(i,3);
    rgraph_row = rgraph(way_pt,:);
    nodes_reach_from_way_pt = sum(rgraph_row);
    nodes_reach_from_route = nodes_reach_from_route + nodes_reach_from_way_pt;
end

x = route(:,1);
y = route(:,2);
d = diff([x(:) y(:)]);
total_length = sum(sqrt(sum(d.*d,2)));

metrics_title = sprintf('nodes visible from route waypoints: %i \n nodes visible per unit length: %.2f \n nodes reachable from route waypoints: %i \n nodes reachable per unit length: %.2f',nodes_vis_from_route,nodes_vis_from_route/total_length,nodes_reach_from_route,nodes_reach_from_route/total_length);
title(metrics_title);

all_pts = [all_pts; start; finish];
%% highlight visible nodes and reachable nodes
num_waypoints = size(route,1);
for i = 2:num_waypoints-1
    waypoint_id = route(i,3);
    visible_nodes = find(vgraph(waypoint_id,:));
    num_visible_nodes = size(visible_nodes,2);
    for j = 1:num_visible_nodes
        visible_node_id = visible_nodes(j);
        plot(all_pts(visible_node_id,1),all_pts(visible_node_id,2),'o','MarkerSize',3,'MarkerEdgeColor','r','MarkerFaceColor',[0.9290 0.6940 0.1250])
    end
end

return
max_translation_distance = 0.15;
final_time = 20;
time_space_polytopes = fcn_make_timespace_polyhedra_from_polygons(shrunk_polytopes, max_translation_distance, final_time);

time_space_polytopes = fcn_make_facets_from_verts(time_space_polytopes);
% figure; hold on; box on; title('polytopes in timespace')
% fig = gcf;

all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes);

% INTERNAL_fcn_format_timespace_plot();

% TODO @sjharnett break out this code into functions for:
% vertex interp
% line segment to facet intersection checking and vgraph creation
%
% verts (Nx3 array of verts)
% isObs (is it a size wall or polytope obstacle)
% obsID (ID for polytope obstacle it belongs to)
% list of obstacles
% each obstacle contains a verts table
% verts with the same vert ID are the same vert
% num time slices is num time stamps
% facets: 1 flat polytope at each time slice + 1 side wall per side for every 2 time steps

% for each facet, find centroid, move vertices towards centroid
% then triangulate
% then do ray triangle intersection

% verts = [1+10e-5 1 0+10e-5; 2-10e-5 1 0+10e-5;  3-10e-5 1 20-10e-5; 2+10e-5 1 20-10e-5]; % a line that translates its length in x over the course of 20 seconds
% verts = [1+eps 1 0+eps; 2-eps 1 0+eps;  3-eps 1 20-eps; 2+eps 1 20-eps]; % a line that translates its length in x over the course of 20 seconds
start = [0 1.25 0];
% finish = [1.5*ones(6,1) -1*ones(6,1) (21:2:31)']; % multiple time static finish
finish = [2 2.25 0; 2 2.25 20]; % moving finish
dt = 10;
finish = fcn_interpolate_route_in_time(finish,dt);
num_finish_pts = size(finish,1);
starts = [start(1)*ones(num_finish_pts,1) start(2)*ones(num_finish_pts,1) start(3)*ones(num_finish_pts,1)];
% either for each facet, pull each vertex towards the centroid
% or try different border options in triangle ray intersection function
% or try overlapping numerous triangles on one facet
% or see what is done for triangle mesh in demo script
% or move all vertices away from shapes by eps
% also new matlab add on needs to be updated
% figure; hold on; box on; title('surfels, start, and goals')
% fig = gcf;
% for i = 1:size(all_surfels,1)
%     X = [all_surfels(i,1), all_surfels(i,4), all_surfels(i,7)];
%     Y = [all_surfels(i,2), all_surfels(i,5), all_surfels(i,8)];
%     Z = [all_surfels(i,3), all_surfels(i,6), all_surfels(i,9)];
%     fill3(X,Y,Z,rand(3,1),'FaceAlpha',0.3);
% end
% plot3(start(1),start(2),start(3),'gx');
% plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
% INTERNAL_fcn_format_timespace_plot();

verts = [];
for i = 1:length(time_space_polytopes)
    verts_this_poly = time_space_polytopes(i).vertices;
    dense_verts_this_poly = fcn_interpolate_polytopes_in_time(verts_this_poly,dt);
    time_space_polytopes(i).dense_vertices = dense_verts_this_poly;
    verts = [verts; dense_verts_this_poly];
end


%% this code is required to vectorize the edge, triangle intersection checking
verts = verts(:,1:3);
all_pts = [verts; start; finish];

num_verts = size(verts,1);

num_pts = size(all_pts,1); % number of rows
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts

% figure; hold on; box on; title('all vertices and start and finish')
% INTERNAL_fcn_format_timespace_plot();
% plot3(start(1),start(2),start(3),'gx');
% plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
% plot3(verts(:,1),verts(:,2),verts(:,3),'cx')

speed_limit = 0.12;
vgraph = fcn_visibility_graph_3d_global(verts, start, finish, all_surfels, speed_limit);


num_starts = size(start,1);
num_finishes = size(finish,1);


start_with_ids = all_pts(num_verts+1:num_verts+num_starts,:);
finish_with_ids = all_pts(num_verts+num_starts+1:num_verts+num_starts+num_finishes,:);
all_pts_with_ids_no_start_and_fin = all_pts(1:num_verts,:);

[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph, start_with_ids, finish_with_ids);

[cost, route] = fcn_algorithm_Astar3d(vgraph, all_pts_with_ids_no_start_and_fin, start_with_ids, finish_with_ids,brgraph);
% route metrics follow
total_time = max(route(:,3));
route_x = route(:,1);
route_y = route(:,2);
route_t = route(:,3);
lengths = diff([route_x(:) route_y(:)]);
total_length = sum(sqrt(sum(lengths.*lengths,2)));
lengths_3d = diff([route_x(:) route_y(:) route_t(:)]);
total_length_3d = sum(sqrt(sum(lengths_3d.*lengths_3d,2)));

% plot path on vgraph
metrics_title = sprintf('route duration [s]: %.3f \n route length [m]: %.3f \n route length 3D: %.3f',total_time,total_length,total_length_3d);
% title(metrics_title);
% plot3(route(:,1),route(:,2),route(:,3),'-b','LineWidth',3);

% plot path on surfels
figure; hold on; box on; title(metrics_title);
plot3(route(:,1),route(:,2),route(:,3),'-b','LineWidth',3);
fig = gcf;
for i = 1:size(all_surfels,1)
    X = [all_surfels(i,1), all_surfels(i,4), all_surfels(i,7)];
    Y = [all_surfels(i,2), all_surfels(i,5), all_surfels(i,8)];
    Z = [all_surfels(i,3), all_surfels(i,6), all_surfels(i,9)];
    fill3(X,Y,Z,rand(3,1),'FaceAlpha',0.3);
end
plot3(start(1),start(2),start(3),'gx');
plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
INTERNAL_fcn_format_timespace_plot();

% end for each shape

figure; hold on; box on; title('all vertices, interpolated, and start and finish')
INTERNAL_fcn_format_timespace_plot();
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');
plot3(verts(:,1),verts(:,2),verts(:,3),'cx')

%% example of speed limit inforcement
% my_title = sprintf('example of speed limit enforcement,\n speed limit %0.1f m/s',speed_limit);
% figure; hold on; box on; title(my_title);
% INTERNAL_fcn_format_timespace_plot();
% fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b','FaceAlpha',0.3);
% fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'b','FaceAlpha',0.3);
% beg = 20;
% example_vgraph_row = vgraph(beg,:);
% cone_apex = [0 0 0];
% cone_apex = [all_pts(beg,1) all_pts(beg,2) all_pts(beg,3)];
% speed_lim_time_change = 3;
% speed_lim_dist_change = speed_limit*speed_lim_time_change;
% [X,Y,Z]=cylinder((0:0.2:1)*speed_lim_dist_change,1000);
% M=makehgtform('translate',cone_apex);
% h=surf(X,Y,speed_lim_time_change*Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.3);
% for term = 1:1:length(example_vgraph_row)
%     if example_vgraph_row(term)
%         color = 'g';
%     else
%         color = 'k';
%     end
%     plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],color,'LineWidth',2)
% end
% view([1 0 0])

route_dense = fcn_interpolate_route_in_time(route,dt);
toc
return
fcn_animate_timespace_path_plan(start, finish, time_space_polytopes, route_dense, dt, [0 1], [0 1]);

% https://www.mathworks.com/matlabcentral/fileexchange/33073-triangle-ray-intersection
% https://en.wikipedia.org/wiki/Intersection_of_a_polyhedron_with_a_line
% https://www.mathworks.com/help/matlab/visualize/multifaceted-patches.html
% at each time t, calculate P
% ir for all points on all bodies that are candidates for collision
% select one point P
% jr on body j and test if it is inside body i
% cast an infinite ray in any direction from P
% jr and compute intersections with all facets on body i
% if there are an even number of intersections, P
% jr is not inside body i
% if there are an odd number of intersections, P
% jr is inside body i
% works for concavities, holes and self-crossing and is independent of CW versus CCW boundary
% ray-facet intersection is similar to edge-facet intersection described below
% point in polygon does not always work for thin bodies (may need edge intersection)
%
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
