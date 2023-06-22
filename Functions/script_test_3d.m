clear; close all; clc

addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'

addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\Desktop\gif\gif'

facets = [];
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

verts = [1 1 0; 2 1 0;  3 1 20; 2 1 20]; % a line that translates its length in x over the course of 20 seconds
% verts = [1+10e-5 1 0+10e-5; 2-10e-5 1 0+10e-5;  3-10e-5 1 20-10e-5; 2+10e-5 1 20-10e-5]; % a line that translates its length in x over the course of 20 seconds
% verts = [1+eps 1 0+eps; 2-eps 1 0+eps;  3-eps 1 20-eps; 2+eps 1 20-eps]; % a line that translates its length in x over the course of 20 seconds
start = [2 0 0];
finish = [2*ones(6,1) 2*ones(6,1) (11:2:21)']; % multiple time static finish
% finish = [2 2 11; 1.25 1.25 21]; % moving finish
dt = 1;
finish = fcn_interpolate_route_in_time(finish,dt);
num_finish_pts = size(finish,1);
starts = [2*ones(num_finish_pts,1) 2*ones(num_finish_pts,1) zeros(num_finish_pts,1)];
% either for each facet, pull each vertex towards the centroid
% or try different border options in triangle ray intersection function
% or try overlapping numerous triangles on one facet
% or see what is done for triangle mesh in demo script
% or move all vertices away from shapes by eps
% also new matlab add on needs to be updated
figure; hold on; box on; title('surfels and line from start to goal')
fig = gcf;
fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b');
fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'r');
plot3(start(1),start(2),start(3),'gx');
plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
plot3([start(1) finish(1)],[start(2) finish(2)],[start(3) finish(3)])
% finish = start + dir
% thus dir = finish-start
[intersect, t, u, v, xcoor] = TriangleRayIntersection (starts, finish-starts, verts(1,:),verts(2,:),verts(3,:),'lineType','segment','border','exclusive');
[intersect2, t2, u2, v2, xcoor2] = TriangleRayIntersection (starts, finish-starts, verts(1,:),verts(3,:),verts(4,:),'lineType','segment','border','exclusive');
plot3(xcoor(1),xcoor(2),xcoor(3),'cx')
legend('tri 1','tri 2','start','goal','','intersection')
INTERNAL_fcn_format_timespace_plot();

verts_orig = verts;
verts = [1 1 0 1; 2 1 0 2;  3 1 20 2; 2 1 20 1]; % a line that translates its length in x over the course of 20 seconds
verts = fcn_interpolate_polytopes_in_time(verts,dt);

%% this code is required to vectorize the edge, triangle intersection checking
verts = verts(:,1:3);
all_pts = [verts; start; finish];

num_verts = size(verts,1);

num_pts = size(all_pts,1); % number of rows
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts
all_surfels = [verts_orig(1,:),verts_orig(2,:),verts_orig(3,:);verts_orig(1,:),verts_orig(3,:),verts_orig(4,:)];
figure; hold on; box on; title('all vertices and start and finish')
INTERNAL_fcn_format_timespace_plot();
plot3(start(1),start(2),start(3),'gx');
plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
plot3(verts(:,1),verts(:,2),verts(:,3),'cx')

speed_limit = 0.01;
vgraph = fcn_visibility_graph_3d_global(verts, start, finish, all_surfels, speed_limit);
start = all_pts(num_verts+1,:);
finish = all_pts(num_verts+2:end,:);
[is_reachable, num_steps] = fcn_check_reachability(vgraph,start,finish)

[cost, route] = fcn_algorithm_Astar3d(vgraph, all_pts(1:num_verts,:), all_pts(num_verts+1,:), all_pts(num_verts+2:end,:));
% route metrics follow
total_time = max(route(:,3));
route_x = route(:,1);
route_y = route(:,2);
route_t = route(:,3);
lengths = diff([route_x(:) route_y(:)]);
total_length = sum(sqrt(sum(lengths.*lengths,2)));
lengths_3d = diff([route_x(:) route_y(:) route_t(:)]);
total_length_3d = sum(sqrt(sum(lengths_3d.*lengths_3d,2)));

metrics_title = sprintf('route duration [s]: %.3f \n route length [m]: %.3f \n route length 3D: %.3f',total_time,total_length,total_length_3d);
title(metrics_title);
plot3(route(:,1),route(:,2),route(:,3),'-b','LineWidth',3);

% end for each shape

figure; hold on; box on; title('all vertices, interpolated, and start and finish')
INTERNAL_fcn_format_timespace_plot();
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');
plot3(verts(:,1),verts(:,2),verts(:,3),'cx')

%% example of speed limit inforcement
my_title = sprintf('example of speed limit enforcement,\n speed limit %0.1f m/s',speed_limit);
figure; hold on; box on; title(my_title);
INTERNAL_fcn_format_timespace_plot();
fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b','FaceAlpha',0.3);
fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'b','FaceAlpha',0.3);
beg = 20;
example_vgraph_row = vgraph(beg,:);
cone_apex = [0 0 0];
cone_apex = [all_pts(beg,1) all_pts(beg,2) all_pts(beg,3)];
speed_lim_time_change = 3;
speed_lim_dist_change = speed_limit*speed_lim_time_change;
[X,Y,Z]=cylinder((0:0.2:1)*speed_lim_dist_change,1000);
M=makehgtform('translate',cone_apex);
h=surf(X,Y,speed_lim_time_change*Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.3);
for term = 1:1:length(example_vgraph_row)
    if example_vgraph_row(term)
        color = 'g';
    else
        color = 'k';
    end
    plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],color,'LineWidth',2)
end
view([1 0 0])

route_dense = fcn_interpolate_route_in_time(route,dt);
return
fcn_animate_timespace_path_plan(start, finish, verts, route_dense, dt);

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
