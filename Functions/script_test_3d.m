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

verts = [1 1 0; 2 1 0;  3 1 20; 2 1 20]; % a line that translates its length in x over the course of 20 seconds
start = [2 0 0];
finish = [2*ones(6,1) 2*ones(6,1) (11:2:21)'];
starts = [2*ones(6,1) 2*ones(6,1) zeros(6,1)];

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
[intersect2, t2, u2, v2, xcoor2] = TriangleRayIntersection (starts, finish-starts, verts(1,:),verts(3,:),verts(4,:),'lineType','segment');
plot3(xcoor(1),xcoor(2),xcoor(3),'cx')
legend('tri 1','tri 2','start','goal','','intersection')
INTERNAL_fcn_format_timespace_plot();

%% interpolation code for a shape
% TODO @sjharnett when functionalizing this, add the vertex ID column but then remove it
dt = 1;
% for each shape
% get shape.verts
verts = [1 1 0 1; 2 1 0 2;  3 1 20 2; 2 1 20 1]; % a line that translates its length in x over the course of 20 seconds
% columns of verts are x,y,t,id
% for number of unique time values in verts...
unique_times = unique(verts(:,3));
num_unique_times = length(unique(verts(:,3)));

unique_verts = unique(verts(:,4));
num_unique_verts = length(unique(verts(:,4)));

dense_times = [];
for i = 2:1:num_unique_times
    new_times = unique_times(i-1):dt:unique_times(i);
    dense_times = [dense_times; new_times'];
end
dense_times = unique(dense_times);
num_dense_times = length(dense_times);

for i = 1:1:num_unique_verts
    this_vert_id = unique_verts(i);
    this_vert_rows = find(verts(:,4)==this_vert_id);
    this_vert_x = verts(this_vert_rows,1);
    this_vert_y = verts(this_vert_rows,2);
    this_vert_t = verts(this_vert_rows,3);

    this_vert_dense_x = interp1(this_vert_t,this_vert_x,dense_times);
    this_vert_dense_y = interp1(this_vert_t,this_vert_y,dense_times);
    this_vert_id_repeated = ones(num_dense_times,1);
    verts = [verts; this_vert_dense_x this_vert_dense_y dense_times this_vert_id_repeated];
end

% will need to remove duplicate rows

% end for each shape

%% this code is required to vectorize the edge, triangle intersection checking
verts = verts(:,1:3);
all_pts = [verts; start; finish];
all_surfels = [verts(1,:),verts(2,:),verts(3,:);verts(1,:),verts(3,:),verts(4,:)];
figure; hold on; box on; title('all vertices and start and finish')
INTERNAL_fcn_format_timespace_plot();
plot3(start(1),start(2),start(3),'gx');
plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
plot3(verts(:,1),verts(:,2),verts(:,3),'cx')

num_pts = size(all_pts,1); % number of rows
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts
all_pt_combos = nchoosek(all_pts_idx,2); % each row of this matrix is a combination of 2 point idxs

all_ray_starts = all_pts(all_pt_combos(:,1),:); % take all cols of all_pts at the row provided by the first col of all combos
all_ray_ends = all_pts(all_pt_combos(:,2),:); % take all cols of all_pts at the row provided by the second col of all combos
all_ray_dirs = all_ray_ends - all_ray_starts; % TriangleRayIntersection takes a ray direction which is end minus beginning
num_rays = size(all_ray_starts,1);

figure; hold on; box on; title('all rays casted')
INTERNAL_fcn_format_timespace_plot();
for i = 1:1:num_rays
    plot3([all_ray_starts(i,1), all_ray_ends(i,1)],[all_ray_starts(i,2), all_ray_ends(i,2)],[all_ray_starts(i,3), all_ray_ends(i,3)],'LineWidth',2)
end

figure; hold on; box on; title('vgraph')
INTERNAL_fcn_format_timespace_plot();
for i = 1:1:num_rays
    plot3([all_ray_starts(i,1), all_ray_ends(i,1)],[all_ray_starts(i,2), all_ray_ends(i,2)],[all_ray_starts(i,3), all_ray_ends(i,3)],'g','LineWidth',2)
end

num_surfels = size(all_surfels,1);
all_ray_idx = 1:1:num_rays;
all_surfel_idx = 1:1:num_surfels;
all_surfel_ray_combos = table2array(combinations(all_ray_idx,all_surfel_idx));

all_ray_starts_repeated = all_ray_starts(all_surfel_ray_combos(:,1),:);
all_ray_ends_repeated = all_ray_ends(all_surfel_ray_combos(:,1),:);
all_ray_dirs_repeated = all_ray_dirs(all_surfel_ray_combos(:,1),:);
all_surfels_repeated = all_surfels(all_surfel_ray_combos(:,2),:);

[intersects, ts, us, vs, xcoors] = TriangleRayIntersection (all_ray_starts_repeated(:,1:3), all_ray_dirs_repeated(:,1:3), all_surfels_repeated(:,1:3),all_surfels_repeated(:,4:6),all_surfels_repeated(:,7:9),'lineType','segment','border','exclusive');

vgraph = ones(num_pts); % initialize vgraph as zero
intersects_idx = find(intersects);
for k = 1:1:length(intersects_idx)
    i = intersects_idx(k);
    plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'r','LineWidth',2)
    plot3(rmmissing(xcoors(i,1)),rmmissing(xcoors(i,2)),rmmissing(xcoors(i,3)),'cx','MarkerSize',10)
    start_id = all_ray_starts_repeated(i,4);
    end_id = all_ray_ends_repeated(i,4);
    vgraph(start_id,end_id) = 0;
    vgraph(end_id,start_id) = 0;
end
fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b','FaceAlpha',0.3);
fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'b','FaceAlpha',0.3);
%% discard rays that are too high in velocity
% ray slope is rise over run
% rise is delta t
all_delta_ts = abs(all_ray_ends_repeated(:,3) - all_ray_starts_repeated(:,3));
% run is change in total length regardless of x or y
all_delta_xs = all_ray_ends_repeated(:,1) - all_ray_starts_repeated(:,1);
all_delta_ys = all_ray_ends_repeated(:,2) - all_ray_starts_repeated(:,2);
all_delta_dist = (all_delta_xs.^2 + all_delta_ys.^2).^0.5;
all_slopes = all_delta_ts./all_delta_dist;

speed_violation_idx = find(all_slopes == 0);
for l = 1:1:length(speed_violation_idx)
    i = speed_violation_idx(l);
    plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'k','LineWidth',2)
    start_id = all_ray_starts_repeated(i,4);
    end_id = all_ray_ends_repeated(i,4);
    vgraph(start_id,end_id) = 0;
    vgraph(end_id,start_id) = 0;
end

[cost, route] = fcn_algorithm_Astar3d(vgraph, all_pts(1:end-7,:), all_pts(end-6,:), all_pts(end-5:end,:));
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


%% interpolation code for a shape
dt = 1;
% for each shape
% get shape.verts
verts = [1 1 0 1; 2 1 0 2;  3 1 20 2; 2 1 20 1]; % a line that translates its length in x over the course of 20 seconds
% columns of verts are x,y,t,id
% for number of unique time values in verts...
unique_times = unique(verts(:,3));
num_unique_times = length(unique(verts(:,3)));

unique_verts = unique(verts(:,4));
num_unique_verts = length(unique(verts(:,4)));

dense_times = [];
for i = 2:1:num_unique_times
    new_times = unique_times(i-1):dt:unique_times(i);
    dense_times = [dense_times; new_times'];
end
dense_times = unique(dense_times);
num_dense_times = length(dense_times);

for i = 1:1:num_unique_verts
    this_vert_id = unique_verts(i);
    this_vert_rows = find(verts(:,4)==this_vert_id);
    this_vert_x = verts(this_vert_rows,1);
    this_vert_y = verts(this_vert_rows,2);
    this_vert_t = verts(this_vert_rows,3);

    this_vert_dense_x = interp1(this_vert_t,this_vert_x,dense_times);
    this_vert_dense_y = interp1(this_vert_t,this_vert_y,dense_times);
    this_vert_id_repeated = this_vert_id*ones(num_dense_times,1);
    verts = [verts; this_vert_dense_x this_vert_dense_y dense_times this_vert_id_repeated];
end

% will need to remove duplicate rows

% end for each shape

figure; hold on; box on; title('all vertices, interpolated, and start and finish')
INTERNAL_fcn_format_timespace_plot();
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');
plot3(verts(:,1),verts(:,2),verts(:,3),'cx')
return
close all;
%% create an animation for moving line
for i = 1:num_dense_times
    hold on; box on; title(sprintf('animation of \n moving two point wall shown at 10x speed'))
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
    set(gcf,'color','white')
    % set text properties
    set(fig.Children, ...
        'FontName',     'Times', ...
        'FontSize',     9);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [m]')
    ylabel('y [m]')
    ylim([0 2])
    xlim([0 4])
    % for each poly
    % this polys verts
    % need to get x y and z coords at this time
    cur_time = dense_times(i);
    cur_time_locations = find(verts(:,3) == cur_time);
    cur_x = verts(cur_time_locations,1);
    cur_y = verts(cur_time_locations,2);
    fill(cur_x,cur_y,'b','FaceAlpha',0.2);
    if i == 1
        gif('moving_wall_demo.gif','LoopCount',1,'DelayTime',dt/10)
    else
        gif
    end
    delete(gca)
end


%% interpolation code for a route
dt = 1;
% columns of verts are x,y,t,id
% for number of unique time values in verts...
unique_times = unique(route(:,3));
num_unique_times = length(unique(route(:,3)));

num_route_verts = size(route,1);

dense_times = [];
for i = 2:1:num_unique_times
    new_times = unique_times(i-1):dt:unique_times(i);
    dense_times = [dense_times; new_times'];
end
dense_times = unique(dense_times);
num_dense_times = length(dense_times);

try
    route_dense_x = interp1(route(:,3),route(:,1),dense_times);
catch
    route_dense_x = route(1,1)*ones(size(dense_times,1),size(dense_times,2));
end
try
    route_dense_y = interp1(route(:,3),route(:,2),dense_times);
catch
    route_dense_y = route(1,2)*ones(size(dense_times,1),size(dense_times,2));
end

route_dense = [route_dense_x route_dense_y dense_times];

close all;
%% create an animation for moving line
for i = 1:num_dense_times
    hold on; box on; title(sprintf('animation of routing around \n moving two point wall shown at 10x speed'))
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
    set(gcf,'color','white')
    % set text properties
    set(fig.Children, ...
        'FontName',     'Times', ...
        'FontSize',     9);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [m]')
    ylabel('y [m]')
    ylim([-1 3])
    xlim([0 4])
    % for each poly
    % this polys verts
    % need to get x y and z coords at this time
    cur_time = dense_times(i);
    cur_time_locations = find(verts(:,3) == cur_time);
    cur_x = verts(cur_time_locations,1);
    cur_y = verts(cur_time_locations,2);

    cur_route_idx = find(route_dense(:,3) == cur_time);

    p_route = plot(route_dense(1:cur_route_idx,1),route_dense(1:cur_route_idx,2),'-k','LineWidth',2);
    p_pose = plot(route_dense(cur_route_idx,1),route_dense(cur_route_idx,2),'xk','MarkerSize',2)
    p_start = plot(start(1),start(2),'gx');
    p_finish = plot(finish(1),finish(2),'rx');
    fill(cur_x,cur_y,'b','FaceAlpha',0.2);
    if i == 1
        gif('moving_wall_with_path.gif','LoopCount',1,'DelayTime',dt/10)
    else
        gif
    end
    delete(gca)
    delete(p_route)
    delete(p_pose)
    delete(p_start)
    delete(p_finish)
end
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
