clear all; close all; clc
% define figure properties
opts.width      = 8;
opts.height     = 6;
opts.fontType   = 'Times';
opts.fontSize   = 9;

facets = [];

% verts (Nx3 array of verts)
% isObs (is it a size wall or polytope obstacle)
% obsID (ID for polytope obstacle it belongs to)

verts = [1 1 0; 2 1 0;  3 1 20; 2 1 20]; % a line that translates its length in x over the course of 20 seconds
verts2 = [2 2 10; 3 2 10;  4 2 30; 3 2 30]; % a line that translates its length in x over the course of 20 seconds
start = [2 0 0];
finish = [4 4 45];
figure; hold on; box on; title('surfels and line from start to goal');
fig = gcf;
% scaling
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
fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b');
fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'r');
fill3(verts2(1:3,1),verts2(1:3,2),verts2(1:3,3),'g');
fill3(verts2([1,3,4],1),verts2([1,3,4],2),verts2([1,3,4],3),'y');
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');
plot3([start(1) finish(1)],[start(2) finish(2)],[start(3) finish(3)])
% finish = start + dir
% thus dir = finish-start
[intersect, t, u, v, xcoor] = TriangleRayIntersection (start, finish-start, verts(1,:),verts(2,:),verts(3,:),'lineType','segment','border','exclusive')
[intersect2, t2, u2, v2, xcoor2] = TriangleRayIntersection (start, finish-start, verts(1,:),verts(3,:),verts(4,:),'lineType','segment')
[intersect3, t3, u3, v3, xcoor3] = TriangleRayIntersection (start, finish-start, verts2(1,:),verts2(2,:),verts2(3,:),'lineType','segment','border','exclusive')
[intersect4, t4, u4, v4, xcoor4] = TriangleRayIntersection (start, finish-start, verts2(1,:),verts2(3,:),verts2(4,:),'lineType','segment','border','exclusive')
plot3(xcoor(1),xcoor(2),xcoor(3),'cx')
plot3(xcoor3(1),xcoor3(2),xcoor3(3),'cx')
plot3(xcoor4(1),xcoor4(2),xcoor4(3),'cx')
legend('tri 1','tri 2','tri3','tri4','start','goal','','intersection')

%% this code is required to vectorize the edge, triangle intersection checking
all_pts = [verts; verts2; start; finish];
all_surfels = [verts(1,:),verts(2,:),verts(3,:);verts(1,:),verts(3,:),verts(4,:);verts2(1,:),verts2(2,:),verts2(3,:);verts2(1,:),verts2(3,:),verts2(4,:)]
figure; hold on; box on; title('all vertices and start and finish')
fig = gcf;
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');
plot3(verts(:,1),verts(:,2),verts(:,3),'cx')
plot3(verts2(:,1),verts2(:,2),verts2(:,3),'cx')

num_pts = size(all_pts,1); % number of rows
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pt_combos = nchoosek(all_pts_idx,2); % each row of this matrix is a combination of 2 point idxs

all_ray_starts = all_pts(all_pt_combos(:,1),:); % take all cols of all_pts at the row provided by the first col of all combos
all_ray_ends = all_pts(all_pt_combos(:,2),:); % take all cols of all_pts at the row provided by the second col of all combos
all_ray_dirs = all_ray_ends - all_ray_starts; % TriangleRayIntersection takes a ray direction which is end minus beginning
num_rays = size(all_ray_starts,1);
figure; hold on; box on; title('all rays casted')
fig = gcf;
% scaling
fig.Units               = 'centimeters';
fig.Position(3)         = opts.width;
fig.Position(4)         = opts.height;

% set text properties
fig.Units               = 'centimeters';
set(fig.Children, ...
    'FontName',     'Times', ...
    'FontSize',     9);

% remove unnecessary white space
set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
xlabel('x [m]')
ylabel('y [m]')
zlabel('t [s]')
for i = 1:1:num_rays
    plot3([all_ray_starts(i,1), all_ray_ends(i,1)],[all_ray_starts(i,2), all_ray_ends(i,2)],[all_ray_starts(i,3), all_ray_ends(i,3)],'LineWidth',2)
end

figure; hold on; box on; title('vgraph')
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

[intersects, ts, us, vs, xcoors] = TriangleRayIntersection (all_ray_starts_repeated, all_ray_dirs_repeated, all_surfels_repeated(:,1:3),all_surfels_repeated(:,4:6),all_surfels_repeated(:,7:9),'lineType','segment','border','exclusive');

intersects_idx = find(intersects);
for k = 1:1:length(intersects_idx)
    i = intersects_idx(k);
    plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'r','LineWidth',2)
    plot3(rmmissing(xcoors(i,1)),rmmissing(xcoors(i,2)),rmmissing(xcoors(i,3)),'cx','MarkerSize',10)
end
fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b','FaceAlpha',0.3);
fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'b','FaceAlpha',0.3);
fill3(verts2(1:3,1),verts2(1:3,2),verts2(1:3,3),'b','FaceAlpha',0.3);
fill3(verts2([1,3,4],1),verts2([1,3,4],2),verts2([1,3,4],3),'b','FaceAlpha',0.3);
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