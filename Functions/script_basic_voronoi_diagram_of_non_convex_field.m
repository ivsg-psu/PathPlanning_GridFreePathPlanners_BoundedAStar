clear all; close all; clc;

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;
flag_do_plot_slow = 0;

fig_num = 1;

%% load test fixture of polytopes
load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_3.mat'));
polytopes = flood_plain_3;

%% plot polytopes
figure(fig_num); hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
for j = 1:length(polytopes)
     fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
end

%% make voronoi diagram
stretch = [1 1]; % default stretch value
verts = [[polytopes.xv]', [polytopes.yv]'];
% get xmin and xmax also ymin and ymax
xmax = max(verts(:,1));
xmin = min(verts(:,1));
ymax = max(verts(:,2));
ymin = min(verts(:,2));
AABB = [xmin ymin xmax ymax];
seed_points = verts;
voronoi(seed_points(:,1),seed_points(:,2))
return
[V,C] = voronoin(seed_points);
voronoi_polytopes = fcn_MapGen_generatePolysFromTiling(seed_points,V,C,AABB, stretch);

%% plot the voronoi polytopes
fcn_MapGen_plotPolytopes(polytopes,fig_num,'g',2,[xmin xmax ymin ymax]);
plot(seed_points(:,1),seed_points(:,2),'r.','Markersize',10);
