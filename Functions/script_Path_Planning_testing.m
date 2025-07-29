% script_Path_Planning_testing.m
% Written by: S. Tau, sat5340@psu.edu
% Edits by: S. Brennan, sbrennan@psu.edu
%
% Details: This is a script to demonstrate all the different function
% within the path planning folder. Assumes that you are working out of the
% folder "Path_Planning", and that this subfolder follows the structure of
% the Voronoi folders as a whole (see Box example). If you rename folders,
% you will have to change the path details within the script below.
%
% Revisions:
%   2020_02_11  -- first write of code to demonstrate mapping functionality
%   2020_03_26  -- edits made to comment and cleanup the code
%   2025_07_08 - K. Hayes, kxh1031@psu.edu
%   -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%      with vector sum method
%   -- Added planner_mode input to Astar setup function for compatibility
%      with newer function versions


%% Prep the workspace
close all
clear
clc
% Set the random number seed so that the results are consistent throughout.
rng(678);

%% add necessary scripts so that scripts can be accessed
cur_path = pwd;
main_folder = '!Voronoi Tiling Obstacles - Organized';
parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
addpath([parent_dir '\' main_folder '\General_Calculation'])
addpath([parent_dir '\' main_folder '\Plotting'])
addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
addpath([parent_dir '\' main_folder '\Map_Generation\polytope_editing'])
addpath([parent_dir '\' main_folder '\Map_Generation\polytope_generation'])
addpath([parent_dir '\' main_folder '\Path_Planning\visibility'])
addpath([parent_dir '\' main_folder '\Path_Planning\bounding_ellipse'])
addpath([parent_dir '\' main_folder '\Path_Planning\algorithm_setup'])
addpath([parent_dir '\' main_folder '\Path_Planning\algorithm'])

%% generate the map
% Details on what each of these functions do can be found in the
% Map_Generation folder in scrpt_Map_Generation_testing.m.
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[],[100 100],-1);
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0 0 100 100], (-1));
shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);

%% determine vertice information
% Determine the obstacle number for each vertex and whether or not each
% vertex is the first or last one in the set for each obstacle.
xvert = [shrunk_polytopes.xv]; % pull the x and y vertices
yvert = [shrunk_polytopes.yv];
point_tot = length(xvert); % determine the total number of vertices
% initialize the variables to store the vertex info
beg_end = zeros(point_tot,1); 
obs_id = zeros(point_tot,1);
curpt = 0;
for poly = 1:size(shrunk_polytopes,2) % check each polytope
    verts = length(shrunk_polytopes(poly).xv); % verts for this polytope
    % obs_id is the same for every vertex on a single polytope
    obs_id(curpt+1:curpt+verts) = ones(verts,1)*poly; 
    % the first and last vertices are marked with 1 and all others are 0
    beg_end([curpt+1,curpt+verts]) = 1; 
    curpt = curpt+verts; % shift the index variable
end

%% setup the remain variables need for path planning
% Store all the point information 
% [x-vertices, y-vertices, vertex id, obstacle id, beginning or end indicator]
all_pts = [xvert' yvert' [1:length(xvert)]' obs_id beg_end];
% The bound points indicates which points are considered at each step of
% the path planning process. Here we chose all the points within the map,
% but a subset of these points could be used if desired
bound_pts = all_pts;
% Choose the starting (A) and finishing (B) points
A.x = 0; A.y = 50; B.x = 100; B.y = 50;
% Add a vertex id, obstacle id, and b/e indicator to the start point to
% format it the same as the other points
start = [A.x A.y point_tot+1 -1 0];
% Add the same thing for finish. Additionally, add the remainder of the
% points to the finish variable to indicate that these are finishing points
% to intermediate paths.
finish = [[B.x; xvert'] [B.y; yvert'] [point_tot+2;[1:point_tot]']...
    [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];


%% find clear and blocked points
% Determine which (if any) finish points are visible from the start point.
% Note this function also returns intersection information
% (D,di,dj,num_int) and potentially intersecting line information
% (xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ) for use in later functions.
[clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_visibility_clear_and_blocked_points(shrunk_polytopes,start,finish);
% plot the results showing which points are clear (green circles) and which
% points are not (red x's) from the starting point (black x)
fcn_plot_polytopes(shrunk_polytopes,9999,'b-',2,[0 100 0 100],'square');
plot([start(1) finish(1,1)],[start(2) finish(1,2)],'kx','linewidth',1)
plot(clear_pts(:,1),clear_pts(:,2),'go','linewidth',1)
plot(blocked_pts(:,1),blocked_pts(:,2),'rx','linewidth',1)

%% find the intersecting points along a straight path 
% Determine which obstacles are intersected and at what points (if any) on
% a straight line from the start point to each potential finish point.
% xings is a structure of the points intersected, the index of the lines
% that intersect, and the obstacles intersected.
xings=fcn_visibility_line_polytope_intersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,shrunk_polytopes);
% Here we pull the obstacles intersected on a straigh line between the
% start and the goal point, since these will be used in the next step.
xing_polytopes = shrunk_polytopes(xings(1).obstacles);
% plot the results with the intersected polytopes in green and the
% intersection points plotted as red x's
fig=fcn_plot_polytopes(shrunk_polytopes,9998,'b-',2,[0 100 0 100],'square');
fcn_plot_polytopes(xing_polytopes,fig,'g-',2,[0 100 0 100],'square');
plot([start(1) finish(1,1)],[start(2) finish(1,2)],'kx','linewidth',1)
plot([start(1) finish(1,1)],[start(2) finish(1,2)],'k--','linewidth',1)
plot(xings(1).points(:,1),xings(1).points(:,2),'rx','linewidth',2)

%% determine the distance of the minimum perimeter path
% Here we take the intersecting points of each polytope and determine the
% shortest path around the perimeter of the polytope to reach the other
% intersecting point (if two exist). We then add the distances for each
% polytope perimeter path and the intermediate distances between the
% polytopes and the start and goal point to find a worst case scenario path
% length for an intelligent path planner. This is the worst case because we
% know we can choose this path if all other paths fail.
max_dist=fcn_bounding_ellipse_min_perimeter_path(xing_polytopes,xings,start,finish);
disp(['Minimum perimeter path distance: ' num2str(max_dist)]);

%% determine the polytopes bound by an ellipse bounding box
% We can use the worst case distance from above as a bound for how far we
% need to search within the space. For instance, if the worse case path
% length is only 110 m then there is no point in considering a point that
% would require a minimum path length of 115 m to reach and then reach the
% goal. This limit creates an elliptical boundary, but for computational
% purposes, we use a rectangular bounding box that the ellipse is
% circumscribed within.
% Calculate the straight-line distance from start to goal
straight = sum((start(1:2) - finish(1:2)).^2,2).^0.5;
% Use the max_dist to determine how far to offset the edges of the
% rectangular bounding box in the perpendicular and parallel directions in
% order to circumscribe the ellipse, using triangular geometry
% (perpendicular) and straight line distance (parallel).
perp_offset = ones(1,2)*sqrt((max_dist/2)^2 - (straight/2)^2); 
para_offset = ones(1,2)*(max_dist - straight)/2;
% This function determines the points that are bound within the bounding
% box and any polytopes associated with those points. It also returns the
% corners of the bounding box.
[bound_polytopes,bound_box,bound_pts]=...
    fcn_bounding_ellipse_polytope_bounding_box...
    (start,finish,shrunk_polytopes,all_pts,bound_pts,perp_offset,para_offset);
% plot the results with the bound polytopes plotted in red and the bound
% points plotted as green circles.
fig=fcn_plot_polytopes(shrunk_polytopes,9997,'b-',2,[-6 106 0 100],'square');
plot([start(1) finish(1,1)],[start(2) finish(1,2)],'k--','linewidth',2)
plot([start(1) finish(1,1)],[start(2) finish(1,2)],'kx','linewidth',2)
fcn_plot_polytopes(bound_polytopes,fig,'r-',2);
plot([bound_box(:,1); bound_box(1,1)], [bound_box(:,2); bound_box(1,2)],'k--','linewidth',2)
plot(bound_pts(:,1),bound_pts(:,2),'go','linewidth',1)

%% plan the path using Bound Astar
% This function does the setup steps above and then iteratively repeats the
% process above and at the end of each step performs one step in the Astar 
% algorithm to choose the starting point for the next iteration. This 
% process is repeated until there are no more polytopes between the chosen 
% start point and the goal point. path is the series of points used for the
% ideal path and cost is the distance to travel along that path.
planner_mode = 'through at vertices'
[path,cost] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes...
    (shrunk_polytopes,A,B,planner_mode);
% plot the results showing the ideal path in black
fcn_plot_polytopes(shrunk_polytopes,9996,'b-',2,[0 100 0 100],'square');
plot(path(:,1),path(:,2),'k-','linewidth',2)
plot([start(1) finish(1,1)],[start(2) finish(1,2)],'kx','linewidth',2)
disp(['Path cost: ' num2str(cost)])
