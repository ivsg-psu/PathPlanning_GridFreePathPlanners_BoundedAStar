% script_test_fcn_visibility_clear_and_blocked_points_global
% Tests: fcn_visibility_clear_and_blocked_points_global

%
% REVISION HISTORY:
%
% 2022_10_28 by S. Harnett
% -- first write of script
%%%%%%%%%%%%%%§

clear
clc
close all

%% add necessary directories
addpath([pwd '\..\Example_Map_Generation_Code'])
addpath([pwd '\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\..\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

flag_do_plot = 1;

%% generate polytope map
Halton_seed = 10;
low_pt = 1+Halton_seed; high_pt = 6+Halton_seed; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
% shink the polytopes so that they are no longer tiled
gap_size = 0.125; % desired average maximum radius
polytopes = fcn_MapGen_polytopesShrinkFromEdges(trim_polytopes,gap_size);

% plot the map
if flag_do_plot
    fig = 99; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
end
%% plot visibility and path for convex map
% generate all_pts table
point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes
beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
curpt = 0;
for poly = 1:size(polytopes,2) % check each polytope
    verts = unique(polytopes(poly).vertices,'stable','rows');
    num_verts = size(verts,1);
    polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
    polytopes(poly).xv = verts(:,1)';
    polytopes(poly).yv = verts(:,2)';
    polytopes(poly).vertices = [verts; verts(1,:)];
    polytopes(poly).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly).vertices(1:end-1,:),polytopes(poly).vertices(2:end,:));
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]


% calculate vibility graph
tic
[vgraph, visibility_results] = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts,all_pts);
toc
% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
            end
        end
    end
end

A.x = 0.0; A.y = 0.5; B.x = 1; B.y = 0.5;
[path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(polytopes,A,B,'legacy');
% path: series of points [x y point_id obs_id beg_end]
% cost: path length
% err: marker indicating if there was an error in setup (1) or not (0)

% plot path
if flag_do_plot
    plot(path(:,1),path(:,2),'k-','linewidth',2)
    plot(A.x, A.y, 'gx','linewidth',2)
    plot(B.x, B.y, 'rx','linewidth',2)
    my_title = sprintf('Path length [m]: %.4f',cost)
    title(my_title)
    box on
end



%% change one polytope to be concave and repeat

% make a polytope concave
% polytopes(2) = [];
polytopes(2).vertices = [0.5717    0.3059;
    0.5892    0.2072;
    0.9375    0.4032;
    0.9375    0.6633;
    0.7       0.42;
    0.7598    0.6233];
polytopes(2) = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes(2));


if flag_do_plot
    fig = 100; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
end

% generate all_pts table
point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes
beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
curpt = 0;
for poly = 1:size(polytopes,2) % check each polytope
    verts = unique(polytopes(poly).vertices,'stable','rows');
    num_verts = size(verts,1);
    polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
    polytopes(poly).xv = verts(:,1)';
    polytopes(poly).yv = verts(:,2)';
    polytopes(poly).vertices = [verts; verts(1,:)];
    polytopes(poly).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly).vertices(1:end-1,:),polytopes(poly).vertices(2:end,:));
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]


% calculate vibility graph
tic
[vgraph, visibility_results] = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts,all_pts);
toc
% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
            end
        end
    end
end

A.x = 0.0; A.y = 0.5; B.x = 1; B.y = 0.5;
[path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(polytopes,A,B,'legacy');
% path: series of points [x y point_id obs_id beg_end]
% cost: path length
% err: marker indicating if there was an error in setup (1) or not (0)

% plot path
if flag_do_plot
    plot(path(:,1),path(:,2),'k-','linewidth',2)
    plot(A.x, A.y, 'gx','linewidth',2)
    plot(B.x, B.y, 'rx','linewidth',2)
    my_title = sprintf('Path length [m]: %.4f',cost)
    title(my_title)
    box on
end

%% add a concave polytope that blocks the path
polytopes(5).vertices = [
    0.6374    0.6620;
    0.5178    0.7517;
    0.4 0.6;
    0.45 .55;
    0.4 0.4;
    0.4878    0.4096;
    0.6374    0.6620];

polytopes(5) = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes(5));


if flag_do_plot
    fig = 101; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
end

% generate all_pts table
point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes
beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
curpt = 0;
for poly = 1:size(polytopes,2) % check each polytope
    verts = unique(polytopes(poly).vertices,'stable','rows');
    num_verts = size(verts,1);
    polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
    polytopes(poly).xv = verts(:,1)';
    polytopes(poly).yv = verts(:,2)';
    polytopes(poly).vertices = [verts; verts(1,:)];
    polytopes(poly).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly).vertices(1:end-1,:),polytopes(poly).vertices(2:end,:));
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]


% calculate vibility graph
tic
[vgraph, visibility_results] = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts,all_pts);
toc
% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
            end
        end
    end
end

A.x = 0.0; A.y = 0.5; B.x = 1; B.y = 0.5;
[path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(polytopes,A,B,'legacy');
% path: series of points [x y point_id obs_id beg_end]
% cost: path length
% err: marker indicating if there was an error in setup (1) or not (0)

% plot path
if flag_do_plot
    plot(path(:,1),path(:,2),'k-','linewidth',2)
    plot(A.x, A.y, 'gx','linewidth',2)
    plot(B.x, B.y, 'rx','linewidth',2)
    my_title = sprintf('Path length [m]: %.4f',cost)
    title(my_title)
    box on
end
