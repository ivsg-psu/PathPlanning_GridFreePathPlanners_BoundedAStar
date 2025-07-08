clear
clc
close all

%% add necessary directories
addpath([pwd '\..\Example_Map_Generation_Code'])
addpath([pwd '\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\..\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

%% generate map
% generate Voronoi tiling from Halton points
low_pt = 1; high_pt = 100; % range of Halton points to use to generate the tiling
tiled_polytopes_legacy = fcn_polytope_generation_halton_voronoi_tiling(low_pt,high_pt);
tiled_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
% remove the edge polytope that extend past the high and low points
% shink the polytopes so that they are no longer tiled
des_radius = 0.05; % desired average maximum radius
sigma_radius = 0.002; % desired standard deviation in maximum radii
min_rad = 0.0001; % minimum possible maximum radius for any obstacle
[shrunk_polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes,des_radius,sigma_radius,min_rad);

% plot the map
fig = 99; % figure to plot on
line_spec = 'b-'; % edge line plotting
line_width = 2; % linewidth of the edge
axes_limits = [0 1 0 1]; % x and y axes limits
axis_style = 'square'; % plot axes style
fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);

%% plan path
% starting (A) and finish (B) coordinates
A.x = 0; A.y = 0.5; B.x = 1; B.y = 0.5;
[path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B,'through or around');
% path: series of points [x y point_id obs_id beg_end]
% cost: path length
% err: marker indicating if there was an error in setup (1) or not (0)

% plot path
plot(path(:,1),path(:,2),'k-','linewidth',2)
plot(A.x, A.y, 'gx','linewidth',2)
plot(B.x, B.y, 'rx','linewidth',2)


%% info needed for further work
% gather data on all the points
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

appex_x = zeros(size(path,1)-2,3);
appex_y = appex_x;
prev_pt = [A.x A.y];
for app = 1:size(appex_x,1)
    pt = path(app+1,:);
    appex_x(app,1) = pt(1);
    appex_y(app,1) = pt(2);
    if pt(5) == 1
        obstacle = pt(4);
        other_beg_end_pt = all_pts(((all_pts(:,4)==obstacle).*(all_pts(:,5)==1).*(all_pts(:,3)~=pt(3)))==1,:);
        if other_beg_end_pt(3) > pt(3)
            other_pt = all_pts(pt(3)+1,1:2);
        else % pt(3) > other_beg_end_pt(3)
            other_pt = all_pts(pt(3)-1,1:2);
        end
        dist = sum((ones(2,1)*prev_pt - [other_beg_end_pt(1:2); other_pt]).^2,2).^0.5;
        if dist(1) < dist(2) % other_pt farther
            appex_x(app,2:3) = [other_beg_end_pt(1), other_pt(1)];
            appex_y(app,2:3) = [other_beg_end_pt(2), other_pt(2)];
        else % other_pt closer
            appex_x(app,2:3) = [other_pt(1), other_beg_end_pt(1)];
            appex_y(app,2:3) = [other_pt(2), other_beg_end_pt(2)];
        end
    else
        pt1 = all_pts(pt(3)-1,1:2);
        pt2 = all_pts(pt(3)+1,1:2);
        dist = sum((ones(2,1)*prev_pt - [other_beg_end_pt(1:2); other_pt]).^2,2).^0.5;
        if dist(1) < dist(2) % pt1 closer
            appex_x(app,2:3) = [pt1(1), pt2(1)];
            appex_y(app,2:3) = [pt1(2), pt2(2)];
        else % pt1 farther
            appex_x(app,2:3) = [pt2(1), pt1(1)];
            appex_y(app,2:3) = [pt2(2), pt1(2)];
        end
    end
    prev_pt = pt(1:2);
end
plot(appex_x,appex_y,'o','linewidth',2)
% appex_x = [appex_x1 closer_x1 farther_x1; appex_x2 closer_x2 farther_x2; .... appex_xn closer_xn farther_xn]
% appex_y = [appex_y1 closer_y1 farther_y1; appex_y2 closer_y2 farther_y2; .... appex_yn closer_yn farther_yn]

%% Final Info
% A, B, appex_x, appex_y
