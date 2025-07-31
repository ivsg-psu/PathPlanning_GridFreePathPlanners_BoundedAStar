% script_test_fcn_Visibility_clearAndBlockedPointsGlobal
% Tests: fcn_Visibility_clearAndBlockedPointsGlobal

%
% REVISION HISTORY:
%
% 2022_10_28 by S. Harnett
% -- first write of script
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
%%%%%%%%%%%%%%§

clear
clc
close all

%% add necessary directories
addpath([pwd '\..\Example_Map_Generation_Code'])
addpath([pwd '\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\..\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

flag_do_plot = 1;
%% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(convex_polytope);


% plot the map
if flag_do_plot
    fig = 111; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [-3 3 -3 3]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('valid edges')
    fig = 112; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [-3 3 -3 3]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('blocked edges')
    fig = 113; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [-3 3 -3 3]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('all edges')
end

%% generate all_pts table
start = [-2.5, 1];
finish = start + [4 0];
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
    polytopes(poly).distances = sum((polytopes(poly).vertices(1:end-1,:) - polytopes(poly).vertices(2:end,:)).^2,2).^0.5;
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
all_pts = [all_pts; start, point_tot + 1, -1, 0; finish, point_tot + 2, -1, 0];
% hardcode what we expect the visibility graph to be for the convex polytope example
convex_obstacle_vgraph = [1 1 0 0 1 0 1;
                          1 1 1 0 0 0 1;
                          0 1 1 1 0 1 1;
                          0 0 1 1 1 1 0;
                          1 0 0 1 1 1 0;
                          0 0 1 1 1 1 0;
                          1 1 1 0 0 0 1];
% test without concavity flag
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
assert(isequal(vgraph,convex_obstacle_vgraph));
% test with concavity flag explicitly off
[vgraph, visibility_results] =fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,0);
assert(isequal(vgraph,convex_obstacle_vgraph));
% test with invalid concavity flag
try
    [vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,10);
catch ME
    assert(strcmp(ME.message,'optional argument is the is_concave flag and can either be 1 or 0'));
end
% test with concavity flag explicitly on
tic
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,1);
toc
assert(isequal(vgraph,convex_obstacle_vgraph));
% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                figure(111)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
                figure(113)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
            end
            if vgraph(i,j) == 0
                figure(112)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--r','LineWidth',2)
                figure(113)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--r','LineWidth',2)
            end
        end
    end
end
%% nonconvex polytope
convex_polytope(1).vertices = [0 0; 1 1; 0.5, 2.5; -2, 2.5; -1 2; -2 1; -1 0; 0 0];
polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(convex_polytope,1);

% plot the map
if flag_do_plot
    fig = 121; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [-3 3 -3 3]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('valid edges')
    fig = 122; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [-3 3 -3 3]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('blocked edges')
    fig = 123; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [-3 3 -3 3]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('all edges')
end

%% generate all_pts table
start = [-2.5, 1];
finish = start + [4 0];
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
    polytopes(poly).distances = sum((polytopes(poly).vertices(1:end-1,:) - polytopes(poly).vertices(2:end,:)).^2,2).^0.5;
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
all_pts = [all_pts; start, point_tot + 1, -1, 0; finish, point_tot + 2, -1, 0];
% hard code what we expect the concave obstacle vgraph to be when it is treated as a convex obstalce in vgraph calculation
concave_obstacle_vgraph_sub_optimal_result = [1 1 0 0 0 0 1 0 1;
                                              1 1 1 0 0 0 0 0 1;
                                              0 1 1 1 0 0 0 0 1;
                                              0 0 1 1 1 0 0 1 0;
                                              0 0 0 1 1 1 0 1 0;
                                              0 0 0 0 1 1 1 1 0;
                                              1 0 0 0 0 1 1 1 0;
                                              0 0 0 1 1 1 1 1 0;
                                              1 1 1 0 0 0 0 0 1];
concave_obstacle_vgraph_optimal_result = [1 1 0 0 0 0 1 0 1;
                                          1 1 1 0 0 0 0 0 1;
                                          0 1 1 1 0 0 0 0 1;
                                          0 0 1 1 1 1 0 1 0;
                                          0 0 0 1 1 1 0 1 0;
                                          0 0 0 1 1 1 1 1 0;
                                          1 0 0 0 0 1 1 1 0;
                                          0 0 0 1 1 1 1 1 0;
                                          1 1 1 0 0 0 0 0 1];

% test without concavity flag
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
assert(isequal(vgraph,concave_obstacle_vgraph_sub_optimal_result));
% test with concavity flag explicitly off
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,0);
assert(isequal(vgraph,concave_obstacle_vgraph_sub_optimal_result));
num_edges_in_suboptimal_case = sum(sum(vgraph));
% test with invalid concavity flag
try
    [vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,10);
catch ME
    assert(strcmp(ME.message,'optional argument is the is_concave flag and can either be 1 or 0'));
end
% test with appropriately set concavity flag
tic
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,1);
toc
assert(isequal(vgraph,concave_obstacle_vgraph_optimal_result));
num_edges_in_optimal_case = sum(sum(vgraph));
% sanity check that there are more edges in the optimal case (i.e. that the suboptimal case is conservative)
assert(num_edges_in_optimal_case>num_edges_in_suboptimal_case);
% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                figure(121)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
                figure(123)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
            end
            if vgraph(i,j) == 0
                figure(123)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--r','LineWidth',2)
                figure(122)
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--r','LineWidth',2)
            end
        end
    end
end

%% generate map
Halton_seed = 10;
low_pt = 1+Halton_seed; high_pt = 11+Halton_seed; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
% shink the polytopes so that they are no longer tiled
gap_size = 0.025; % desired average maximum radius
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

%% generate all_pts table
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
    polytopes(poly).distances = sum((polytopes(poly).vertices(1:end-1,:) - polytopes(poly).vertices(2:end,:)).^2,2).^0.5;   
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]


%% calculate vibility graph
tic
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
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

%% test zero gap case
% generate map
polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
% shink the polytopes so that they are no longer tiled
gap_size = 0.01; % desired average maximum radius
if gap_size ~=0
    polytopes = fcn_MapGen_polytopesShrinkFromEdges(polytopes,gap_size);
end

% plot the map
if flag_do_plot
    fig = 199; % figure to plot on
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

%% generate all_pts table
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
    polytopes(poly).distances = sum((polytopes(poly).vertices(1:end-1,:) - polytopes(poly).vertices(2:end,:)).^2,2).^0.5;
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end
obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]

%% calculate vibility graph
tic
vgraph = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
toc
deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
% plot visibility graph edges
if flag_do_plot && gap_size ==0
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([deduped_pts(i).x,deduped_pts(j).x],[deduped_pts(i).y,deduped_pts(j).y],'--g','LineWidth',1)
            end
        end
    end
end
if flag_do_plot && gap_size ~=0
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
            end
        end
    end
end

% broken test for 0 gap size special case that is not implemented
% % generate map
% polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
% % shink the polytopes so that they are no longer tiled
% gap_size = 0; % desired average maximum radius
% if gap_size ~=0
%     polytopes = fcn_MapGen_polytopesShrinkFromEdges(polytopes,gap_size);
% end
%
% % plot the map
% if flag_do_plot
%     fig = 299; % figure to plot on
%     line_spec = 'b-'; % edge line plotting
%     line_width = 2; % linewidth of the edge
%     axes_limits = [0 1 0 1]; % x and y axes limits
%     axis_style = 'square'; % plot axes style
%     fcn_plot_polytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
%     hold on
%     box on
%     xlabel('x [km]')
%     ylabel('y [km]')
% end
%
% %% generate all_pts table
% point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes
% beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
% curpt = 0;
% for poly = 1:size(polytopes,2) % check each polytope
%     verts = unique(polytopes(poly).vertices,'stable','rows');
%     num_verts = size(verts,1);
%     polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
%     polytopes(poly).xv = verts(:,1)';
%     polytopes(poly).yv = verts(:,2)';
%     polytopes(poly).vertices = [verts; verts(1,:)];
%     polytopes(poly).distances = sum((polytopes(poly).vertices(1:end-1,:) - polytopes(poly).vertices(2:end,:)).^2,2).^0.5;
%     beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
%     curpt = curpt+num_verts;
%     polytopes(poly).perimeter = sum(polytopes(poly).distances);
% end
% obs_id = [polytopes.obs_id];
% point_tot = length([polytopes.xv]); % need to recheck total points
% beg_end = beg_end(1:point_tot); % remove any extra points
%
% all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
%
% %% calculate vibility graph
% tic
% vgraph = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
% toc
% deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
% % plot visibility graph edges
% if flag_do_plot && gap_size ==0
%     for i = 1:size(vgraph,1)
%         for j = 1:size(vgraph,1)
%             if vgraph(i,j) == 1
%                 plot([deduped_pts(i).x,deduped_pts(j).x],[deduped_pts(i).y,deduped_pts(j).y],'--g','LineWidth',1)
%             end
%         end
%     end
% end
% if flag_do_plot && gap_size ~=0
%     for i = 1:size(vgraph,1)
%         for j = 1:size(vgraph,1)
%             if vgraph(i,j) == 1
%                 plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
%             end
%         end
%     end
% end
%
