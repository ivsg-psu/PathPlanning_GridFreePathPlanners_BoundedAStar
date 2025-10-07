% script_visibility_graph_modification
% Tests: fcn_Visibility_addObstacle
%        fcn_visibility_graph_remove_obstacle
%
% REVISION HISTORY:
%
% 2024_03 by S. Harnett
% -- first write of script
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
% 2025_10_07 by S. Brennan, sbrennan@psu.edu
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal
% -- removed calls to fcn_MapGen_fillPolytopeFieldsFromVertices,
%    % replaced with fcn_MapGen_polytopesFillFieldsFromVertices

% TO DO:
% -- update functions to be compatible with new MapGen

clear
clc
close all

%% add necessary directories
flag_do_plot = 1;

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


%% calculate original visibility graph
create_vgraph_timer = tic;
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
toc(create_vgraph_timer)
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

%% test removing polytope
idx_of_polytope_for_removal = 8;
polytope_for_removal = polytopes(idx_of_polytope_for_removal);
if flag_do_plot
    fig = 99; % figure to plot on
    line_spec = 'r--'; % edge line plotting
    fcn_plot_polytopes(polytope_for_removal,fig,line_spec,line_width,axes_limits,axis_style);
    title('original map with polytope for removal highlighed')
end

modify_vgraph_timer = tic;
[vgraph_new, all_pts_new, start_new, finish_new, new_polytopes] = fcn_visibility_graph_remove_obstacle(vgraph, all_pts, [], [], polytopes, idx_of_polytope_for_removal);
toc(modify_vgraph_timer)

if flag_do_plot
    fig = 999; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(new_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('vgraph and map after polytope removal')
    for i = 1:size(vgraph_new,1)
        for j = 1:size(vgraph_new,1)
            if vgraph_new(i,j) == 1
                plot([all_pts_new(i,1),all_pts_new(j,1)],[all_pts_new(i,2),all_pts_new(j,2)],'-g')
            end
        end
    end
end

%% test adding obstacle
polytope_to_add = polytope_for_removal; % we can add back the polytope we removed
polytope_shift = 0.7; % let's transalate it though
polytope_to_add.xv = polytope_to_add.xv + polytope_shift; % apply translation to vertices
polytope_to_add.vertices(:,1) = polytope_to_add.vertices(:,1) + polytope_shift;
add_obs_timer = tic;
[vgraph_new2, all_pts_new2, start_new2, finish_new2, new_polytopes2] = ...
    fcn_Visibility_addObstacle(...
    vgraph_new, all_pts_new, start_new, finish_new, new_polytopes, polytope_to_add);
toc(add_obs_timer)

if flag_do_plot
    fig = 9999; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1.5 0 1.5]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(new_polytopes2,fig,line_spec,line_width,axes_limits,axis_style);
    line_spec = 'r--'; % edge line plotting
    fcn_plot_polytopes(polytope_to_add,fig,line_spec,line_width,axes_limits,axis_style);
    title('original map with polytope for addition highlighed')
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('vgraph and map after polytope addition')
    n_vgraph_new2 = size(vgraph_new2,2);
    n_vgraph_new = size(vgraph_new,2);
    vgraph_new_for_comparison = [vgraph_new, zeros(n_vgraph_new,n_vgraph_new2-n_vgraph_new); zeros(n_vgraph_new2-n_vgraph_new,n_vgraph_new2)];
    for i = 1:size(vgraph_new2,1)
        for j = 1:size(vgraph_new2,1)
            if vgraph_new2(i,j) == 1 && vgraph_new_for_comparison(i,j) ==1
                plot([all_pts_new2(i,1),all_pts_new2(j,1)],[all_pts_new2(i,2),all_pts_new2(j,2)],'-g')
            end
            if vgraph_new2(i,j) == 1 && ~vgraph_new_for_comparison(i,j) ==1
                plot([all_pts_new2(i,1),all_pts_new2(j,1)],[all_pts_new2(i,2),all_pts_new2(j,2)],'-m')
            end
        end
    end
end

%% try adding another obstacle
polytope_to_add2.vertices = [1.09, 0.78; 1.17, 0.78; 1.17, 0.86; 1.09, 0.86; 1.09, 0.78];
polytope_to_add2 = fcn_MapGen_polytopesFillFieldsFromVertices(polytope_to_add2);
polytope_to_add2.obs_id = nan;
polytope_to_add2.perimeter = sum(polytope_to_add2.distances);
add_obs_timer = tic;
[vgraph_new3, all_pts_new3, start_new3, finish_new3, new_polytopes3] = ...
    fcn_Visibility_addObstacle(...
    vgraph_new2, all_pts_new2, start_new2, finish_new2, new_polytopes2, polytope_to_add2);
toc(add_obs_timer)

if flag_do_plot
    fig = 99999; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [0 1.5 0 1.5]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(new_polytopes3,fig,line_spec,line_width,axes_limits,axis_style);
    line_spec = 'r--'; % edge line plotting
    fcn_plot_polytopes(polytope_to_add2,fig,line_spec,line_width,axes_limits,axis_style);
    title('original map with polytope for addition highlighed')
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('vgraph and map after blocking polytope addition')
    n_vgraph_new3 = size(vgraph_new3,2);
    vgraph_new2_for_comparison = [vgraph_new2, zeros(n_vgraph_new2,n_vgraph_new3-n_vgraph_new2); zeros(n_vgraph_new3-n_vgraph_new2,n_vgraph_new3)];
    for i = 1:size(vgraph_new3,1)
        for j = 1:size(vgraph_new3,1)
            if vgraph_new3(i,j) == 1
                plot([all_pts_new3(i,1),all_pts_new3(j,1)],[all_pts_new3(i,2),all_pts_new3(j,2)],'-g')
            end
            if vgraph_new3(i,j) == 1 && ~vgraph_new2_for_comparison(i,j) ==1
                plot([all_pts_new3(i,1),all_pts_new3(j,1)],[all_pts_new3(i,2),all_pts_new3(j,2)],'-m')
            end
        end
    end
end
