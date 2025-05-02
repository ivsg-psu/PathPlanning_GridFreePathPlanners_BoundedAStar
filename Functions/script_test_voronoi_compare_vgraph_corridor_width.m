% script_test_voronoi_compare_vgraph_corridor_width
% demo script comparing corridor width as estimtated from vgraph to estiamted from medial axis
clear; close all; clc

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;

%% make halton set polytope field
% pull halton set
rng(1);
halton_points = haltonset(2);
points_scrambled = scramble(halton_points,'RR2'); % scramble values

% pick values from halton set
Halton_range = [1801 1851];
low_pt = Halton_range(1,1);
high_pt = Halton_range(1,2);
seed_points = points_scrambled(low_pt:high_pt,:);

% fill polytopes from tiling
AABB = [0 0 1 1];
stretch = AABB(3:4);
tiled_polytopes = fcn_MapGen_generatePolysFromVoronoiAABBWithTiling(seed_points,AABB, stretch);

% stretch polytopes to cover more area
new_stretch = [30 40];
stretched_polytopes = [];
for poly = 1:length(tiled_polytopes) % pull each cell from the voronoi diagram
    stretched_polytopes(poly).vertices  = tiled_polytopes(poly).vertices.*new_stretch;
end % Ends for loop for stretch
stretched_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(stretched_polytopes);

% shrink polytopes to desired radius
% des_rad = 2; sigma_radius = 0.4; min_rad = 0.1;
% [polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(stretched_polytopes,des_rad,sigma_radius,min_rad);
%% shrink based on gap size so we know the exact corridor width
des_gap_size = 1;
polytopes=...
    fcn_MapGen_polytopesShrinkFromEdges(...
    stretched_polytopes,des_gap_size);

clear Halton_range
clear halton_points
clear points_scrambled

start_init = [-2 20];
finish_init = [32 20];
%% make a boundary around the polytope field
boundary.vertices = [-3 -5; -3 45; 33 45; 33 -5];
boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % close the shape by repeating first vertex
boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
shrunk_polytopes = [boundary, polytopes]; % put the boundary polytope as the first polytope

resolution_scale = 20; % this map has many fine features and resolution can be 10x the nominal


%% for sake of comparisons, do this with a visibility graph as well
polytopes = shrunk_polytopes;
if flag_do_plot
    fig = 99; % figure to plot on
    line_spec = 'b-'; % edge line plotting
    line_width = 2; % linewidth of the edge
    axes_limits = [1025 1055 -4726 -4710]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    fcn_plot_polytopes(polytopes(2:end),fig,line_spec,line_width,axes_limits,axis_style);
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
end
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
[vgraph, visibility_results] = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts,all_pts);

%% generate vgraph corridor width estimate
start = [start_init size(all_pts,1)+1 -1 1];
finish = [finish_init size(all_pts,1)+2 -1 1];
mode = '2d';
dilation_robustness_matrix = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode, polytopes);
finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];

%% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-r')
            end
        end
    end
end
%% plot dilation robustness (corridor width) heat map
for left_or_right = [1,2]
    % show the difference between measuring to the right and to the left
    dilation_robustness_values = dilation_robustness_matrix(:,:,1) +  dilation_robustness_matrix(:,:,2);
    dilation_robustness_values = dilation_robustness_values(:)';
    max_dilation_robustness_excluding_inf = max(dilation_robustness_values(~isinf(dilation_robustness_values) & ~isinf(-dilation_robustness_values)));

    % plot corridor width approximation graph edges
    if flag_do_plot
        fig_num = 123410;
        fcn_MapGen_plotPolytopes(polytopes,fig_num,'g-',line_width);
        hold on; box on;
        xlabel('x [km]');
        ylabel('y [km]');
        title('Visibility Graph');
        for i = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(i,j) == 1
                    alpha = dilation_robustness_matrix(i,j,left_or_right)/max_dilation_robustness_excluding_inf;
                    if alpha == inf %| alpha == -inf
                        continue % don't plot infinite values
                    end
                    plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--','Color',[alpha 0 1-alpha],'LineWidth',2)
                end
            end
        end
        colormap(turbo)
        set(gca,'CLim',sort([0 1]*max_dilation_robustness_excluding_inf));
        c = colorbar;
        c.Label.String = 'corridor width [km]';
    end
end

%% constrained delaunay triangulation
[adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);

%% prune graph
[adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, flag_do_plot);

%% get costs for navigating each triangle chain
% TODO add zcc as optional input
[triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, flag_do_plot);
figure(13)
title('Medial Axis Graph')
dilation_robustness_values = (dilation_robustness_matrix(:,:,1)' + dilation_robustness_matrix(:,:,2)');
dilation_robustness_values = dilation_robustness_values(dilation_robustness_values ~= 0);
dilation_robustness_values = dilation_robustness_values(~isinf(dilation_robustness_values));
dilation_robustness_values = dilation_robustness_values(dilation_robustness_values < 1.5);
mean_dilation_robustness = mean(dilation_robustness_values)
median_dilation_robustness = median(dilation_robustness_values)
var_dilation_robustness = var(dilation_robustness_values)

triangle_chain_widths = [triangle_chains{:,4}];
triangle_chain_widths = triangle_chain_widths(~isnan(triangle_chain_widths));
triangle_chain_widths = triangle_chain_widths(~isinf(triangle_chain_widths));
triangle_chain_widths = triangle_chain_widths(triangle_chain_widths ~= 0);
triangle_chain_widths = triangle_chain_widths(triangle_chain_widths < 1.5);
mean_triangle_chain_width = mean(triangle_chain_widths)
median_triangle_chain_width = median(triangle_chain_widths)
var_triangle_chain_width = var(triangle_chain_widths)
