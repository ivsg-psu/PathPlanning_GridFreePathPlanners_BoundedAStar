% script_test_fcn_find_edge_weights
% Tests: fcn_find_edge_weights

%
% REVISION HISTORY:
%
% 2022_11_01 by S. Harnett
% -- first write of script
%%%%%%%%%%%%%%§

clear
clc
close all

%% add necessary directories
addpath([pwd '\Example_Map_Generation_Code'])
addpath([pwd '\PathPlanning_MapTools_MapGenClassLibrary\Functions'])
addpath([pwd '\PathPlanning_GeomTools_GeomClassLibrary\Functions'])

flag_do_plot = 1;

%% generate map
Halton_seed = 10;
low_pt = 1+Halton_seed; high_pt = 5+Halton_seed; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_haltonVoronoiTiling([low_pt,high_pt],[1 1]);
% shink the polytopes so that they are no longer tiled
gap_size = 0; % desired average maximum radius
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
    polytopes(poly).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly).vertices(1:end-1,:),polytopes(poly).vertices(2:end,:));
    beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+num_verts;
    polytopes(poly).perimeter = sum(polytopes(poly).distances);
end

obs_id = [polytopes.obs_id];
point_tot = length([polytopes.xv]); % need to recheck total points
beg_end = beg_end(1:point_tot); % remove any extra points

all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]

%% calculate weighted visibility graph (cost graph)
[cgraph, vgraph] = fcn_find_edge_weights(polytopes, all_pts, gap_size);
deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
% plot visibility graph edges
if flag_do_plot
    
deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
% plot visibility graph edges
    if gap_size ==0
        for i = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(i,j) == 1
                    plot([deduped_pts(i).x,deduped_pts(j).x],[deduped_pts(i).y,deduped_pts(j).y],'--g','LineWidth',1)
                end
            end
        end
    end
    if gap_size ~=0
        for i = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(i,j) == 1
                    plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
                end
            end
        end
    end
%     for i = 1:size(all_pts,1)
%         hold on;
%             txt = sprintf('%.2f',round(all_pts(i,3),0));
%             %% test that all_pts id column is correct
%             assert(isequal(all_pts(i,3),i));
%             x = all_pts(i,1);
%             y = all_pts(i,2);
%             text(x, y, txt, 'clipping', 'off', 'Color', 'r');
%     end
end

% plot cgraph edges
if flag_do_plot
    % for symmetric we only ned upper triangular part
    cgraph_upper_tri = triu(cgraph,1);
    [r, c] = find(cgraph_upper_tri>0);
    for i = 1:size(r,1)
            hold on;
            txt = sprintf('%.2f',round(cgraph(r(i),c(i)),2));
            x1 = all_pts(r(i),1);
            x2 = all_pts(c(i),1);
            y1 = all_pts(r(i),2);
            y2 = all_pts(c(i),2);
            xbar = mean([x1, x2]);
            ybar = mean([y1, y2]);
            text(xbar, ybar, txt, 'clipping', 'off', 'Color', 'b');
    end
    % test for symmetry
    assert(issymmetric(vgraph))
    assert(issymmetric(cgraph))
    % test that vgraph and cgraph have the same number of non-zero elements
    assert(isequal(sum(sum(vgraph)),sum(sum(cgraph>0))))
end
