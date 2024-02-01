% script_fcn_algorithm_generate_dilation_robustness_matrix
% Tests: fcn_convert_polytope_struct_to_deduped_points

%
% REVISION HISTORY:
%
% 2022_11_10 by S. Harnett
% -- first write of script
%%%%%%%%%%%%%%§
close all; clear all; clc;

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_plot_slow = 0;

fig_num = 2;
line_width = 3;
clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 4; 0 0];
polytopes(2).vertices = [0 -1; 4, -1; 5 -2; 3 -10; 0 -1];
polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes);
fcn_MapGen_plotPolytopes(polytopes,fig_num,'b-',line_width);
hold on; box on;
xlabel('x [m]');
ylabel('y [m]');
title('dilation robustness')


start = ones(1,2)*(-0.5);
finish = start;
finish(1) = 6;

plot(start(1),start(2),'xg','MarkerSize',6);
plot(finish(1),finish(2),'xr','MarkerSize',6);

point_tot = length([polytopes.xv]); % total number of vertices in the polytopes
beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
curpt = 0;
for poly = 1:size(polytopes,2) % check each polytope
    verts = length(polytopes(poly).xv);
    polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
    beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+verts;
end
obs_id = [polytopes.obs_id];
all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
start = [start size(all_pts,1)+1 -1 1];
finish = [finish size(all_pts,1)+2 -1 1];

plot(all_pts(:,1), all_pts(:,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
nudge = 1.05;
text(all_pts(:,1)*nudge,all_pts(:,2)*nudge,string(all_pts(:,3)));

finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
[vgraph, visibility_results_all_pts] = fcn_visibility_clear_and_blocked_points_global(polytopes, starts, finishes,1);
% plot visibility graph edges
if flag_do_plot_slow
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--g','LineWidth',2)
            end
            if vgraph(i,j) == 0
                plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--r','LineWidth',2)
            end
        end
    end
end
mode = '2d';
dilation_robustness_matrix = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode)
% check that all zeros are in the same place
vgraph = vgraph - eye(size(vgraph));
zero_vgraph_edge_idx = find(vgraph==0);
zero_dilation_robustness_matrix_idx = find(vgraph==0);
assert(isequal(zero_vgraph_edge_idx, zero_dilation_robustness_matrix_idx));

dilation_robustness_values = dilation_robustness_matrix';
dilation_robustness_values = dilation_robustness_values(:)';
max_dilation_robustness_excluding_inf = max(dilation_robustness_values(~isinf(dilation_robustness_values)));

% plot visibility graph edges
if flag_do_plot
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                alpha = dilation_robustness_matrix(i,j)/max_dilation_robustness_excluding_inf;
                if alpha == inf
                    continue
                end
                plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--','Color',[alpha 0 1-alpha],'LineWidth',2)
            end
        end
    end
end
map = [(linspace(0,1,100))' zeros(100,1) (linspace(1,0,100))'];
colormap(map)
set(gca,'CLim',[0 1]*max_dilation_robustness_excluding_inf);
c = colorbar
c.Label.String = 'dilation robustness'
