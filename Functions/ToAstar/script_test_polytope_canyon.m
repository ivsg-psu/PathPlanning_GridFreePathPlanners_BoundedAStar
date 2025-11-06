% script_test_3d_polytope_canyon

% example of routing through a field of polytopes with a large chokepoint in the middle
% reachability and visibilty incentive cost functions can be used to route around the choke point

% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability
% -- removed calls to fcn_algorithm_generate_cost_graph,
%    % replaced with fcn_BoundedAStar_generateCostGraph
% 2025_11_02 - S. Brennan
% -- changed fcn_BoundedAStar_polytopesGenerateAllPtsTable 
%    % to fcn_Visibility_polytopesGenerateAllPtsTable
%    % WARNING: inputs/outputs to this changed slightly. Function needs to 
%    % be rechecked


% addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
% addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'
% addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
% addpath 'C:\Users\sjhar\Desktop\gif\gif'
% addpath 'C:\Users\sjh6473\Desktop\gif\gif'
% addpath 'C:\Users\sjh6473\Desktop\TriangleRayIntersection'

%% load test fixtures for polytope map rather than creating it here
% load distribution north of canyon
load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes1.mat'));
% this test fixture was made with the following block of code using functions from the MapGen repo
% tiled_polytopes1 = fcn_MapGen_haltonVoronoiTiling([1,20],[2 1]);
% % remove the edge polytope that extend past the high and low points
% % shink the polytopes so that they are no longer tiled
% des_radius = 0.05; % desired average maximum radius
% sigma_radius = 0.002; % desired standard deviation in maximum radii
% min_rad = 0.0001; % minimum possible maximum radius for any obstacle
% [shrunk_polytopes1,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes1,des_radius,sigma_radius,min_rad);

% load polytopes representing canyon
load(strcat(pwd,'\..\Test_Fixtures\canyon_polys_without_exterior.mat'));
% these polytopes were manually defined

% load distribution south of canyon
load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes2.mat'));
% this test fixture was made with the following block of code using functions from the MapGen repo
% tiled_polytopes2 = fcn_MapGen_haltonVoronoiTiling([1, 20],[2 1]);
% % remove the edge polytope that extend past the high and low points
% % shink the polytopes so that they are no longer tiled
% [shrunk_polytopes2,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes2,des_radius,sigma_radius,min_rad);
tic

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;

%% move second polytope field north of canyon
second_field_vertical_translation = 1.5;
for i = 1:length(shrunk_polytopes2)
    num_verts_this_poly = length(shrunk_polytopes2(i).yv);
    shrunk_polytopes2(i).yv = shrunk_polytopes2(i).yv + second_field_vertical_translation;
    shrunk_polytopes2(i).vertices = shrunk_polytopes2(i).vertices + [zeros(num_verts_this_poly+1,1) second_field_vertical_translation*ones(num_verts_this_poly+1,1)];
end

%% combine two polytope fields and canyon choke point into one field
shrunk_polytopes = [shrunk_polytopes1, shrunk_polytopes2, polytopes_manual_canyon];


if flag_do_plot
    %% plot the map
    figure; hold on; box on;
    xlabel('x [m]');
    ylabel('y [m]');
    title('polytope map in x-y plane at time 0')
    for i = 1:length(shrunk_polytopes)
         fill(shrunk_polytopes(i).vertices(:,1)',shrunk_polytopes(i).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    end
end

%% define start and finish
start = [0 1.25];
finish = [2 1.25];


%% all_pts array creation
if 1==1
    warning('The function fcn_Visibility_polytopesGenerateAllPtsTable is not a direct replacement for the BoundedAStar version. The function needs to be updated from this point onward.')
    all_pts = fcn_Visibility_polytopesGenerateAllPtsTable(shrunk_polytopes, start, finish,-1);
else
    % % OLD:
    % all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes, start, finish,-1);
end


%% plan path
start = [start size(all_pts,1)+1 -1 1]
finish = [finish size(all_pts,1)+2 -1 1]
finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
[vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes);

[is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));

% new experimental cost function prioritizing reachability
reachable_nodes_from_each_node = sum(rgraph,2);
inv_reach_cost = 10*(1./reachable_nodes_from_each_node)';

% new experimental cost function prioritizing visibility
visible_nodes_from_each_node = sum(vgraph,2);
inv_vis_cost = 10*(1./(visible_nodes_from_each_node))';

%% make cgraph
mode = "xy spatial only";
% mode = 'time or z only';
% mode = "xyz or xyt";
[cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode);

hvec = hvec + inv_reach_cost + inv_vis_cost;

[cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

if flag_do_plot
    hold on
    plot(route(:,1),route(:,2),'k','LineWidth',2);
    plot(start(1),start(2),'xg');
    plot(finish(1),finish(2),'xr');
end

%% generate metrics on visibility and reachability
nodes_vis_from_route = 0;
for i = 1:size(route,1)
    way_pt = route(i,3);
    vgraph_row = vgraph(way_pt,:);
    nodes_vis_from_way_pt = sum(vgraph_row);
    nodes_vis_from_route = nodes_vis_from_route + nodes_vis_from_way_pt;
end
nodes_reach_from_route = 0;
for i = 1:size(route,1)
    way_pt = route(i,3);
    rgraph_row = rgraph(way_pt,:);
    nodes_reach_from_way_pt = sum(rgraph_row);
    nodes_reach_from_route = nodes_reach_from_route + nodes_reach_from_way_pt;
end

x = route(:,1);
y = route(:,2);
d = diff([x(:) y(:)]);
total_length = sum(sqrt(sum(d.*d,2)));

metrics_title = sprintf('nodes visible from route waypoints: %i \n nodes visible per unit length: %.2f \n nodes reachable from route waypoints: %i \n nodes reachable per unit length: %.2f',nodes_vis_from_route,nodes_vis_from_route/total_length,nodes_reach_from_route,nodes_reach_from_route/total_length);
title(metrics_title);

all_pts = [all_pts; start; finish];

%% highlight visible nodes and reachable nodes
if flag_do_plot
    num_waypoints = size(route,1);
    for i = 2:num_waypoints-1
        waypoint_id = route(i,3);
        visible_nodes = find(vgraph(waypoint_id,:));
        num_visible_nodes = size(visible_nodes,2);
        for j = 1:num_visible_nodes
            visible_node_id = visible_nodes(j);
            plot(all_pts(visible_node_id,1),all_pts(visible_node_id,2),'o','MarkerSize',3,'MarkerEdgeColor','r','MarkerFaceColor',[0.9290 0.6940 0.1250])
        end
    end
end
toc
function INTERNAL_fcn_format_timespace_plot()
    box on
    % define figure properties
    opts.width      = 8;
    opts.height     = 6;
    opts.fontType   = 'Times';
    opts.fontSize   = 9;
    fig = gcf;
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
    view([36 30])
end
