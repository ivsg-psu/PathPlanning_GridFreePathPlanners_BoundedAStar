% script_test_voronoi_planning_interface
% test script of planning along voronoi diagram edges via a wrapper function that encompases the
% medial axis planning stack
clear; close all; clc

%% declair dependencies
addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% polytopes for testing
n_polys = 16; % number of polytopes
polytope_vertices = cell(1,n_polys); % initialize empty cell array
rng(10); % set rng seed for determinism
num_verts = randi([3 3],n_polys); % random integer for num. verts of each poly, between 3 and 10
poly_scale = ones(1,n_polys); % random vector of scale factors between 0 and 3 for polytope size
figure; hold on; box on;
for poly = 1:n_polys
    verts = rand(num_verts(poly),2); % create random nx2 matrix of vertices
    verts = verts - 0.5; % center random dist. at 0
    verts = verts*poly_scale(poly);
    [x_shift, y_shift] = ind2sub([4, 4], poly);
    verts = verts + [x_shift*ones(num_verts(poly),1) y_shift*ones(num_verts(poly),1)]; % shift vertices based on position in matrix
    verts = [verts; verts(1,:)]; % repeat first vertex at end
    fill(verts(:,1),verts(:,2),poly_scale(poly));
    polytope_vertices{poly} = verts;
end

%% other inputs for testing
start = [0.5 0.5];
finish = [4.5 4.5];
boundary_verts = [0.25 0.25;...
                  0.25 4.75;...
                  4.75 4.75;...
                  4.75 0.25;...
                  0.25 0.25];
min_corridor_width = 0;
length_cost_weight = 1;

% TODO functional interface should begin here
fcn_MedialAxis_plannerWrapper(polytope_vertices, start, finish, boundary_verts, min_corridor_width, length_cost_weight)
function fcn_MedialAxis_plannerWrapper(polytope_vertices, start, finish, boundary_verts, min_corridor_width, length_cost_weight)
flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;

clear polytopes
for poly = 1:length(polytope_vertices)
    polytopes(poly).vertices = polytope_vertices{poly};
end
boundary.vertices = boundary_verts;
boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
shrunk_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes);
shrunk_polytopes = [boundary, shrunk_polytopes]; % put the boundary polytope as the first polytope

fig_num = 2;
line_width = 3;
fcn_MapGen_plotPolytopes(polytopes,fig_num,'r-',line_width);

resolution_scale = 0.5;
%% constrained delaunay triangulation
[adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(shrunk_polytopes, resolution_scale, flag_do_plot);

%% prune graph
[adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, flag_do_plot);

%% get costs for navigating each triangle chain
% TODO add zcc as optional input
[triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, flag_do_plot);

%% planning through triangle graph
start_xy = start;
finish_xy = finish;
% add start and finish to nearest medial axis edge
% TODO add zcc as optional input
[adjacency_matrix, triangle_chains, nodes, start_closest_tri, start_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(start_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);
[adjacency_matrix, triangle_chains, nodes, finish_closest_tri, finish_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(finish_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri);

% loop over cost function weights
w = length_cost_weight;
denylist_route_chain_ids = []; % no need to denylist any triangle chains
% make cost matrix
[adjacency_matrix, cgraph, all_pts, start, finish, best_chain_idx_matrix] = fcn_MedialAxis_makeCostGraphAndAllPoints(adjacency_matrix, triangle_chains, nodes, xcc, ycc, start_closest_tri, start_closest_node, finish_closest_tri, finish_closest_node, w, min_corridor_width, denylist_route_chain_ids);
% adjacency matrix is vgraph
vgraph = adjacency_matrix;
num_nodes = length(nodes);
vgraph(1:num_nodes+1:end) = 1;
% check reachability
[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(3),finish(3));
if ~is_reachable
    error('initial mission, prior to edge deletion, is not possible')
end
% run Dijkstra's algorithm (no heuristic)
hvec = zeros(1,num_nodes);

% plan a path
[cost, route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

% expand route nodes into actual path
[route_full, route_length, route_choke, route_triangle_chain, route_triangle_chain_ids] = fcn_MedialAxis_processRoute(route, triangle_chains, best_chain_idx_matrix, xcc, ycc, start_xy, finish_xy);
% plot result
figure; hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
leg_str = {};
leg_str{end+1} = 'medial axis graph';
not_first = 0;
% plot all triangle chains
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    % plot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjacency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',0.6*ones(1,3));
    if not_first
        leg_str{end+1} = '';
    end
    not_first =1;
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',0.6*ones(1,3));
    leg_str{end+1} = '';
end
plot(start_xy(1),start_xy(2),'xg','MarkerSize',10);
plot(finish_xy(1),finish_xy(2),'xr','MarkerSize',10);
plot(start(1),start(2),'.g','MarkerSize',10);
plot(finish(1),finish(2),'.r','MarkerSize',10);
leg_str{end+1} = 'start';
leg_str{end+1} = 'finish';
leg_str{end+1} = 'start node';
leg_str{end+1} = 'finish node';
plot(route(:,1),route(:,2),'--r','MarkerSize',20,'LineWidth',1);
leg_str{end+1} = sprintf('adjacency of route nodes');
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
leg_str{end+1} = 'obstacles';
for i = 1:length(shrunk_polytopes)-2
    leg_str{end+1} = '';
end
plot(route_full(:,1), route_full(:,2), '-k','LineWidth',2.5) % plot approx. medial axis
leg_str{end+1} = 'medial axis route';
legend(leg_str,'Location','best');
tit_str = sprintf('length cost weight was: %.1f \n total length: %.2f km \n worst corridor: %.2f km',w, route_length, route_choke);
title(tit_str)
end % end weight loop
