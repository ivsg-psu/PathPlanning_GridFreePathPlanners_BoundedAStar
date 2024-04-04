clear; close all; clc
% script_test_voronoi_planning
% test script of planning along voronoi diagram edges

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;

for map_idx =5
    if map_idx == 1 % generic canyon map
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
        %% move second polytope field north of canyon
        second_field_vertical_translation = 1.5;
        for i = 1:length(shrunk_polytopes2)
            num_verts_this_poly = length(shrunk_polytopes2(i).yv);
            shrunk_polytopes2(i).yv = shrunk_polytopes2(i).yv + second_field_vertical_translation;
            shrunk_polytopes2(i).vertices = shrunk_polytopes2(i).vertices + [zeros(num_verts_this_poly+1,1) second_field_vertical_translation*ones(num_verts_this_poly+1,1)];
        end

        %% combine two polytope fields and canyon choke point into one field
        shrunk_polytopes = [shrunk_polytopes1, shrunk_polytopes2, polytopes_manual_canyon];
        %% define start and finish
        start_init = [0 1.25];
        finish_init = [2 1.25];
    elseif map_idx == 2 % the lower triangular flood plain
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_1.mat'));
        shrunk_polytopes = flood_plain_1;
        start_init = [-78.3 40.88];
        % finish_init = [-78.1 40.9];
        finish_init = [-78.07 40.82];
    elseif map_idx == 3 % the mustafar mining rig map (the comb)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_2.mat'));
        shrunk_polytopes = flood_plain_2;
        start_init = [-78.02 40.96];
        % finish_init = [-77.86 40.93];
        finish_init = [-77.82 40.97];
    elseif map_idx == 4 % also good for edge deletion case (the long river valleys)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_3.mat'));
        shrunk_polytopes = flood_plain_3;
        start_init = [-77.49 40.84];
        % finish_init = [-77.58 40.845];
        finish_init = [-77.68 40.85];
    elseif map_idx == 5 % bridge map, good for random edge deletion case
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_4.mat'));
        shrunk_polytopes = flood_plain_4;
        is_nonconvex = 1;
        start_init = [-77.68 40.9];
        finish_init = [-77.5 40.8];
    elseif map_idx == 6 % large map, good for dilation case, nearly fully tiled
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_5.mat'));
        shrunk_polytopes = flood_plain_5;
        start_init = [-78.01 41.06];
        finish_init = [-77.75 40.93];
    elseif map_idx == 7 % generic polytope map
        is_nonconvex = 0;
        % pull halton set
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
        des_rad = 2; sigma_radius = 0.4; min_rad = 0.1;
        [shrunk_polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(stretched_polytopes,des_rad,sigma_radius,min_rad);

        clear Halton_range
        clear halton_points
        clear points_scrambled

        start_init = [-2 20];
        finish_init = [32 20];
        % tile field to hedgerow by making a set above and a set below
    end % if conditions for different map test fixtures
end

%% make a boundary around the polytope field
figure; hold on; box on;
boundary.vertices = [-77.7 40.78; -77.7 40.92; -77.45 40.92; -77.45 40.78];
boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % close the shape by repeating first vertex
boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
boundary.parent_poly_id = nan; % ignore parend ID
shrunk_polytopes = [boundary, shrunk_polytopes]; % put the boundary polytope as the first polytope

%% interpolate polytope vertices
distances = diff([[shrunk_polytopes.xv]',[shrunk_polytopes.yv]']); % find side lengths in whole field
min_distance_between_verts = min(sqrt(sum(distances.*distances,2))); % the min of this is the smallest space between features
resolution = min_distance_between_verts/2; % want even the smallest feature to be bisected
shrunk_polytopes = fcn_MapGen_increasePolytopeVertexCount(shrunk_polytopes, resolution); % interpolate sides

%% constrained delaunay triangulation
C = []; % initialize constriant matrix
P = []; % initialize points matrix
largest_idx = 0;
for poly_idx = 1:length(shrunk_polytopes)
    P = [P; shrunk_polytopes(poly_idx).vertices(1:end-1,:)]; % add poly verts to points
    % want to list every pair of points representing a polytope side as a constraint
    num_verts = size(shrunk_polytopes(poly_idx).vertices,1)-1;
    C1 = [1:num_verts]'; % this will be points 1 through n
    C2 = [C1(2:end);C1(1)]; % C2 is equal to C1 with the order shifted by 1
    Ccomb = [C1 C2]; % this will be a table saying point 1 to 2 is a side, 2 to 3, etc.
    Ccomb = Ccomb + largest_idx; % shift the whole thing by the largest ID already in C so point IDs are global, not unique to this polytope onlyk
    C = [C; Ccomb]; % add constriants for this polytope to total constraint list
    largest_idx = max(max(C)); % update largest point ID in this polytope for use offsetting next polytope
end
x = P(:,1); % all x's
y = P(:,2); % all y's
DT = delaunayTriangulation(P,C) % perform constrained triangulation

figure; triplot(DT); title('triangulation')
inside = isInterior(DT); % identify triangles statisfying constriants C (i.e. tris within the boundary and outside polytopes, i.e. free space)
tr = triangulation(DT(inside,:),DT.Points); % keep only the triangles of free space, not the ones in polytopes
figure; triplot(tr); title('triangulation, no interior')
numt = size(tr,1); % numbe of triangles
T = (1:numt)'; % list of triangle idx
neigh = neighbors(tr); % this indicates which triangles are connected to which other triangles
cc = circumcenter(tr); % get all circumcenters (centers of circumscribed circles)
nodes = find(~isnan(sum(neigh, 2))); % identify all 3 connected triangles (tris with no nan neighbor)
xcc = cc(:,1); % x coords of circumcenters
ycc = cc(:,2); % y coords of circumcenters
% the following code rearranges the numtx3 'neigh' matrix where row i is the three neighbors of tri i,
% into a 2xm matrix where each columb is a pair of neighboring triangles
% this is useful for plotting as neigh_for_plotting has no nan values while neigh does
idx1 = T < neigh(:,1);
idx2 = T < neigh(:,2);
idx3 = T < neigh(:,3);
neigh_for_plotting = [T(idx1) neigh(idx1,1); T(idx2) neigh(idx2,2); T(idx3) neigh(idx3,3)]';
% plot the triangulation and approximate medial axis
figure; hold on; box on;
triplot(tr,'g')
hold on
plot(xcc(neigh_for_plotting), ycc(neigh_for_plotting), '-r','LineWidth',1.5) % plot approx. medial axis
plot(xcc(nodes), ycc(nodes), '.k','MarkerSize',30) % plot 3 connected triangle circumcenters
plot(x(C'),y(C'),'-b','LineWidth',1.5) % plot constriants (i.e. polytopes)
xlabel('Medial Axis of Polygonal Domain','FontWeight','b')

% make a plannable graph from triangulation
% identify the 3 connected triangles
adjascency_matrix = eye(length(nodes)); % set to 1 if chain of 2 connected triangles exists between three connected triangle node(i) and node(j)
triangle_chains = {}; % each row contains (i,1) start 3 connected tri, (i,2) end 3 connected tri, and (i,3) array of 2 connected tris between them
path_is_explored = zeros(length(nodes),3); % set to 1 when a direction is explored when node(i) neighbor(i,j) is explored
% while there is still a 0 in the path explored list...
while ~isempty(find(path_is_explored == 0))
    % find the first 0...it will be at i,j so we want nodes(i) in direction neigh(i,j)
    [r,c] = find(path_is_explored == 0);
    i = r(1);
    direction = c(1);

    tris_visited = []; % initialize triangles visited list
    tris_visited = [nodes(i)]; % pick one starting node
    % pick a direction from neighbors of the node triangle
    direction_choices = neigh(nodes(i),:); % there should be three directions leaving nodes(i)
    % next tri should just be neigh(node(i),j) the jth neightbor of node(i)
    next_tri = direction_choices(direction); % go a direction that hasn't been explored yet
    tris_visited = [tris_visited, next_tri];
    % want to keep looking while the current triangle is not a 3 connected one
    % i.e. while the last triangle visited is not a node
    while ~ismember(tris_visited(end),nodes)
        % march down direction until hit a 3 connected triangle, noting every triangle on the way
        neighbors = neigh(next_tri,:); % find the neighbors of the current triangle, this is a 1x3 list of tri idx
        % whichever of the two neighbors we haven't visited, is the direction we didn't come from
        next_dir = find(~isnan(neighbors)&~ismember(neighbors,tris_visited)); % this will be an ID between 1 and 3
        % if there is no next neighbor satisfying (not nan) && (not already visited)
        % we can assume we hit a dead end and this whole chain can be removed
        if isempty(next_dir)
            % set the flag that indicates this was a dead end as this has special handling
            is_dead_end = 1;
            % break out of the while loop
            break
        else
            % otherwise we must have selected a 2 or three connected triangle
            is_dead_end =0;
        end
        next_tri = neighbors(next_dir); % update the next tiangle based on the valid direction ID
        tris_visited = [tris_visited, next_tri]; % append to the triangle visited list
    end % end triangle chain while loop
    % special dead end handling
    if is_dead_end
        % don't store the triangles
        % mark the direction as explored
        path_is_explored(i,direction) = 1;
        % don't udpate adjacency
        % reset dead end flag
        is_dead_end = 0;
        continue % to next unexplored direction
    end
    % store tris visited list in the triangle chains lookup table
    triangle_chains{end+1,1} = find(nodes==tris_visited(1)); % index of start in nodes
    triangle_chains{end,2} = find(nodes==tris_visited(end)); % index of end in nodes
    triangle_chains{end,3} = tris_visited; % list of triangles between them
    % note that the start and end of the triangle chain are "adjascent" in a graph sense
    adjascency_matrix(find(nodes==tris_visited(1)),find(nodes==tris_visited(end))) = 1;
    % flag the direction as explored
    path_is_explored(i,direction) = 1;
end % end direction while loop

% plot the graph on the triangles
figure; hold on; box on;
triplot(tr,'g')
hold on
plot(xcc(neigh_for_plotting), ycc(neigh_for_plotting), '-r','LineWidth',1.5) % plot approx. medial axis
plot(xcc(nodes), ycc(nodes), '.k','MarkerSize',30) % plot 3 connected triangle circumcenters
plot(x(C'),y(C'),'-b','LineWidth',1.5) % plot constriants (i.e. polytopes)
xlabel('Medial Axis of Polygonal Domain','FontWeight','b')
colors = {"#A2142F","#7E2F8E","#EDB120","#0072BD"}; % some different colors
color_idx = 1;
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    % pot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjascency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
    color_idx = color_idx + 1;
end
% plot the graph
figure; hold on; box on;
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    % pot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjascency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
    color_idx = color_idx + 1;
end

%% compute branching factor (connectivity)
% branching factor is the number of nodes connected to each node
branching_factor_outbound = sum(adjascency_matrix,2)-1; % number of destination nodes per node (excluding self)
branching_factor_inbound = [sum(adjascency_matrix,1)-1]'; % number of departing nodes per node (excluding self)
% need to compare the sum of the rows and the sum of the columns.
% This will tell us inbound and outbound connections per node in an asymmetric graph
% if a node has 2 in but 3 out we would want to treat it as 3-connected becuase it does serve that role in one direction
% i.e. a nodes connectedness is determined by its max connectedness of max{in,out}
max_branching_factor = max(branching_factor_inbound,branching_factor_outbound);
% TODO fix the bug...
    % the problem is that "three connected" doesn't mean has three destinations
    % it means has three departing triangle paths
    % if it means "has three destinations" you can accidentally prune an edge that
    % it has two connections
%% remove through-put nodes
idx_2_connected_nodes = find(max_branching_factor == 2); % all two connected nodes are through nodes
% TODO this should be a while loop
while ~isempty(idx_2_connected_nodes)
    % for each through node, t...
    t = idx_2_connected_nodes(1);
    adjascent_to_t = adjascency_matrix(t,:); % find t's adjascent nodes
    adjascent_to_t(t) = 0; % don't need self adjascency for this
    % find the node on either side...call these d and b
    d_and_b = find(adjascent_to_t==1);
    d = d_and_b(1);
    b = d_and_b(2);
    if flag_do_plot_slow
        % plot the through node being removed and its neighbors
        plot(xcc(nodes(t)), ycc(nodes(t)), '.b','MarkerSize',30) % plot 3 connected triangle circumcenters
        plot(xcc(nodes(b)), ycc(nodes(b)), '.g','MarkerSize',30) % plot 3 connected triangle circumcenters
        plot(xcc(nodes(d)), ycc(nodes(d)), '.g','MarkerSize',30) % plot 3 connected triangle circumcenters
    end
    % connect those two nodes in the adjascency matrix
    adjascency_matrix(d_and_b, d_and_b) = 1; % note this line makes Adb, Abd, Add, and Abb =1
    % need to find triangle chains for d to t and t to b...
    idx_chain_dt = find([triangle_chains{:,1}]'== d & [triangle_chains{:,2}]'== t);
    idx_chain_tb = find([triangle_chains{:,1}]'== t & [triangle_chains{:,2}]'== b);
    chain_dt = triangle_chains{idx_chain_dt,3};
    chain_tb = triangle_chains{idx_chain_tb,3};
    % do this again for reverse direction
    idx_chain_bt = find([triangle_chains{:,1}]'== b & [triangle_chains{:,2}]'== t);
    idx_chain_td = find([triangle_chains{:,1}]'== t & [triangle_chains{:,2}]'== d);
    chain_bt = triangle_chains{idx_chain_bt,3};
    chain_td = triangle_chains{idx_chain_td,3};
    % make an entry for d to b and set tri list to the other two tri lists
    triangle_chains{end+1,1} = d; % index of start in nodes
    triangle_chains{end,2} = b; % index of end in nodes
    triangle_chains{end,3} = [chain_dt, chain_tb]; % list of triangles between them
    % do this again for reverse direction
    triangle_chains{end+1,1} = b; % index of start in nodes
    triangle_chains{end,2} = d; % index of end in nodes
    triangle_chains{end,3} = [chain_bt, chain_td]; % list of triangles between them
    % delete the rows for d to t and t to b
    [triangle_chains{idx_chain_dt,3}] = deal([]);
    [triangle_chains{idx_chain_tb,3}] = deal([]);
    % do this again for reverse direction
    [triangle_chains{idx_chain_bt,3}] = deal([]);
    [triangle_chains{idx_chain_td,3}] = deal([]);
    %  remove the t node from the adjascency matrix
    adjascency_matrix(t, :) = zeros(1, size(adjascency_matrix,1));
    adjascency_matrix(:, t) = zeros(size(adjascency_matrix,2), 1);
    % remove from node list
    nodes(t) = nan;

    %% re-compute branching factor (connectivity)
    branching_factor_outbound = sum(adjascency_matrix,2)-1; % number of destination nodes per node (excluding self)
    branching_factor_inbound = [sum(adjascency_matrix,1)-1]'; % number of departing nodes per node (excluding self)
    max_branching_factor = max(branching_factor_inbound,branching_factor_outbound);
    idx_2_connected_nodes = find(max_branching_factor == 2); % all two connected nodes are through nodes
    if flag_do_plot_slow
        % plot the graph after this through node removal
        figure; hold on; box on;
        for i = 1:(size(triangle_chains,1))
            % pop off a triangle chain
            chain_of_note = triangle_chains{i,3};
            if isempty(chain_of_note)
                continue
            end
            % pot big markers for the start and end node
            beg_end = [chain_of_note(1) chain_of_note(end)];
            % plot a straight line between them (this is the adjascency graph connection)
            plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
            % plot the medial axis path between them (this is the curved path from the triangle chain)
            plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
            color_idx = color_idx + 1;
        end
    end
end
% plot the graph without through nodes
figure; hold on; box on;
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    % pot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjascency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
    color_idx = color_idx + 1;
end

return
% TODO @sjharnett do we want to set these to zero/empty or actually remove them? Removing would require re-indexing
%% remove dead ends
idx_1_connected_nodes = find(max_branching_factor == 1); % all one connected nodes are dead ends
% remove the node from the adjacency matrix
adjascency_matrix(idx_1_connected_nodes, :) = zeros(length(idx_1_connected_nodes), size(adjascency_matrix,1));
adjascency_matrix(:, idx_1_connected_nodes) = zeros(size(adjascency_matrix,2), length(idx_1_connected_nodes));
% remove the node from the node list
nodes(idx_1_connected_nodes) = nan;
% find triangle chains that start and end at this node
idx_chain_starts_at_1_connected_node = find(ismember([triangle_chains{:,1}]', idx_1_connected_nodes));
idx_chain_ends_at_1_connected_node = find(ismember([triangle_chains{:,2}]', idx_1_connected_nodes));
% remove these triangle chains
[triangle_chains{idx_chain_ends_at_1_connected_node,3}] = deal([]);
[triangle_chains{idx_chain_starts_at_1_connected_node,3}] = deal([]);

% plot the graph without dead ends
figure; hold on; box on;
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    % pot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjascency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
    color_idx = color_idx + 1;
end
% TODO for each triangle, get each side length, keep max - data structure of tri max sides, per triangle
% for each triangle chain, keep min of the triangle sides as this is the choke point
return
%% attempt 3d
% close all; clear all; clc;
% load trimesh3d
% trisurf(tri,x,y,z)
% dt = delaunayTriangulation(x,y,z)
% tr = triangulation(dt(:,:),dt.Points)
% trisurf(tri,x,y,z)
% numt = size(tr,1);
% T = (1:numt)';
% neigh = neighbors(tr);
% cc = circumcenter(tr);
cc = incenter(tr);
% nodes = find(~isnan(sum(neigh, 2)));
% xcc = cc(:,1);
% ycc = cc(:,2);
% zcc = cc(:,3);
% idx1 = T < neigh(:,1);
% idx2 = T < neigh(:,2);
% idx3 = T < neigh(:,3);
% neigh = [T(idx1) neigh(idx1,1); T(idx2) neigh(idx2,2); T(idx3) neigh(idx3,3)]';
% figure(1); hold on; box on;
% trisurf(tri,x,y,z)
% hold on
plot3(xcc(neigh), ycc(neigh), zcc(neigh), '-r','LineWidth',1.5)
% plot3(xcc(nodes), ycc(nodes), zcc(nodes), '.k','MarkerSize',30)
% xlabel('Medial Axis of Polygonal Domain','FontWeight','b')
%
%% attepmt planning through triangle graph
all_pts = [xcc, ycc, [1:length(xcc)]', -1*ones(length(xcc),1), zeros(length(xcc),1)];
vgraph = zeros(length(xcc));
neigh_orig = neighbors(tr);
for i = 1:size(neigh_orig,1)
    neigh_list = neigh_orig(i,:);
    neigh_list = neigh_list(~isnan(neigh_list));
    if ~(length(neigh_list) == 2)
        continue
    end
    vgraph(neigh_list(1),neigh_list(2)) = 1;
    vgraph(neigh_list(2),neigh_list(1)) = 1;
end
start = all_pts(end-1,:);
start(end) = 1;
finish = all_pts(end,:);
finish(end) = 1;
all_pts = all_pts(1:end-2,:);

start_for_reachability = start;
start_for_reachability(4) = start(3);
finish_for_reachability = finish;
finish_for_reachability(4) = finish(3);

[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start_for_reachability,finish_for_reachability);
if ~is_reachable
    error('initial mission, prior to edge deletion, is not possible')
end

mode = "xy spatial only";
% mode = 'time or z only';
% mode = "xyz or xyt";
[cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);


[init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);


figure; hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
plot(start(1),start(2),'xg','MarkerSize',6);
plot(finish(1),finish(2),'xr','MarkerSize',6);
leg_str = {'start','finish'};
plot(init_route(:,1),init_route(:,2),'LineWidth',2);
leg_str{end+1} = sprintf('route');
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
end
leg_str{end+1} = 'obstacles';
for i = 1:length(shrunk_polytopes)-1
    leg_str{end+1} = '';
end
legend(leg_str,'Location','best');
