% script_test_voronoi_planning_hill
% test script of planning along voronoi diagram edges while incorporating elevation changes into cost
% this script is WIP and does not use the MedialAxis functions, rather, their contents are explicitly
% stated below and modified to allow for elevations
clear; close all; clc

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

map_idx = 7;
flag_do_plot = 1;
flag_do_animation = 0;
flag_do_plot_slow = 0;
[shrunk_polytopes, start_init, finish_init, resolution_scale] = fcn_util_load_test_map(map_idx, 1);

%% interpolate polytope vertices
distances = diff([[shrunk_polytopes.xv]',[shrunk_polytopes.yv]']); % find side lengths in whole field
min_distance_between_verts = min(sqrt(sum(distances.*distances,2))); % the min of this is the smallest space between features
resolution = resolution_scale*min_distance_between_verts/2; % want even the smallest feature to be bisected
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

figure; box on; hold on; triplot(DT); title('triangulation')
xlabel('x [km]')
ylabel('y [km]')
inside = isInterior(DT); % identify triangles statisfying constriants C (i.e. tris within the boundary and outside polytopes, i.e. free space)
tr = triangulation(DT(inside,:),DT.Points); % keep only the triangles of free space, not the ones in polytopes
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
figure; box on; hold on; triplot(tr); title('triangulation, no interior')
xlabel('x [km]')
ylabel('y [km]')
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
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
xlabel('x [km]')
ylabel('y [km]')
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
triplot(tr,'g')
hold on
plot(xcc(neigh_for_plotting), ycc(neigh_for_plotting), '-r','LineWidth',1.5) % plot approx. medial axis
plot(xcc(nodes), ycc(nodes), '.k','MarkerSize',30) % plot 3 connected triangle circumcenters
plot(x(C'),y(C'),'-b','LineWidth',1.5) % plot constriants (i.e. polytopes)
title('Medial Axis, 3-connected nodes highlighted')
% make a plannable graph from triangulation
% identify the 3 connected triangles
adjacency_matrix = eye(length(nodes)); % set to 1 if chain of 2 connected triangles exists between three connected triangle node(i) and node(j)
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
    % note that the start and end of the triangle chain are "adjacent" in a graph sense
    adjacency_matrix(find(nodes==tris_visited(1)),find(nodes==tris_visited(end))) = 1;
    % flag the direction as explored
    path_is_explored(i,direction) = 1;
end % end direction while loop

% plot the graph on the triangles
figure; hold on; box on; title('medial axis graph overlaid on triangulation')
xlabel('x [km]')
ylabel('y [km]')
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
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
    % plot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjacency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
    color_idx = color_idx + 1;
end
% plot the graph
figure; hold on; box on; title('medial axis graph')
xlabel('x [km]')
ylabel('y [km]')
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
end
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    % plot big markers for the start and end node
    beg_end = [chain_of_note(1) chain_of_note(end)];
    % plot a straight line between them (this is the adjacency graph connection)
    plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
    color_idx = color_idx + 1;
end
prev_triangle_chains = nan; % initialize the previous triangle_chains structure to nothing since there wasn't one before
iterand = 1;
% need to repeat the removal of through nodes and the pruning of dead ends until the triangle_chains structure stops changing
while ~isequal(triangle_chains,prev_triangle_chains)
    %% compute branching factor (connectivity)
    % branching factor is the number of nodes connected to each node
    branching_factor_outbound = sum(adjacency_matrix,2)-1; % number of destination nodes per node (excluding self)
    branching_factor_inbound = [sum(adjacency_matrix,1)-1]'; % number of departing nodes per node (excluding self)
    % need to compare the sum of the rows and the sum of the columns.
    % This will tell us inbound and outbound connections per node in an asymmetric graph
    % if a node has 2 in but 3 out we would want to treat it as 3-connected becuase it does serve that role in one direction
    % i.e. a nodes connectedness is determined by its max connectedness of max{in,out}
    max_branching_factor = max(branching_factor_inbound,branching_factor_outbound);
    %% remove through-put nodes
    idx_2_connected_nodes = find(max_branching_factor == 2); % all two connected nodes are through nodes
    false_through_nodes = [];
    % a false through node has 2 possible destinations but more than 1 possible path to at least one of these destinations
    % it proved difficult to iterate over nodes with 2 connected triangle chains directly
    % so instead we iterate over nodes with 2 possible destinations and
    % allowlist the subset of these nodes with 2 possible destinations but more than 2 triangle chains
    % these are the so-called false through nodes that must be excluded from the through nodes that will be removed
    prev_triangle_chains = triangle_chains; % store the current triangle chain structure before we modify it
    while ~isempty(idx_2_connected_nodes)
        % for each through node, t...
        t = idx_2_connected_nodes(1);
        adjacent_to_t = adjacency_matrix(t,:); % find t's adjacent nodes
        adjacent_to_t(t) = 0; % don't need self adjacency for this
        % find the node on either side...call these d and b
        d_and_b = find(adjacent_to_t==1);
        d = d_and_b(1);
        b = d_and_b(2);
        if flag_do_plot_slow
            % plot the through node being removed and its neighbors
            plot(xcc(nodes(t)), ycc(nodes(t)), '.b','MarkerSize',30) % plot 3 connected triangle circumcenters
            plot(xcc(nodes(b)), ycc(nodes(b)), '.g','MarkerSize',30) % plot 3 connected triangle circumcenters
            plot(xcc(nodes(d)), ycc(nodes(d)), '.g','MarkerSize',30) % plot 3 connected triangle circumcenters
        end
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
        % need to check for a false through node (a node with two possible destinations but more than two possible paths)
        if (length(idx_chain_dt) > 1 | length(idx_chain_tb) > 1 | length(idx_chain_bt) > 1 | length(idx_chain_td) > 1)
            false_through_nodes = [false_through_nodes, t];
            % re-compute branching factor (connectivity)
            branching_factor_outbound = sum(adjacency_matrix,2)-1; % number of destination nodes per node (excluding self)
            branching_factor_inbound = [sum(adjacency_matrix,1)-1]'; % number of departing nodes per node (excluding self)
            max_branching_factor = max(branching_factor_inbound,branching_factor_outbound);
            idx_2_connected_nodes = find(max_branching_factor == 2); % all two connected nodes are through nodes
            % need to remove the allowlisted false through nodes from the list of 2 connected nodes
            is_false_through_node = ismember(idx_2_connected_nodes,false_through_nodes); % boolean array of which 2 connected nodes are false through nodes
            idx_2_connected_nodes = idx_2_connected_nodes(~is_false_through_node); % only keep idx of 2 connected nodes that aren't false through nodes
            continue % don't want to remove a false through node since it affords multiple paths to the same destination
        end
        % connect those two nodes in the adjacency matrix
        adjacency_matrix(d_and_b, d_and_b) = 1; % note this line makes Adb, Abd, Add, and Abb =1
        % make an entry for d to b and set tri list to the other two tri lists
        triangle_chains{end+1,1} = d; % index of start in nodes
        triangle_chains{end,2} = b; % index of end in nodes
        triangle_chains{end,3} = [chain_dt(1:end-1), chain_tb]; % list of triangles between them
        % do this again for reverse direction
        triangle_chains{end+1,1} = b; % index of start in nodes
        triangle_chains{end,2} = d; % index of end in nodes
        triangle_chains{end,3} = [chain_bt(1:end-1), chain_td]; % list of triangles between them
        % delete the rows for d to t and t to b
        [triangle_chains{idx_chain_dt,3}] = deal([]);
        [triangle_chains{idx_chain_tb,3}] = deal([]);
        % do this again for reverse direction
        [triangle_chains{idx_chain_bt,3}] = deal([]);
        [triangle_chains{idx_chain_td,3}] = deal([]);
        %  remove the t node from the adjacency matrix
        adjacency_matrix(t, :) = zeros(1, size(adjacency_matrix,1));
        adjacency_matrix(:, t) = zeros(size(adjacency_matrix,2), 1);
        % remove from node list
        nodes(t) = nan;

        %% re-compute branching factor (connectivity)
        branching_factor_outbound = sum(adjacency_matrix,2)-1; % number of destination nodes per node (excluding self)
        branching_factor_inbound = [sum(adjacency_matrix,1)-1]'; % number of departing nodes per node (excluding self)
        max_branching_factor = max(branching_factor_inbound,branching_factor_outbound);
        idx_2_connected_nodes = find(max_branching_factor == 2); % all two connected nodes are through nodes
        % need to remove the allowlisted false through nodes from the list of 2 connected nodes
        is_false_through_node = ismember(idx_2_connected_nodes,false_through_nodes); % boolean array of which 2 connected nodes are false through nodes
        idx_2_connected_nodes = idx_2_connected_nodes(~is_false_through_node); % only keep idx of 2 connected nodes that aren't false through nodes

        if flag_do_plot_slow
            % plot the graph after this through node removal
            figure; hold on; box on;
            xlabel('x [km]')
            ylabel('y [km]')
            for j = 2:length(shrunk_polytopes)
                fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
            end
            for i = 1:(size(triangle_chains,1))
                % pop off a triangle chain
                chain_of_note = triangle_chains{i,3};
                if isempty(chain_of_note)
                    continue
                end
                % plot big markers for the start and end node
                beg_end = [chain_of_note(1) chain_of_note(end)];
                % plot a straight line between them (this is the adjacency graph connection)
                plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
                % plot the medial axis path between them (this is the curved path from the triangle chain)
                plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
                color_idx = color_idx + 1;
            end
        end
    end
    % plot the graph without through nodes
    figure; hold on; box on; title('medial axis graph with through nodes removed')
    for j = 2:length(shrunk_polytopes)
        fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    end
    for i = 1:(size(triangle_chains,1))
        % pop off a triangle chain
        chain_of_note = triangle_chains{i,3};
        if isempty(chain_of_note)
            continue
        end
        % plot big markers for the start and end node
        beg_end = [chain_of_note(1) chain_of_note(end)];
        % plot a straight line between them (this is the adjacency graph connection)
        plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
        % plot the medial axis path between them (this is the curved path from the triangle chain)
        plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
        color_idx = color_idx + 1;
    end
    % TODO @sjharnett do we want to set these to zero/empty or actually remove them? Removing would require re-indexing
    %% remove dead ends
    idx_1_connected_nodes = find(max_branching_factor == 1); % all one connected nodes are dead ends
    % remove the node from the adjacency matrix
    adjacency_matrix(idx_1_connected_nodes, :) = zeros(length(idx_1_connected_nodes), size(adjacency_matrix,1));
    adjacency_matrix(:, idx_1_connected_nodes) = zeros(size(adjacency_matrix,2), length(idx_1_connected_nodes));
    % remove the node from the node list
    nodes(idx_1_connected_nodes) = nan;
    % find triangle chains that start and end at this node
    idx_chain_starts_at_1_connected_node = find(ismember([triangle_chains{:,1}]', idx_1_connected_nodes));
    idx_chain_ends_at_1_connected_node = find(ismember([triangle_chains{:,2}]', idx_1_connected_nodes));
    % remove these triangle chains
    [triangle_chains{idx_chain_ends_at_1_connected_node,3}] = deal([]);
    [triangle_chains{idx_chain_starts_at_1_connected_node,3}] = deal([]);

    % plot the graph without dead ends
    figure; hold on; box on; title('medial axis graph with dead ends removed')
    xlabel('x [km]')
    ylabel('y [km]')
    for j = 2:length(shrunk_polytopes)
        fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    end
    for i = 1:(size(triangle_chains,1))
        % pop off a triangle chain
        chain_of_note = triangle_chains{i,3};
        if isempty(chain_of_note)
            continue
        end
        % plot big markers for the start and end node
        beg_end = [chain_of_note(1) chain_of_note(end)];
        % plot a straight line between them (this is the adjacency graph connection)
        plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
        % plot the medial axis path between them (this is the curved path from the triangle chain)
        plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
        color_idx = color_idx + 1;
    end
    if flag_do_plot_slow % plot entire adjacency graph for debugging
        [r, c] = find(adjacency_matrix);
        for j = 1:length(r)
            idx_chain_rc = find([triangle_chains{:,1}]'== r(j) & [triangle_chains{:,2}]'== c(j));
            if length(idx_chain_rc)>=1
                plot(xcc(nodes([r((j)) c((j))])), ycc(nodes([r((j)) c((j))])), '--.','MarkerSize',10,'Color','g')
            else
                plot(xcc(nodes([r((j)) c((j))])), ycc(nodes([r((j)) c((j))])), '--.','MarkerSize',10,'LineWidth',3,'Color','r')
            end
        end
    end
    sprintf('loop has iterated %i times',iterand)
    iterand = iterand + 1;
end % end outer while loop that loops until convergence occurs

%% get costs for navigating each triangle chain
% to get corridor width:
% (1) need to get triangle side lengths
% (2) need to store the max side length of each triangle
% (3) need to store the min of these max side lengths for each triangle chain
%% get max length side for each triangle
max_side_lengths_per_tri = nan(size(tr,1),1); % initialize array to store max side of each tri
for my_tri = 1:size(tr,1) % loop over each triangle in the triangulation
    my_connectivity = tr.ConnectivityList(my_tri,:); % find the 3 point IDs forming the tri
    my_points = tr.Points(my_connectivity,:); % extract the points forming the tri
    my_verts = [my_points; my_points(1,:)]; % duplicate first point to close the triangle
    side_deltas = diff([my_verts(:,1) my_verts(:,2)]); % find x and y difference between each vertex
    side_lengths = sqrt(side_deltas(:,1).^2 + side_deltas(:,2).^2); % find length from deltaX and deltaY
    max_side_lengths_per_tri(my_tri) = max(side_lengths); % only need to store max length
end
%% store route segment length and corridor width for each triangle chain and elev change
zcc = f_peaks(xcc(1:size(tr,1)),ycc(1:size(tr,1)));
for i = 1:(size(triangle_chains,1)) % for each triangle chain
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note) % if the chain was deleted during pruning we skip it
        triangle_chains{i,4} = nan; % just store nan in the corridor width column
        triangle_chains{i,5} = nan; % just store nan in the chain length column
        triangle_chains{i,6} = nan; % just store nan in the chain length column
        continue
    end
    % the min of the triangle "sizes" should be the tightest choke in that triangle chain
    triangle_chains{i,4} = min(max_side_lengths_per_tri(chain_of_note));
    % the cumulative length of all the distance between circumcenters is the triangle chain length
    delta_x_and_y = diff([xcc(chain_of_note) ycc(chain_of_note)]);
    triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
    triangle_chains{i,5} = triangle_chain_length;
    % the following gives change in elevation as a cost
    % delta_z = zcc(chain_of_note(end)) - zcc(chain_of_note(1));
    % triangle_chains{i,6} = delta_z;
    % the following gives average pitch as a cost
    deltaZ = diff(zcc(chain_of_note)); % the ith element contains the change in height from i+1 to i
    avg_pitch = sum(deltaZ)/triangle_chain_length;
    if avg_pitch<0
        avg_pitch = 0; % don't reward descending as much as we punish ascending
    end
    triangle_chains{i,6} = avg_pitch;
end

figure; hold on; box on; title('medial axis graph with corridor width expressed')
xlabel('x [km]')
ylabel('y [km]')
corridor_widths = [triangle_chains{:,4}]';
corridor_widths(isnan(corridor_widths)) = [];
max_corridor_width = 0.6*max(corridor_widths);
min_corridor_width = min(corridor_widths);
my_colormap = colormap(turbo);
num_colors = size(my_colormap,1);
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    width_of_note = triangle_chains{i,4};
    if width_of_note > max_corridor_width
        width_of_note = max_corridor_width;
    end
    width_portion = (width_of_note-min_corridor_width)/(max_corridor_width-min_corridor_width);
    if width_portion > 1
        width_portion = 1;
    end
    width_portion_color_idx = round(width_portion*num_colors,0); % convert width_portion to an index in colormap
    if width_portion_color_idx == 0
        width_portion_color_idx = 1; % matlab is 1 indexed
    end
    width_color = my_colormap(width_portion_color_idx,:);
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',width_color)
end
set(gca,'CLim',[min_corridor_width max_corridor_width]);
c = colorbar;
xlabel('x [km]')
ylabel('y [km]')
ylabel(c,'corridor width [km]')
plot(xcc(nodes(~isnan(nodes))), ycc(nodes(~isnan(nodes))), '.k','MarkerSize',20) % plot 3 connected triangle circumcenters
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
end

%% get elevation data
figure; hold on; box on; title('medial axis graph with elevation expressed')
elevs = unique(zcc);
max_elev = max(elevs);
min_elev = min(elevs);
my_colormap = colormap(turbo);
num_colors = size(my_colormap,1);
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    dense_x = xcc(chain_of_note(1)):0.1:xcc(chain_of_note(end));
    dense_y = interp1(xcc(chain_of_note), ycc(chain_of_note), dense_x);
    dense_z = f_peaks(dense_x, dense_y);
    elev_portion = (dense_z-min_elev)./(max_elev-min_elev);
    elev_portion_color_idx = round(elev_portion*num_colors,0); % convert elev_portion to an index in colormap
    elev_portion_color_idx(elev_portion_color_idx == 0) = 1;
    elev_portion_color_idx(elev_portion_color_idx > num_colors) = num_colors;
    for j = 1:length(dense_x)
        x = dense_x(j);
        y = dense_y(j);
        z = dense_z(j);
        elev_color = my_colormap(elev_portion_color_idx(j),:);
        plot3(x,y,z, '.','MarkerSize',10,'Color',elev_color)
    end
end
set(gca,'CLim',[min_elev max_elev]);
c = colorbar;
xlabel('x [km]')
ylabel('y [km]')
zlabel('elevation [m]')
plot3(xcc(nodes(~isnan(nodes))), ycc(nodes(~isnan(nodes))),zcc(nodes(~isnan(nodes))), '.k','MarkerSize',20) % plot 3 connected triangle circumcenters
ylabel(c,'elevation [m]')
% for j = 2:length(shrunk_polytopes)
%     fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
% end

figure; hold on; box on; title('medial axis graph with avg. pitch cost expressed')
xlabel('x [km]')
ylabel('y [km]')
avg_pitches = [triangle_chains{:,6}]';
avg_pitches(isnan(avg_pitches)) = [];
max_avg_pitch = 0.6*max(avg_pitches);
min_avg_pitch = min(avg_pitches);
my_colormap = colormap(turbo);
num_colors = size(my_colormap,1);
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    pitch_of_note = triangle_chains{i,6};
    if pitch_of_note > max_avg_pitch
        pitch_of_note = max_avg_pitch;
    end
    pitch_portion = (pitch_of_note-min_avg_pitch)/(max_avg_pitch-min_avg_pitch);
    if pitch_portion > 1
        pitch_portion = 1;
    end
    pitch_portion_color_idx = round(pitch_portion*num_colors,0); % convert pitch_portion to an index in colormap
    if pitch_portion_color_idx == 0
        pitch_portion_color_idx = 1; % matlab is 1 indexed
    end
    pitch_color = my_colormap(pitch_portion_color_idx,:);
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',pitch_color)
end
set(gca,'CLim',[min_avg_pitch max_avg_pitch]);
c = colorbar;
xlabel('x [km]')
ylabel('y [km]')
ylabel(c,'average pitch [km]')
plot(xcc(nodes(~isnan(nodes))), ycc(nodes(~isnan(nodes))), '.k','MarkerSize',20) % plot 3 connected triangle circumcenters
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
end

figure; hold on; box on; title('medial axis graph with route segment length expressed')
lengths = [triangle_chains{:,5}]';
lengths(isnan(lengths)) = [];
max_length = max(lengths);
min_length = min(lengths);
my_colormap = colormap(turbo);
num_colors = size(my_colormap,1);
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if isempty(chain_of_note)
        continue
    end
    length_of_note = triangle_chains{i,5};
    if length_of_note > max_length
        length_of_note = max_length;
    end
    length_portion = (length_of_note-min_length)/(max_length-min_length);
    if length_portion > 1
        length_portion = 1;
    end
    length_portion_color_idx = round(length_portion*num_colors,0); % convert length_portion to an index in colormap
    if length_portion_color_idx == 0
        length_portion_color_idx = 1; % matlab is 1 indexed
    end
    length_color = my_colormap(length_portion_color_idx,:);
    % plot the medial axis path between them (this is the curved path from the triangle chain)
    plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',length_color)
end
set(gca,'CLim',[min_length max_length]);
c = colorbar;
xlabel('x [km]')
ylabel('y [km]')
plot(xcc(nodes(~isnan(nodes))), ycc(nodes(~isnan(nodes))), '.k','MarkerSize',20) % plot 3 connected triangle circumcenters
ylabel(c,'path segment length [km]')
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
end

%% planning through triangle graph
% pick a start and finish
% start = all_pts(24,:);
% finish = all_pts(104,:);
% find the xcc,ycc pair closest to start
% start_xy = [1031.5 -4715.4];
start_xy = start_init(1,:)%+ [0 10];%[1031 -4717];
finish_xy = finish_init(1,:)%+ [0 10];%[1050 -4722];
tris_in_graph = unique([triangle_chains{:,3}]');
start_delta_from_all_tris = start_xy - [xcc(tris_in_graph), ycc(tris_in_graph)];
start_dist_from_all_tris = (start_delta_from_all_tris(:,1).^2 + start_delta_from_all_tris(:,2).^2).^0.5;
[~, start_closest_tri_loc] = min(start_dist_from_all_tris);
start_closest_tri = tris_in_graph(start_closest_tri_loc);
% find which chains the closest tri is in (should really only be 2)
idx_chains_containing_start = [];
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if ismember(start_closest_tri, chain_of_note)
        idx_chains_containing_start = [idx_chains_containing_start i];
    end
end
% make start closest tri a node
nodes = [nodes; start_closest_tri];
start_closest_node = find(nodes == start_closest_tri);
% make a new adjacency matrix row and column for the start triangle
adjacency_matrix = [adjacency_matrix, zeros(size(adjacency_matrix,2),1); zeros(1,size(adjacency_matrix,1)+1)];
for i = 1:length(idx_chains_containing_start)
    % pop off the triangle chain containing the start triangle
    first_node = triangle_chains{idx_chains_containing_start(i),1};
    last_node = triangle_chains{idx_chains_containing_start(i),2};
    chain_of_note = triangle_chains{idx_chains_containing_start(i),3};
    % find where the start triangle is in the chain
    start_tri_location = find(chain_of_note == start_closest_tri);
    % make two new chains from beginning to start tri and start tri to end
    first_chain = chain_of_note(1:start_tri_location);
    last_chain = chain_of_note(start_tri_location:end);
    triangle_chains{end+1,1} = first_node;
    triangle_chains{end,2} = start_closest_node;
    triangle_chains{end,3} = first_chain;
    triangle_chains{end,4} = min(max_side_lengths_per_tri(first_chain));
    % the cumulative length of all the distance between circumcenters is the triangle chain length
    delta_x_and_y = diff([xcc(first_chain) ycc(first_chain)]);
    triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
    triangle_chains{end,5} = triangle_chain_length;
    % TODO the following gives change in elevation as a cost
    % delta_z = zcc(first_chain(end)) - zcc(first_chain(1));
    % triangle_chains{end,6} = delta_z;
    % the following gives average pitch as a cost
    deltaZ = diff(zcc(first_chain)); % the ith element contains the change in height from i+1 to i
    avg_pitch = sum(deltaZ)/triangle_chain_length;
    if avg_pitch<0
        avg_pitch = 0; % don't reward descending as much as we punish ascending
    end
    triangle_chains{end,6} = avg_pitch;
    triangle_chains{end+1,1} = start_closest_node;
    triangle_chains{end,2} = last_node;
    triangle_chains{end,3} = last_chain;
    triangle_chains{end,4} = min(max_side_lengths_per_tri(last_chain));
    % the cumulative length of all the distance between circumcenters is the triangle chain length
    delta_x_and_y = diff([xcc(last_chain) ycc(last_chain)]);
    triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
    triangle_chains{end,5} = triangle_chain_length;
    % TODO the following gives change in elevation as a cost
    % delta_z = zcc(last_chain(end)) - zcc(last_chain(1));
    % triangle_chains{end,6} = delta_z;
    % the following gives average pitch as a cost
    deltaZ = diff(zcc(last_chain)); % the ith element contains the change in height from i+1 to i
    avg_pitch = sum(deltaZ)/triangle_chain_length;
    if avg_pitch<0
        avg_pitch = 0; % don't reward descending as much as we punish ascending
    end
    triangle_chains{end,6} = avg_pitch;
    % add the new chains to adjacency
    adjacency_matrix(start_closest_node,start_closest_node) = 1;
    adjacency_matrix(first_node,start_closest_node) = 1;
    adjacency_matrix(start_closest_node,last_node) = 1;
end
% do the same with the finish
finish_delta_from_all_tris = finish_xy - [xcc(tris_in_graph), ycc(tris_in_graph)];
finish_dist_from_all_tris = (finish_delta_from_all_tris(:,1).^2 + finish_delta_from_all_tris(:,2).^2).^0.5;
[~, finish_closest_tri_loc] = min(finish_dist_from_all_tris);
finish_closest_tri = tris_in_graph(finish_closest_tri_loc);
% find which chains the closest tri is in (should really only be 2)
idx_chains_containing_finish = [];
for i = 1:(size(triangle_chains,1))
    % pop off a triangle chain
    chain_of_note = triangle_chains{i,3};
    if ismember(finish_closest_tri, chain_of_note)
        idx_chains_containing_finish = [idx_chains_containing_finish i];
    end
end
% make finish closest tri a node
nodes = [nodes; finish_closest_tri];
finish_closest_node = find(nodes == finish_closest_tri);
% make a new adjacency matrix row and column for the finish triangle
adjacency_matrix = [adjacency_matrix, zeros(size(adjacency_matrix,2),1); zeros(1,size(adjacency_matrix,1)+1)];
for i = 1:length(idx_chains_containing_finish)
    % pop off the triangle chain containing the finish triangle
    first_node = triangle_chains{idx_chains_containing_finish(i),1};
    last_node = triangle_chains{idx_chains_containing_finish(i),2};
    chain_of_note = triangle_chains{idx_chains_containing_finish(i),3};
    % find where the finish triangle is in the chain
    finish_tri_location = find(chain_of_note == finish_closest_tri);
    % make two new chains from beginning to finish tri and finish tri to end
    first_chain = chain_of_note(1:finish_tri_location);
    last_chain = chain_of_note(finish_tri_location:end);
    triangle_chains{end+1,1} = first_node;
    triangle_chains{end,2} = finish_closest_node;
    triangle_chains{end,3} = first_chain;
    triangle_chains{end,4} = min(max_side_lengths_per_tri(first_chain));
    % the cumulative length of all the distance between circumcenters is the triangle chain length
    delta_x_and_y = diff([xcc(first_chain) ycc(first_chain)]);
    triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
    triangle_chains{end,5} = triangle_chain_length;
    % TODO the following gives change in elevation as a cost
    % delta_z = zcc(first_chain(end)) - zcc(first_chain(1));
    % triangle_chains{end,6} = delta_z;
    % the following gives average pitch as a cost
    deltaZ = diff(zcc(first_chain)); % the ith element contains the change in height from i+1 to i
    avg_pitch = sum(deltaZ)/triangle_chain_length;
    if avg_pitch<0
        avg_pitch = 0; % don't reward descending as much as we punish ascending
    end
    triangle_chains{end,6} = avg_pitch;
    triangle_chains{end+1,1} = finish_closest_node;
    triangle_chains{end,2} = last_node;
    triangle_chains{end,3} = last_chain;
    triangle_chains{end,4} = min(max_side_lengths_per_tri(last_chain));
    % the cumulative length of all the distance between circumcenters is the triangle chain length
    delta_x_and_y = diff([xcc(last_chain) ycc(last_chain)]);
    triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
    triangle_chains{end,5} = triangle_chain_length;
    % TODO the following gives change in elevation as a cost
    % delta_z = zcc(last_chain(end)) - zcc(last_chain(1));
    % triangle_chains{end,6} = delta_z;
    % the following gives average pitch as a cost
    deltaZ = diff(zcc(last_chain)); % the ith element contains the change in height from i+1 to i
    avg_pitch = sum(deltaZ)/triangle_chain_length;
    if avg_pitch<0
        avg_pitch = 0; % don't reward descending as much as we punish ascending
    end
    triangle_chains{end,6} = avg_pitch;
    % add the new chains to adjacency
    adjacency_matrix(finish_closest_node,finish_closest_node) = 1;
    adjacency_matrix(first_node,finish_closest_node) = 1;
    adjacency_matrix(finish_closest_node,last_node) = 1;
end
% make all pts array from nodes
num_nodes = length(nodes);
all_pts = nan(num_nodes,3);
for i = 1:num_nodes
    if isnan(nodes(i))
        continue
    end
    all_pts(i,:) = [xcc(nodes(i)), ycc(nodes(i)), i];
end
%% form cost graph from triangle_chains
% cost is of the form: total cost = w*length + (1-w)*corridor_width
for w = 1%:0.1:1
cgraph = nan(size(adjacency_matrix)); % initialize cgraph
% since there can be multiple chains between two nodes, we need to note which one we are using
best_chain_idx_matrix = nan(size(adjacency_matrix));
% for every one in the adjacency matrix, i.e., every connected pair of nodes
[r, c] = find((adjacency_matrix));
for i = 1:length(r)
    % if this is the self adjacent node...
    if r(i) == c(i)
        cgraph(r(i),c(i)) = 0; % it's always free to stay still
        continue
    end
    % find all the chains connecting r and c in adjacency
    idx_chain_rc = find([triangle_chains{:,1}]'== r(i) & [triangle_chains{:,2}]'== c(i));
    % we want to only use the chain with the lowest total cost form r to c
    corridor_widths = [triangle_chains{idx_chain_rc, 4}]; % the corridor width of all valid chains
    lengths = [triangle_chains{idx_chain_rc, 5}]; % the length of all valid chains
    elev_change = [triangle_chains{idx_chain_rc, 6}]; % the length of all valid chains
    possible_costs = w*lengths + (1-w)*(corridor_widths).^(-1)+0.02*elev_change; % vectorized total cost
    [min_cost, min_cost_location] = min(possible_costs); % the min cost is what we use as cost
    cgraph(r(i),c(i)) = min_cost;
    best_chain_idx_matrix(r(i),c(i)) = idx_chain_rc(min_cost_location); % need to remember which chain we want to use
end

%  set the start for the planner as the start node not the startxy
start = [xcc(start_closest_tri) ycc(start_closest_tri) start_closest_node];
finish = [xcc(finish_closest_tri) ycc(finish_closest_tri) finish_closest_node];

% adjacency matrix is vgraph
vgraph = adjacency_matrix;
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

% take route and tri chains data structure
% also take best path structure
route_triangle_chain = [];
route_choke = inf;
for i = 1:(size(route,1)-1)
    % for route to route + 1 get tri chain
    beg_seg = route(i,3);
    end_seg = route(i+1,3);
    idx_chain = find([triangle_chains{:,1}]'== beg_seg & [triangle_chains{:,2}]'== end_seg);
    % if there's none, error
    if isempty(idx_chain)
        error('no triangle chain exists for this route segment')
    % elseif length(idx_chain) == 1
    %     route_triangle_chain = [route_triangle_chain, triangle_chains{chain_idx,3}];
    %     % coule extract length and min width here
    % if there's two take best
    % if there's one, take it
    else
        best_chain_idx = best_chain_idx_matrix(beg_seg,end_seg);
        % append to list of triangle chains
        route_triangle_chain = [route_triangle_chain, triangle_chains{best_chain_idx,3}];
        segment_choke = triangle_chains{best_chain_idx,4};
        route_choke = min(route_choke, segment_choke);
    end
end
% dedup
route_triangle_chain = unique(route_triangle_chain,'stable');
% append the straightline from startxy to start node to the beginning of the route when transforming the route to tri chains
route_full = [start_xy; xcc(route_triangle_chain), ycc(route_triangle_chain); finish_xy];
route_x = route_full(:,1);
route_y = route_full(:,2);
route_deltas = diff([route_x(:) route_y(:)]);
route_length = sum(sqrt(sum(route_deltas.*route_deltas,2)));

% plot result
figure; hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
leg_str = {};
leg_str{end+1} = 'medial axis graph';
not_first = 0;
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
end

% plot result
figure; hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
leg_str = {};
leg_str{end+1} = 'medial axis graph';
not_first = 0;
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
% plot polytopes with elevation colors
elevs = unique(zcc);
max_elev = max(elevs);
min_elev = min(elevs);
my_colormap = colormap(turbo);
num_colors = size(my_colormap,1);
for j = 2:length(shrunk_polytopes)
    poly_avg_z = f_peaks(shrunk_polytopes(j).mean(1),shrunk_polytopes(j).mean(2));
    if poly_avg_z > max_elev
        poly_avg_z = max_elev;
    end
    elev_portion = (poly_avg_z-min_elev)./(max_elev-min_elev);
    elev_portion_color_idx = round(elev_portion*num_colors,0); % convert elev_portion to an index in colormap
    elev_portion_color_idx(elev_portion_color_idx <= 0) = 1;
    elev_color = my_colormap(elev_portion_color_idx,:);
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),elev_color,'FaceAlpha',0.3)
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
set(gca,'CLim',[min_elev max_elev]);
c = colorbar;
ylabel(c,'elevation [m]')

figure; hold on; box on;
x = -1:1:40;
y = x;
[X,Y] = meshgrid(x,y);
Z = f_peaks(X,Y);
surf(X,Y,Z);
route_full_z = f_peaks(route_full(:,1),route_full(:,2));
plot3(route_full(:,1),route_full(:,2),route_full_z,'-r','LineWidth',2.5);
xlabel('x [km]')
ylabel('y [km]')
zlabel('elevation [m]')

% function to generate a parabolic hill and give elevation values z for x,y pairs
function z = f_saddle(x,y)
    a = 0.1; % x scale
    b = 0.1; % y scale
    xc = 15; % x center
    yc = 20; % y center
    z = -(((x-xc).^2)/a - ((y-yc).^2)/b) + 1000;
end
function z = f_peaks(x,y)
    z = 100*(peaks((x-20)/10,(y-15+10)/10)+10);
end
