function [adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(polytopes, resolution_scale, varargin)
% fcn_MedialAxis_makeCostGraphAndAllPoints
%
% This function forms the medial axis graph (composed of an adjacency matrix and
% the triangle_chains structure describing the edges) from a polytopes struct
% array.  The free space between the polytope obstacles is triangulated.  The
% circumcenters of the triangles are connected to form the medial axes.  The
% 3-connected triangles (i.e., where the medial axes branch) are considered nodes.
% The adjacency matrix tells which nodes are connected by medial axis edges. The
% triangle_chains data structure describes these edges.
%
% FORMAT:
%
% [adjacency_matrix, triangle_chains, nodes, xcc, ycc, tr] = fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains(polytopes, resolution_scale, varargin)
%
%
% INPUTS:
%
%   polytopes: the polytope struct array
%
%   resolution_scale: the multiplier for the minimum spacing between points for triangulation.
%       The medial axis graph is formed using Delaunay triangulation.  The default spacing of points
%       on obstacles used for forming triangles is half the minimum spacing between vertices (thus
%       even the smallest feature will be composed of 2 triangles).  In many cases this is too conservative
%       so the resolution_scale input allows the user to modify this default spacing.
%
%   (optional arguments)
%   flag_do_plot: a 1 or 0 for flagging plotting on or off.  If ommitted, it is assumed to be 0.
%
% OUTPUTS:
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    adjacency_matrix: Like the visibility graph but rather than indicating 2 nodes are visible,
%       it indicates 2 nodes are connected by an edge.
%       This an NxN matrix where N is the number of nodes in the map.
%       A 1 is in position i,j if node j is visible from point i.  0 otherwise.
%
%    triangle_chains: an Mx3 cell array with a row for each edge in the medial axis graph.  The first
%      column contains an int for the node ID for the start of the chain.  The second is the end node.
%      The third column is a 1xP_M array of integers representing IDs of the triangles whose circumcenters
%      form the "chain of triangles" connecting the two nodes. P_M can be different for each row, M.
%
%    nodes: a Nx1 array of integers.  The integers are the IDs of the triangles that are 3-connected,
%      i.e., their circumcenters are nodes in the medial axis graph.  The position in the nodes array
%      is the node ID and the value is the triangle ID.  E.g., if nodes(10)=146, then the 10th node
%      in the adjacency_matrix and triangle_chains struct is the 146th triangle in
%      the Delaunay triangulation.
%
%    xcc: Qx1 array of doubles.  The x positions of the circumcenters of the triangles.
%
%    ycc: Qx1 array of doubles.  The y positions of the circumcenters of the triangles.
%
%    tr: Qx3 triangulation.  See: https://www.mathworks.com/help/matlab/ref/triangulation.html#d126e1402901
%
% DEPENDENCIES:
%
%  -  fcn_MapGen_increasePolytopeVertexCount
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning* for examples of the script in use.
%        script_test_voronoi_planning - basic example of medial axis planning
%        ||_alt_paths - example of generating several paths from the start to the finish using different corridors
%        ||_alt_paths_from_node - example of generating several paths from an arbitrary node to the finish using different corridors
%        ||_alt_paths_local - example of generating several paths from an each node along the initial route to the finish.  This
%                             script has a flag for which corridors are blocked on replanning: just the next segment in the
%                             initial route, the entire initial route, or all previously calculated routes (initial and alternate)
%        ||_hill - example of incorporating elevation into a medial axis graph.  This script is just a WIP demonstration
% See ../Documentation/medial_axis_planning.pptx for a flow chart of the medial axis/voronoi planning stack
%
% This function was written Spring 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024, Spring by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.

    %% check input arguments
    if nargin < 2 || nargin > 3
        error('Incorrect number of arguments');
    end
    % if there is no value in varargin...
    if nargin == 2
        % default is to assume convex obstacles as this is conservative
        flag_do_plot = 0;
    end
    % if there is a value in varargin...
    if nargin == 3
        % check what it is
        if varargin{1} == 1
            % set concave flag if it was passed in
            flag_do_plot = 1;
        elseif varargin{1} == 0
            flag_do_plot = 0;
        else
            % throw error if it was passed in with an incorrect value
            error('optional argument is the plotting flag and can either be 1 or 0')
        end
    end

    %% interpolate polytope vertices
    distances = diff([[polytopes.xv]',[polytopes.yv]']); % find side lengths in whole field
    min_distance_between_verts = min(sqrt(sum(distances.*distances,2))); % the min of this is the smallest space between features
    resolution = resolution_scale*min_distance_between_verts/2; % want even the smallest feature to be bisected
    polytopes = fcn_MapGen_increasePolytopeVertexCount(polytopes, resolution); % interpolate sides

    C = []; % initialize constriant matrix
    P = []; % initialize points matrix
    largest_idx = 0;
    for poly_idx = 1:length(polytopes)
        P = [P; polytopes(poly_idx).vertices(1:end-1,:)]; % add poly verts to points
        % want to list every pair of points representing a polytope side as a constraint
        num_verts = size(polytopes(poly_idx).vertices,1)-1;
        C1 = [1:num_verts]'; % this will be points 1 through n
        C2 = [C1(2:end);C1(1)]; % C2 is equal to C1 with the order shifted by 1
        Ccomb = [C1 C2]; % this will be a table saying point 1 to 2 is a side, 2 to 3, etc.
        Ccomb = Ccomb + largest_idx; % shift the whole thing by the largest ID already in C so point IDs are global, not unique to this polytope onlyk
        C = [C; Ccomb]; % add constriants for this polytope to total constraint list
        largest_idx = max(max(C)); % update largest point ID in this polytope for use offsetting next polytope
    end
    x = P(:,1); % all x's
    y = P(:,2); % all y's
    if flag_do_plot
        DT = delaunayTriangulation(P,C) % perform constrained triangulation
        figure; box on; hold on; triplot(DT); title('triangulation')
        xlabel('x [km]')
        ylabel('y [km]')
    else
        DT = delaunayTriangulation(P,C);
    end

    inside = isInterior(DT); % identify triangles statisfying constriants C (i.e. tris within the boundary and outside polytopes, i.e. free space)
    tr = triangulation(DT(inside,:),DT.Points); % keep only the triangles of free space, not the ones in polytopes
    if flag_do_plot
        for j = 2:length(polytopes)
            fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
        end
        figure; box on; hold on; triplot(tr); title('triangulation, no interior')
        xlabel('x [km]')
        ylabel('y [km]')
        for j = 2:length(polytopes)
            fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
        end
    end
    numt = size(tr,1); % numbe of triangles
    T = (1:numt)'; % list of triangle idx
    neigh = tr.neighbors(); % this indicates which triangles are connected to which other triangles
    cc = circumcenter(tr); % get all circumcenters (centers of circumscribed circles)
    nodes = find(~isnan(sum(neigh, 2))); % identify all 3 connected triangles (tris with no nan neighbor)
    xcc = cc(:,1); % x coords of circumcenters
    ycc = cc(:,2); % y coords of circumcenters
    % the following code rearranges the Qx3 'neigh' matrix where row i is the three neighbors of tri i,
    % into a 2x? matrix where each column is a pair of neighboring triangles
    % this is useful for plotting as neigh_for_plotting has no nan values while neigh does
    idx1 = T < neigh(:,1);
    idx2 = T < neigh(:,2);
    idx3 = T < neigh(:,3);
    neigh_for_plotting = [T(idx1) neigh(idx1,1); T(idx2) neigh(idx2,2); T(idx3) neigh(idx3,3)]';
    if flag_do_plot
                % plot the triangulation and approximate medial axis
        figure; hold on; box on;
        xlabel('x [km]')
        ylabel('y [km]')
        for j = 2:length(polytopes)
            fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
        end
        triplot(tr,'g')
        hold on
        plot(xcc(neigh_for_plotting), ycc(neigh_for_plotting), '-r','LineWidth',1.5) % plot approx. medial axis
        plot(x(C'),y(C'),'-b','LineWidth',1.5) % plot constriants (i.e. polytopes)
        title('Medial Axis')
        % plot the triangulation and approximate medial axis
        figure; hold on; box on;
        xlabel('x [km]')
        ylabel('y [km]')
        for j = 2:length(polytopes)
            fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
        end
        triplot(tr,'g')
        hold on
        plot(xcc(neigh_for_plotting), ycc(neigh_for_plotting), '-r','LineWidth',1.5) % plot approx. medial axis
        plot(xcc(nodes), ycc(nodes), '.k','MarkerSize',30) % plot 3 connected triangle circumcenters
        plot(x(C'),y(C'),'-b','LineWidth',1.5) % plot constriants (i.e. polytopes)
        title('Medial Axis, 3-connected nodes highlighted')
    end
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
    if flag_do_plot
        figure; hold on; box on; title('medial axis graph overlaid on triangulation')
        xlabel('x [km]')
        ylabel('y [km]')
        for j = 2:length(polytopes)
            fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
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
        for j = 2:length(polytopes)
            fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
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
    end % end flag_do_plot
end
