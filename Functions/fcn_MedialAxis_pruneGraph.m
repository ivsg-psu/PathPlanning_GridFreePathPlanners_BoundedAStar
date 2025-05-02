function [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, varargin)
% fcn_MedialAxis_pruneGraph
%
% This trims the medial axis graph, as made by fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains,
% by removing through nodes (nodes that have fewer than 3 departing edges and thus are effectively
% not a decision point) and removing dead ends (edges that branch off from the graph and do not rec-
% onnect anywhere).  It does this in an iterative process until convergence occurs (the graph stops
% changing) because removing a through node can create a dead end and vice versa.  The user may wish
% to use this function to reduce graph size while still having an edge per gap between obstacles
% but the user may wish to skip using this function if having the graph cover a greater spatial ex-
% tent is desirable.
%
% FORMAT:
%
% [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, varargin);
%
%
% INPUTS:
%
%    Useful variables for inputs: N - number of nodes, M - number of edges, P_M - number of triangles
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
%    shrunk_polytopes: the polytope struct array
%
%   (optional arguments)
%   flag_do_plot: a 1 or 0 for flagging plotting on or off.  If ommitted, it is assumed to be 0.
%
%   flag_do_plot_slow: a 1 or 0 for flagging slower plotting on or off.  If ommitted, it is assumed
%      to be 0. This flag is different from flag_do_plot so that slower plotting actions can be con-
%      trolled separately (e.g., a results plot at the end will be controlled by flag_do_plot but
%      a debugging plot that prints at every iteration of a loop will be controlled by flag_do_plot_slow)
%
% OUTPUTS:
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    adjacency_matrix: same size as the input adjacency matrix but nodes that are no longer adjacent
%      due to pruning have their 1's set to 0's
%
%    triangle_chains: same size as input triangle_chains data structure but edges that have been
%      pruned have an empty third column (i.e., there is no chain of triangles between the nodes)
%
%    nodes: same size as input nodes array but nodes that have been removed due to pruning are set
%      to NaN
%
% DEPENDENCIES:
%
%  -  fcn_MedialAxis_removeDeadEnds
%  -  fcn_MedialAxis_removeThroughNodes
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
    if nargin < 6 || nargin > 7
        error('Incorrect number of arguments');
    end
    % if there is no value in varargin...
    if nargin == 6
        % default is to assume convex obstacles as this is conservative
        flag_do_plot_slow = 0;
        flag_do_plot = 0;
    end
    % if there is a value in varargin...
    if nargin == 7
        % check what it is
        if varargin{1} == 1
            % set concave flag if it was passed in
            flag_do_plot_slow = 0;
            flag_do_plot = 1;
        elseif varargin{1} == 0
            flag_do_plot_slow = 0;
            flag_do_plot = 0;
        else
            % throw error if it was passed in with an incorrect value
            error('optional argument is the plotting flag and can either be 1 or 0')
        end
    end
    % need to store previous state of triangle_chains struct to check for convergence
    prev_triangle_chains = nan; % initialize the previous triangle_chains structure to nothing until we iterate the pruning once
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
        prev_triangle_chains = triangle_chains; % store the current triangle chain structure before we modify it
        [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeThroughNodes(adjacency_matrix, triangle_chains, nodes, max_branching_factor, flag_do_plot);
        if flag_do_plot
            % plot the graph without through nodes
            figure; hold on; box on; title('medial axis graph with through nodes removed')
            colors = {"#A2142F","#7E2F8E","#EDB120","#0072BD"}; % some different colors
            color_idx = 1;
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
        % TODO @sjharnett do we want to set these to zero/empty or actually remove them? Removing would require re-indexing
        %% remove dead ends
        [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeDeadEnds(adjacency_matrix, triangle_chains, nodes, max_branching_factor);
        if flag_do_plot
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
end
