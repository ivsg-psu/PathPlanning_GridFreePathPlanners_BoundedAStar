function [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeThroughNodes(adjacency_matrix, triangle_chains, nodes, max_branching_factor, varargin);
% fcn_MedialAxis_removeThroughNodes
%
% This trims the medial axis graph, as made by fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains,
% by removing through nodes (nodes that have fewer than 3 departing edges and thus are effectively
% not a decision point for the planner)
% This can be used by itself but is also wrapped by fcn_MedialAxis_pruneGraph.
%
% FORMAT:
%
% [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeThroughNodes(adjacency_matrix, triangle_chains, nodes, max_branching_factor, varargin);
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
%    max_branching_factor: a Nx1 array of integers.
%      branching factor is the number of nodes connected to each node. An asymm-
%      etric graph can have a different number of nodes connected by outbound edges and nodes conne-
%      cted by inbound edges so this is the maximum of these two numbers
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
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning* for examples of the script in use (it's wrapped by fcn_MedialAxis_pruneGraph).
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
    if nargin < 4 || nargin > 5
        error('Incorrect number of arguments');
    end
    % if there is no value in varargin...
    if nargin == 4
        % default is to assume convex obstacles as this is conservative
        flag_do_plot_slow = 0;
    end
    % if there is a value in varargin...
    if nargin == 5
        % check what it is
        if varargin{1} == 1
            % set concave flag if it was passed in
            flag_do_plot_slow = 0;
        elseif varargin{1} == 0
            flag_do_plot_slow = 0;
        else
            % throw error if it was passed in with an incorrect value
            error('optional argument is the plotting flag and can either be 1 or 0')
        end
    end

    idx_2_connected_nodes = find(max_branching_factor == 2); % all two connected nodes are through nodes
    false_through_nodes = [];
    % a false through node has 2 possible destinations but more than 1 possible path to at least one of these destinations
    % it proved difficult to iterate over nodes with 2 connected triangle chains directly
    % so instead we iterate over nodes with 2 possible destinations and
    % allowlist the subset of these nodes with 2 possible destinations but more than 2 triangle chains
    % these are the so-called false through nodes that must be excluded from the through nodes that will be removed
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
            false_through_nodes = [false_through_nodes, t]; % false through nodes get added to a list so we can track them
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
        % connect those two nodes (d and b) in the adjacency matrix
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
end
