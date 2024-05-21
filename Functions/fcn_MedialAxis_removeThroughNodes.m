function [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeThroughNodes(adjacency_matrix, triangle_chains, nodes, max_branching_factor, varargin);

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
                % pot big markers for the start and end node
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
