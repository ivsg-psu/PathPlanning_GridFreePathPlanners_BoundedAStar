function [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_pruneGraph(adjacency_matrix, triangle_chains, nodes, xcc, ycc, shrunk_polytopes, varargin);
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
        prev_triangle_chains = triangle_chains; % store the current triangle chain structure before we modify it
        [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeThroughNodes(adjacency_matrix, triangle_chains, nodes, max_branching_factor, flag_do_plot);
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
            % pot big markers for the start and end node
            beg_end = [chain_of_note(1) chain_of_note(end)];
            % plot a straight line between them (this is the adjacency graph connection)
            plot(xcc(beg_end), ycc(beg_end), '--.','MarkerSize',20,'Color',colors{mod(color_idx,4)+1})
            % plot the medial axis path between them (this is the curved path from the triangle chain)
            plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',colors{mod(color_idx,4)+1})
            color_idx = color_idx + 1;
        end
        % TODO @sjharnett do we want to set these to zero/empty or actually remove them? Removing would require re-indexing
        %% remove dead ends
        [adjacency_matrix, triangle_chains, nodes] = fcn_MedialAxis_removeDeadEnds(adjacency_matrix, triangle_chains, nodes, max_branching_factor);
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
            % pot big markers for the start and end node
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
end
