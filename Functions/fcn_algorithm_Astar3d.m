
function [cost, route] = fcn_algorithm_Astar(vgraph, all_pts, start, finish)
    % set the diagonal to 0 because while points are technically visible from
    % themselves, A* should not consider them as such else the lowest cost
    % neighbor of any point is itself
    vgraph = vgraph - eye(size(vgraph,1));

    % init the open set, put the start in the open set
    num_nodes = size(vgraph,1); % number of nodes in the cost graph
    open_set = nan(1,num_nodes);
    open_set(start(3)) = start(3); % only store the ID not the whole point

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];
    xs = all_pts_plus_start_and_fin(:,1); % vector of all x coords
    ys = all_pts_plus_start_and_fin(:,2); % vector of all y coords

    % make cost matrix, g
    possible_gs = sqrt((xs - xs').^2 + (ys - ys').^2)'; % distance of every pt from all other pts
    open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
    open_set_gs(start(3)) = possible_gs(start(3),start(3)); % g-value for nodes in open set.  g is the movement cost to

    % make heuristic matrix, h
    hs = sqrt((xs - finish(1)).^2 + (ys - finish(2)).^2)';

    % total cost f, is g for the open set nodes plus the corresponding h
    open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.

    % Initialize the closed list
    closed_set = nan(1,num_nodes);

    % Init. array to track the parent (predecessor) of each node
    % this represents the cheapest way to get to the node and is necessary to
    % reconstruct the cheapest path
    parents = nan(1,num_nodes);

    % while the open list is not empty
    % the condition implies at least one nan
        while (sum(isnan(open_set)) > 0)
        %     a) find the node with the least f on
        %        the open list, call it "q"
            [f_of_q, idx_of_q] = min(open_set_fs);
            q = all_pts_plus_start_and_fin(idx_of_q,:);

%                 b) pop q off the open list
            open_set_fs(idx_of_q) = inf;
            open_set(idx_of_q) = NaN;
%                 c) generate q's 8 successors and set their
%        parents to q
            qs_row = vgraph(idx_of_q,:);
            successor_idxs = [];
            successor_idxs = find(qs_row);
            % TODO @sjharnett set parents to q
%             parents(successor_idxs) = idx_of_q;

    %     d) for each successor
        for i = 1:length(successor_idxs)
            successor = all_pts_plus_start_and_fin(successor_idxs(i),:);
    %         i) if successor is the goal, stop search

            % TODO fix parentage by setting parent once for each point
            if successor(3) == finish(3)
                % code to recover path
                cost = open_set_gs(idx_of_q) + hs(idx_of_q);

                route = [q; finish];
                cur_pt_idx = q(3);
                while parents(cur_pt_idx) ~= start(3)
                    % add cur_pt's parent to the path
                    parent = all_pts_plus_start_and_fin(parents(cur_pt_idx),:);
                    route = [parent; route];
                    % set q to its parent
                    cur_pt_idx = parents(cur_pt_idx);
                end
                route = [start; route];

                % % code to recover path
                % cost = open_set_gs(idx_of_q) + hs(idx_of_q);
                % route = [finish];
                % parent = idx_of_q;
                % route = [q; finish];
                % q_history(end,:) = [];
                % possible_parents = intersect(q_parents{end}, q_history(:,3));
                % while parent ~= start(3)
                    % parent_gs = open_set_gs(possible_parents);
                    % [parent_g, idx_of_parent] = min(parent_gs);
                    % parent = possible_parents(idx_of_parent);
                    % parent_position_in_history = find(parent == q_history(:,3));
                    % parent_point = q_history(parent_position_in_history,:);
                    % route = [parent_point;route];
                    % q_history(parent_position_in_history:end,:) = [];
% %                     parent = possible_parents(idx_of_parent);
% %                     parent_position_in_history = find(parent == q_history(:,3));
                    % possible_parents = intersect(q_parents{parent_position_in_history}, q_history(:,3));
                % end
                return
            else
            tentative_cost = open_set_gs(idx_of_q) + possible_gs(successor(3),q(3));%sqrt((successor(1) - q(1)).^2 + ((successor(2) - q(2)).^2));
            if tentative_cost < open_set_gs(successor(3))
                parents(successor(3)) = idx_of_q;
                open_set_gs(successor(3)) = tentative_cost;
                open_set_fs(successor(3)) = tentative_cost + hs(successor(3));
                open_set(successor(3)) = successor(3);
            end

%             if tentative_gScore < gScore[neighbor]
%                 // This path to neighbor is better than any previous one. Record it!
%                 cameFrom[neighbor] := current
%                 gScore[neighbor] := tentative_gScore
%                 fScore[neighbor] := tentative_gScore + h(neighbor)
%                 if neighbor not in openSet
%                     openSet.add(neighbor)
%
%                 successor_h = hs(successor(3));
%                 successor_f = successor_g + successor_h;
%                 open_set(successor(3)) = successor(3);
%                 open_set_gs(successor(3)) = successor_g;
%                 open_set_fs(successor(3)) = successor_f;
%             end    %         ii) else, compute both g and h for successor




    %      end (for loop)
        end
    %
    %     e) push q on the closed list
        closed_set(idx_of_q) = idx_of_q;
    %     end (while loop)
    end
end
