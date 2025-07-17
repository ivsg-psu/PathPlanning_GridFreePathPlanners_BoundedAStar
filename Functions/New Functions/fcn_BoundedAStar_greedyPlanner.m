function [cost, route] = fcn_BoundedAStar_greedyPlanner(vgraph, all_pts, start, finish)
% fcn_BoundedAStar_greedyPlanner
%
% [INSERT FUNCTION DESCRIPTION HERE]
%
% FORMAT:
% [cost, route] = fcn_BoundedAStar_greedyPlanner(vgraph, all_pts, start, finish)
%
%
% INPUTS:
%
%   [FILL IN INPUTS HERE]
%
%
% OUTPUTS:
%
%  [FILL IN OUTPUTS HERE]
%  
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_fcn_BoundedAStar_greedyPlanner
% for a full test suite.
%
% This function was written in January 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu
%
% REVISION HISTORY:
%
% January 2024 by Steve Harnett
% -- first write of function
% February 2024 by Steve Harnett
% -- function updated to make a right left distinction using cross products
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% -- function copied to new script from
%    fcn_algorithm_greedy_planner.m to follow library
%    conventions
%
% TO DO:
%
% -- fill in to-do items here.

    vgraph = vgraph - eye(size(vgraph,1));
    % 1.  Initialize the open list
    %     put the starting node on the open
    %     list (you can leave its f at zero)
    num_nodes = size(vgraph,1); % number of nodes in the cost graph
    open_set = nan(1,num_nodes);
    open_set(start(3)) = start(3); % only store the ID not the whole point
    open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
    open_set_gs(start(3)) = 0; % g-value for nodes in open set.  g is the movement cost to
    % move from the starting point to a given square on the grid, following the
    % path generated to get there.
    all_pts_plus_start_and_fin = [all_pts; start; finish];
    % find all heuristic costs as distances from point to finish
    hs = sqrt((all_pts_plus_start_and_fin(:,1) - finish(1)).^2 + (all_pts_plus_start_and_fin(:,2) - finish(2)).^2)';

    %  the estimated movement cost to move from that given square on the grid to
    % the final destination. This is often referred to as the heuristic
    open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.
    % f is the sum of the g-value and h-value
    % TODO @sjharnett f, g, and h should only depend on cost graph not including calculations
    % 2.  Initialize the closed list
    closed_set = nan(1,num_nodes);
%     closed_set_fs = inf*ones(1,num_nodes);
    q_history = [];
    q_parents = {};
    % 3.  while the open list is not empty
    % implies at least one nan
        while (sum(isnan(open_set)) > 0)
        %     a) find the node with the least f on
        %        the open list, call it "q"
            [f_of_q, idx_of_q] = min(open_set_fs);
            q = all_pts_plus_start_and_fin(idx_of_q,:);
            q_history = [q_history; q];

        %                 b) pop q off the open list
            open_set_fs(idx_of_q) = inf;
%             open_set_gs(idx_of_q) = inf;
%             open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set
            open_set(idx_of_q) = NaN;
%                 c) generate q's 8 successors and set their
%        parents to q
            qs_row = vgraph(idx_of_q,:);
            successor_idxs = [];
            successor_idxs = find(qs_row);
            % TODO @sjharnett set parents to q






          q_parents{end+1} = successor_idxs;
    %     d) for each successor
        for i = 1:length(successor_idxs)
            successor = all_pts_plus_start_and_fin(successor_idxs(i),:);
    %         i) if successor is the goal, stop search


            if successor(3) == finish(3)
                cost = open_set_gs(idx_of_q) + hs(idx_of_q);
                route = [finish];
                parent = idx_of_q;
                route = [q; finish];
                q_history(end,:) = [];
                possible_parents = intersect(q_parents{end}, q_history(:,3));
                while parent ~= start(3)
                    parent_gs = open_set_gs(possible_parents);
                    [parent_g, idx_of_parent] = min(parent_gs);
                    parent = possible_parents(idx_of_parent);
                    parent_position_in_history = find(parent == q_history(:,3));
                    parent_point = q_history(parent_position_in_history,:);
                    route = [parent_point;route];
                    q_history(parent_position_in_history:end,:) = [];
%                     parent = possible_parents(idx_of_parent);
%                     parent_position_in_history = find(parent == q_history(:,3));
                    possible_parents = intersect(q_parents{parent_position_in_history}, q_history(:,3));
                end
                return
            else
                 %         ii) else, compute both g and h for successor
                successor_g = open_set_gs(idx_of_q) + sqrt((successor(1) - q(1)).^2 + ((successor(2) - q(2)).^2));
                successor_h = sqrt((successor(1) - finish(1)).^2 + ((successor(2) - finish(2)).^2));
                successor_f = successor_g + successor_h;
                open_set(successor(3)) = successor(3);
                open_set_gs(successor(3)) = successor_g;
                open_set_fs(successor(3)) = successor_f;

            end



%         iii) if a node with the same position as
%             successor is in the OPEN list which has a
%            lower f than successor, skip this successor
%         if sum(open_set_fs < successor_f) > 0
%             continue
%         end
%         if sum(closed_set_fs < successor_f)
%             continue
%         else
%         iV) if a node with the same position as
%             successor  is in the CLOSED list which has
%             a lower f than successor, skip this successor
%             otherwise, add  the node to the open list


    %      end (for loop)
%         end

    %
    %     e) push q on the closed list
        closed_set(idx_of_q) = idx_of_q;
%         closed_set_fs(idx_of_q) = open_set_fs(idx_of_q);
    %     end (while loop)
    end
end
