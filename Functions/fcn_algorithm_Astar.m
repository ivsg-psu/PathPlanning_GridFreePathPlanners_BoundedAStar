function [cost, route] = fcn_algorithm_Astar(cgraph, all_pts, start, finish)

% 1.  Initialize the open list
%     put the starting node on the open
%     list (you can leave its f at zero)
num_nodes = size(cgraph,1); % number of nodes in the cost graph
open_set = nan(1,num_nodes);
open_set(start(3)) = [start(3)]; % only store the ID not the whole point
open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
open_set_gs(start(3)) = 0; % g-value for nodes in open set.  g is the movement cost to
% move from the starting point to a given square on the grid, following the
% path generated to get there.
open_set_hs(start(3)) = norm(finish(1:2)-start(1:2));  % h-value for nodes in open set.
%  the estimated movement cost to move from that given square on the grid to
% the final destination. This is often referred to as the heuristic
open_ste_fs = open_set_gs + open_set_hs; % f-vlaue for nodes in the open set.
% f is the sum of the g-value and h-value
% TODO @sjharnett f, g, and h should only depend on cost graph not including calculations
% 2.  Initialize the closed list
closed_set = nan(1,num_nodes);

% 3.  while the open list is not empty
% implies at least one nan
while (sum(isnan(open_set)) > 0)
%     a) find the node with the least f on
%        the open list, call it "q"
    [f_of_q, idx_of_q] = min(open_set_fs);

%     b) pop q off the open list
    open_set(idx_of_q) = NaN;

%
%     c) generate q's 8 successors and set their
%        parents to q
    qs_row = cgraph(idx_of_q,:);
    % TODO @sjharnett find the columns that aren't 0 or aren't infinite
%     d) for each successor
    for i = 1:1:length(successors)
        successor = successors(i);
%         i) if successor is the goal, stop search
        if all_pts(successor,3) == finish(3)
            break
        else
%         ii) else, compute both g and h for successor
%           successor.g = q.g + distance between
%                               successor and q
%           successor.h = distance from goal to
%           successor (This can be done using many
%           ways, we will discuss three heuristics-
%           Manhattan, Diagonal and Euclidean
%           Heuristics)
%
%           successor.f = successor.g + successor.h
%
%         iii) if a node with the same position as
%             successor is in the OPEN list which has a
%            lower f than successor, skip this successor
%
%         iV) if a node with the same position as
%             successor  is in the CLOSED list which has
%             a lower f than successor, skip this successor
%             otherwise, add  the node to the open list
%      end (for loop)
%
%     e) push q on the closed list
%     end (while loop)
end
