function [cost, route] = fcn_algorithm_Astar(cgraph, all_pts, start, finish)

% 1.  Initialize the open list
%     put the starting node on the open
%     list (you can leave its f at zero)
open_set = [start(3)]; % only store the ID not the whole point

open_set_gs = [0]; % g-value for nodes in open set.  g is the movement cost to
% move from the starting point to a given square on the grid, following the
% path generated to get there.
open_set_hs = norm(finish(1:2)-start(1:2));  % h-value for nodes in open set.
%  the estimated movement cost to move from that given square on the grid to
% the final destination. This is often referred to as the heuristic
open_ste_fs = open_set_gs + open_set_hs; % f-vlaue for nodes in the open set.
% f is the sum of the g-value and h-value

% 2.  Initialize the closed list
closed_set = [];

% 3.  while the open list is not empty
while ~isempty(open_set)
%     a) find the node with the least f on
%        the open list, call it "q"
    [f_of_q, idx_of_q] = min(open_set_fs);

%     b) pop q off the open list
%
%     c) generate q's 8 successors and set their
%        parents to q
%
%     d) for each successor
%         i) if successor is the goal, stop search
%
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
