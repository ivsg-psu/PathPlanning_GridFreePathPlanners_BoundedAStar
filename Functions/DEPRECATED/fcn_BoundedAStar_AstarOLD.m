function [cost, route] = fcn_BoundedAStar_Astar(vgraph, cgraph, hvec, all_pts, start, finish)
% fcn_BoundedAstar_Astar
%
% A minimal version of the A* algorithm for graph searching.  Designed to contain minimal subproceses e.g. visibility graph
%
%
%
% FORMAT:
% [cost, route] = fcn_BoundedAstar_Astar(vgraph, all_pts, start, finish, rgraph)
%
%
% INPUTS:
%
%   start: the start point vector (x,y,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,id)
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%    cgraph: the cost graph matrix. A cost matrix is an nxn matrix where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element i-j is the cost of routing from i to j.
%
%    hvec: the heuristic cost vector. A 1xn vector where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element k is the estimated cost of routing from point k to
%      the finish based on a heuristic cost estimation method.
%
% OUTPUTS:
%
%     cost: the total cost of the selected route
%
%    route: the matrix as produced by fcn_BoundedAstar_Astar consisting of waypoints.  Each row is a
%    waypoint, and each column is x, y, and point ID
%
%
% DEPENDENCIES:
%
% none but several functions exist to create visibility matrices and fcn_algorithm_generate_cost_graph can create cost matrices (cgraph) and heuristic cost vectors (hvec)
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAstar_Astar
% for a full test suite.
%
% This function was written on spring 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, spring by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.

    % vgraph is nxn matrix of 1s and 0s where n is the number of points including the start and goal
    % 1 implies reachability and 0 implies blocked
    % vgrah is indexed from row to col (i.e. vgrpah(3,4) is the reachability of 4 from 3)

    % all_pts is (n-2)x3 matrix where each row is a point and the columns are x,y, and point ID

    % start and finish are points of the same format as all points

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

    % make cost matrix, g - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
    possible_gs = cgraph;
    open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
    open_set_gs(start(3)) = possible_gs(start(3),start(3)); % assign g value from possible_gs to open_set_gs for the start

    hs = hvec;

    % total cost f, is g for the open set nodes plus the corresponding h
    open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.

    % Initialize the closed list
    closed_set = nan(1,num_nodes);

    % Init. array to track the parent (predecessor) of each node
    % this represents the cheapest way to get to the node and is necessary to
    % reconstruct the cheapest path
    parents = nan(1,num_nodes);

    % while the open list is not empty...
    % the condition implies at least one nan
        while (sum(isnan(open_set)) > 0)
            % find the node with the least f on
            % the open list, call it "q"
            [f_of_q, idx_of_q] = min(open_set_fs);
            q = all_pts_plus_start_and_fin(idx_of_q,:);

            % pop q off the open list
            open_set_fs(idx_of_q) = inf;
            open_set(idx_of_q) = NaN;
            % generate q's successors (points reachable from q)
            qs_row = vgraph(idx_of_q,:);
            successor_idxs = [];
            successor_idxs = find(qs_row);

            % for each successor...
            for i = 1:length(successor_idxs)
                successor = all_pts_plus_start_and_fin(successor_idxs(i),:);
               % check if this successor is the goal, if so we're done
            if successor(3) == finish(3)
                %% execute code to recover path
                % total path cost is the cost so far to reach q, plus the distance
                % from q to the goal
                cost = open_set_gs(idx_of_q) + hs(idx_of_q);
                % initialize the route consisting of q and the finish
                route = [q; finish];
                % set the current point back one from the finish, to q
                cur_pt_idx = q(3);
                % then walk back through the parents array until the start is reached
                % recall the parent array contains the lowest cost predecessor to each node
                % at that nodes ID (i.e. parents(5) = 3 implies the best way to reach 5 is through 3,
                % thus you could then look at parents(3) to find the best way to reach 3 until you have
                % reached the start and therefore recovered the optimal path)
                if cur_pt_idx == start(3)
                    return
                end
                while parents(cur_pt_idx) ~= start(3)
                    % add cur_pt's parent to the route
                    parent = all_pts_plus_start_and_fin(parents(cur_pt_idx),:);
                    route = [parent; route];
                    % set current point to the current point's parent
                    cur_pt_idx = parents(cur_pt_idx);
                end
                % return the route
                route = [start; route];
                return
            else
            % if the finish is not a successor of q, find the cost of reaching the successor via q
            % this is the cost to reach q + the cost from q to successor
            tentative_cost = open_set_gs(idx_of_q) + possible_gs(q(3),successor(3));
            if isnan(possible_gs(q(3),successor(3)))
                my_err = sprintf('cost to go from node %i to node %i is undefined or nan',q(3),successor(3));
                error(my_err)
            end
            % if this is less than the last recorded cost to reach successor,
            % update the cost to reach successor, add successor to the open set,
            % and set successor's parent to q
            if tentative_cost < open_set_gs(successor(3))
                parents(successor(3)) = idx_of_q;
                open_set_gs(successor(3)) = tentative_cost;
                open_set_fs(successor(3)) = tentative_cost + hs(successor(3));
                open_set(successor(3)) = successor(3);
            end % end tentative cost comparison check
        end % end looping through successors
        % having checked all of q's successors, push q on the closed list
        closed_set(idx_of_q) = idx_of_q;
    end % end while loop through open set
end % end function
