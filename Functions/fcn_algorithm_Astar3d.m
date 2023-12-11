function [cost, route] = fcn_algorithm_Astar3d(vgraph, cgraph, hvec, all_pts, start, finish)
% fcn_algorithm_Astar3d
%
% A minimal version of the A* algorithm for graph searching.  Designed to contain minimal subproceses e.g. visibility graph
% This assumes points are 3D e.g. having an x, y, and z or t dimensions
%
%
%
% FORMAT:
% [cost, route] = fcn_algorithm_Astar3d(vgraph, all_pts, start, finish, rgraph)
%
%
% INPUTS:
%
%   start: the start point vector (x,t,t,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,t,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,t,id)
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%   rgraph: the total reachability graph as an nxn matrix where n is the number of pointes (nodes) in the map.
%         A 1 is in position i,j if j is reachable from point i in a path with n or fewer steps (path segments). 0 otherwise.
%
%
% OUTPUTS:
%
%     cost: the total cost of the selected route
%
%    route: the matrix as produced by fcn_algorithm_Astar3d consisting of waypoints.  Each row is a
%    waypoint, and each column is x, y, t, and point ID
%
%
% DEPENDENCIES:
%
% none but several functions exist to create visibility matrices and fcn_check_reachability can create reachability matrices
%
% EXAMPLES:
%
% See the script: script_test_3d*
% for a full test suite.
%
% This function was written on summer 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, summer by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.

    % vgraph is nxn matrix of 1s and 0s where n is the number of points including the start and goal
    % 1 implies reachability and 0 implies blocked
    % vgrah is indexed from row to col (i.e. vgraph(3,4) is the reachability of 4 from 3)

    % all_pts is (n-2)x3 matrix where each row is a point and the columns are x,y, and point ID

    % start and finish are points of the same format as all points

    % set the diagonal to 0 because while points are technically visible from
    % themselves, A* should not consider them as such else the lowest cost
    % neighbor of any point is itself
    vgraph = vgraph - eye(size(vgraph,1));

    % init the open set, put the start in the open set
    num_nodes = size(vgraph,1); % number of nodes in the cost graph
    open_set = nan(1,num_nodes);
    open_set(start(4)) = start(4); % only store the ID not the whole point

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];

    % make cost matrix, g - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
    possible_gs = cgraph; % the cost graph is all the possible g values that can be added to the open set
    open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
    open_set_gs(start(4)) = possible_gs(start(4),start(4)); % g-value for nodes in open set.  g is the movement cost to

    % make heuristic vector, h - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
    hs = hvec;

   % total cost f, is g for the open set nodes plus the corresponding h
    open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.

    % Initialize the closed list
    closed_set = nan(1,num_nodes);

    % Init. array to track the parent (predecessor) of each node
    % this represents the cheapest way to get to the node and is necessary to
    % reconstruct the cheapest path
    parents = nan(1,num_nodes);

    nodes_expanded = 0;
    nodes_explored = 0;
    % while the open list is not empty
    % the condition implies at least one nan
    while (sum(isnan(open_set)) > 0)
        % find the node with the least f on
        % the open list, call it "q"
        nodes_expanded = nodes_expanded + 1;
        [f_of_q, idx_of_q] = min(open_set_fs);
        q = all_pts_plus_start_and_fin(idx_of_q,:);

        % pop q off the open list
        open_set_fs(idx_of_q) = inf;
        open_set(idx_of_q) = NaN;
        % generate q's successors (points reachable from q)
        qs_row = vgraph(idx_of_q,:);
        successor_idxs = [];
        successor_idxs = find(qs_row);

    % the following block of code sorts the successors so they are explored
    % in ascending cost order.  This is important for the multiple goal
    % case as if multiple successors are goals, the lowest cost
    % option may not be selected without this
        gs_from_q = possible_gs(q(4),:);
        gs_from_q_to_successors = gs_from_q(successor_idxs);
        successors_with_gs = [gs_from_q_to_successors' successor_idxs'];
        successors_with_gs_sorted = sortrows(successors_with_gs);
        successor_idxs = successors_with_gs_sorted(:,2);
        % for each successor...
        for i = 1:length(successor_idxs)
            nodes_explored = nodes_explored + 1;
            successor = all_pts_plus_start_and_fin(successor_idxs(i),:);

            % check if this successor is the goal, if so we're done
            if ismember(successor(4), finish(:,4))
                %% execute code to recover path
                % total path cost is the cost so far to reach q, plus the distance
                % from q to the goal
                cost = open_set_gs(idx_of_q) + hs(idx_of_q);
                % in the multiple finish case, we need to know which finish was selected
                selected_finish_idx = find(successor(4) == finish(:,4));
                % initialize the route consisting of q and the finish
                route = [q; finish(selected_finish_idx,:)];
                cur_pt_idx = q(4);
                % then walk back through the parents array until the start is reached
                % recall the parent array contains the lowest cost predecessor to each node
                % at that nodes ID (i.e. parents(5) = 3 implies the best way to reach 5 is through 3,
                % thus you could then look at parents(3) to find the best way to reach 3 until you have
                % reached the start and therefore recovered the optimal path)
                if cur_pt_idx == start(4)
                    return
                end
                while parents(cur_pt_idx) ~= start(4)
                    % add cur_pt's parent to the path
                    parent = all_pts_plus_start_and_fin(parents(cur_pt_idx),:);
                    route = [parent; route];
                    % set current point to the current point's parent
                    cur_pt_idx = parents(cur_pt_idx);
                end
                % return the route
                route = [start; route];
                sprintf('total nodes expanded: \n %0f',nodes_expanded)
                sprintf('total nodes explored: \n %0f',nodes_explored)
                return
            else
                % if the finish is not a successor of q, find the cost of reaching the successor via q
                % this is the cost to reach q + the cost from q to successor
                tentative_cost = open_set_gs(idx_of_q) + possible_gs(q(4),successor(4));%sqrt((successor(1) - q(1)).^2 + ((successor(2) - q(2)).^2));
                % if this is less than the last recorded cost to reach successor,
                % update the cost to reach successor, add successor to the open set,
                % and set successor's parent to q
                if tentative_cost < open_set_gs(successor(4))
                    parents(successor(4)) = idx_of_q;
                    open_set_gs(successor(4)) = tentative_cost;
                    open_set_fs(successor(4)) = tentative_cost + hs(successor(4));
                    open_set(successor(4)) = successor(4);
                end % end tentative cost comparison check
        end % end looping through successors
        % having checked all of q's successors, push q on the closed list
        closed_set(idx_of_q) = idx_of_q;
    end % end while loop through open set
end % end function
