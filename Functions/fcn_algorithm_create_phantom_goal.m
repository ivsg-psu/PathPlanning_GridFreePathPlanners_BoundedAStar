function [vgraph, cgraph, hvec, finish, all_pts] = fcn_algorithm_create_phantom_goal(vgraph, cgraph, hvec, finish, all_pts)
% fcn_algorithm_create_phantom_goal
%
% this function creates a fictional "phantom goal" connected to all true goals
% this phantom goal exists in the graph but has no spatial or temporal state
% if A* is instructed to find the phantom goal, it will pass through the optimal
% true goal on the way.  Therefore this method can be used to transform a multi-
% goal problem into a single-goal problem.  See:
% Section 6.5 of Stephen Harnett's PhD dissertation
% Likhachev, M. (2019) 6-350 Planning Techniques for Robotics Search Algorithms:
% Multi-goal A*, IDA*, Tech. rep., Carnegie Mellon University.
% https://www.cs.cmu.edu/~maxim/classes/robotplanning/
% https://www.cs.cmu.edu/~maxim/classes/robotplanning/lectures/informedastar_16350_sp25.pdf
%
%
% FORMAT:
% [vgraph, cgraph, hvec, finish, all_pts] = fcn_algorithm_create_phantom_goal(vgraph, cgraph, hvec, finish, all_pts)
%
% INPUTS:
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
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,id)
%
%
% OUTPUTS:
%
%   Outputs are the same as the inputs except modified to include the phantom goal point
%    which is connected to each input finish point with a zero weight edge
%
% DEPENDENCIES:
%
%   none
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a test that can be run by flagging on "do_phantom".
%
% This function was written on spring 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% Feb 2 2024, spring by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.
    % create a new vgraph column and row of zeros for the imaginary finish
    orig_size = size(vgraph,1);
    new_row = zeros(1,orig_size);
    new_col = new_row';
    vgraph = [[vgraph, new_col]; [new_row, 1]];
    % row can stay zeros (don't need to go from finish to anywhere)
    % column gets a 1 at every finish ID
    % this implies you can go from each finish to the phantom goal
    finish_idx = finish(:,4);
    vgraph(finish_idx,end) = 1;
    % cgraph gets a cost of 0 to go from each finish ID to the phantom
    % and a cost of inf. otherwise
    new_row = inf*ones(1,orig_size);
    new_col = inf*new_row';
    cgraph = [cgraph, new_col; new_row, inf];
    cgraph(finish_idx,end) = 0;
    % HVEC gets a 1 at every finish ID
    hvec(finish_idx) = 1;
    hvec = [hvec 0];
    % append old finish to all_pts since the original finishes are now considered
    % ordinary points in the all_pts table (which conventionally excludes starts
    % and finishes)
    all_pts = [all_pts; finish];
    % make phantom finish: recall it doesn't need x, y, or t coords so these are NaN
    finish = [NaN NaN NaN max(finish_idx)+1];
end
