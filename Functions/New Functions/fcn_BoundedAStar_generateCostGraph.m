function [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode)
% fcn_BoundedAStar_generateCostGraph
%
% A function for generating a cost matrix and heuristic cost vector.  The cost matrix describes the
% actual cost to go from one point to another in a map.  The heuristic vector describes the estimated
% cost of going from each point to the goal.  If there are multiple goals, this is the minimum of
% going to any goal because the heuristic should be an underestimate of the actual cost (see:
% http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html).  Note using this function
% is completely optional.  It is commonly used prior to calling a planner such as Astar
% to conveniently generate a cost map but it may be circumvented if the user wishes to generate
% their own cost function and cost function matrix
%
%
% FORMAT:
% [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode)
%
%
% INPUTS:
%
%   start: the start point vector as (x,y,id) or (x,t,t,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point
%     vector as (x,y,id) or (x,y,t,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,t,id)
%
%   mode: a string for how the cost should be expressed. The mode argument must be a string with
%     one of the following values:
%       - "xyz or xyt" - full 3D cost of the Euclidean distance between two points in x,y,z or x,y,t
%       - "time or z only" - cost between two points is only based on their t or z distance
%       - "xy spatial only" - cost between two points is based on their x-y Euclidean distance
%
%
% OUTPUTS:
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
%
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_test_3d* and script_test_fcn_algorithm_astar
% for a full test suite.
%
% This function was written on December 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu
%
% REVISION HISTORY:
%
% December 2023 by Steve Harnett
% -- first write of function
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% -- copied function to new file from fcn_algorithm_generate_cost_graph to
%    follow library conventions
%
% TO DO:
%
% -- fill in to-do items here.

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];
    xs = all_pts_plus_start_and_fin(:,1); % vector of all x coords
    ys = all_pts_plus_start_and_fin(:,2); % vector of all y coords

    if mode == "xyz or xyt"
        zs = all_pts_plus_start_and_fin(:,3); % vector of all z coords
        % make cost matrix, called g in A* parlance
        cgraph = sqrt((xs - xs').^2 + (ys - ys').^2 + (zs - zs').^2)'; % distance of every pt from all other pts
        % make heuristic cost vector, called h in A* parlance
        % WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
        hvec = min(sqrt((xs - finish(:,1)').^2 + (ys - finish(:,2)').^2 + (zs - finish(:,3)').^2),[],2)';
        % xs - finish(:,1)' gives a matrix where each row is a point and each
        % column is a finish point so the element in 3,4 is the difference of
        % point 3 and finish 4
        % then performing min(M,[],2) on this matrix gives a vector with the
        % minimum of each row, i.e. for each point the lowest heuristic cost to
        % a goal.  This is important for the multiple goal case as A* must have
        % a heuristic that underestimtes actual cost
    elseif mode == "time or z only" % same code as above but only accounting for z (or t) distance
        zs = all_pts_plus_start_and_fin(:,3); % vector of all y coords
        cgraph = sqrt((zs - zs').^2)';
        hvec = min(sqrt((zs - finish(:,3)').^2),[],2)';
    elseif mode == "xy spatial only" % same code as above but only accounting for x-y distance
        cgraph = sqrt((xs - xs').^2 + (ys - ys').^2)';
        hvec = min(sqrt((xs - finish(:,1)').^2 + (ys - finish(:,2)').^2),[],2)';
    else
        % if the mode string is not a valid value, throw an error with explanation and suggestion
        error('The mode argument must be a string with the value "xyz or xyt", or "time or z only", "xy spatial only". You may make a cost matrix manually and circumvent using this function by creating as well by making square N-dimensional matrix where N is the numebr of nodes including starts and goals.  Element i-j has a value corresponding to the cost of going form node i to node j.')
    end
end % end function
