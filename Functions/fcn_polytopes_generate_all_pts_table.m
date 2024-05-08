function [all_pts, start, finish] = fcn_generate_all_pts_table(polytopes, start_xy, finish_xy)
% fcn_generate_all_pts_table
%
% A short function to turn polytope vertices into a points array of the form used by bounded Astar
%
%
%
% FORMAT:
% [cost, route] = fcn_algorithm_Astar(vgraph, all_pts, start, finish, rgraph)
%
%
% INPUTS:
%
%   start: the start point vector (x,t,id)
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
%    route: the matrix as produced by fcn_algorithm_Astar3d consisting of waypoints.  Each row is a
%    waypoint, and each column is x, y, and point ID
%
%
% DEPENDENCIES:
%
% none but MapGen library may be useful for creating polytopes
%
% EXAMPLES:
%
% See the script: script_test_fcn_algorithm_Astar
% for demonstration of this function in use.
%
% This function was written on 8 May 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024_May_08, by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.
    point_tot = length([polytopes.xv]); % total number of vertices in the polytopes
    beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    for poly = 1:size(polytopes,2) % check each polytope
        verts = length(polytopes(poly).xv);
        polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
        beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
        curpt = curpt+verts;
    end
    obs_id = [polytopes.obs_id];
    all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
    start = [start_xy size(all_pts,1)+1 -1 1];
    finish = [finish_xy size(all_pts,1)+2 -1 1];
end
