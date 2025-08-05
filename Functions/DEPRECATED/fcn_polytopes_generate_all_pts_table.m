function [all_pts, start, finish] = fcn_polytopes_generate_all_pts_table(polytopes, start_xy, finish_xy)
% fcn_polytopes_generate_all_pts_table
%
% A short function to turn polytope vertices into an nx5 table of poitns of the form used by:
%  - fcn_algorithm_bound_Astar
%  - fcn_visibility_clear_and_blocked_points_global
%  - fcn_visibility_clear_and_blocked_points
%
%
% FORMAT:
% [all_pts, start, finish] = fcn_polytopes_generate_all_pts_table(polytopes, start_xy, finish_xy)
%
%
% INPUTS:
%
%     start_xy: the start point vector (x,y)
%
%     finish_xy: the finish point vector (x,y)
%
%     polytopes: the polytope struct array
%
%
% OUTPUTS:
%
%     ALL_PTS: p-by-5 matrix of all the possible start points
%       the information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id number
%         obstacle id number
%         beginning/ending indication (1 if the point is a beginning or ending
%         point and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%
%     start: the start point vector as a 1x5 array with the same information as all_pts
%
%     finish: the finish point vector  as a 1x5 array with the same information as all_pts
%
%
% DEPENDENCIES:
%
% none but MapGen library may be useful for creating polytopes
%
% EXAMPLES:
%
% See the script: script_demo_fcn_BoundedAStar_Astar 
% for demonstration of this function in use.
%
% This function was written on 8 May 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024_05_08, by Steve Harnett
% -- first write of function
% 2025_07_07 S. Brennan and K. Hayes
% -- changed demo script 
%    from: script_test_fcn_algorithm_Astar
%    to:   script_demo_fcn_BoundedAStar_Astar

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
