function [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(polytopes,A,B,planner_mode,varargin)
% FCN_DASTAR_FOR_VORONOI sets up the information needed to for the Dijkstra
% Astar hybrid function and calls the function when the input polytopes are
% generated from the voronoi diagram
%
% [PATH,COST,ERR]=FCN_DASTAR_FOR_VORONOI(POLYTOPES,A,B,BOUNDS)
% returns:
% PATH: a series of points along the near optimal path
% COST: the distance along that path
% ERR: error flag for reporting errors without stopping the code
%
% with intputs:
% POLTYOPES: a 1-by-n seven field structure of voronoi polytopes, where
%   n <= number of polytopes
%   with fields:
%   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%       the number of the individual polytope vertices
%   xv: a 1-by-m vector of vertice x-coordinates
%   yv: a 1-by-m vector of vertice y-coordinates
%   distances: a 1-by-m vector of perimeter distances from one point to the
%       next point, distances(i) = distance from vertices(i) to vertices(i+1)
%   mean: average xy coordinate of the polytope
%   area: area of the polytope
%   max_radius: distance from the mean to the farthest vertex
% A: starting point structure with fields for the x and y coordinates
% B: finishing point structure with fields for the x and y coordinates
% BOUNDS: b-by-2 matrix of xy coordinates of the boundaries the path
% planner must stay within, where b is the number of boundary points and
% b>=3. If this argument not specified, there are no bounds.
% PLANNER_MODE: string containing option for planner behavior
% indicates the planner mode
% "legacy" only goes around obstacles
% "through at vertices" allows the planner to go through or around each obstacle
% but only entering or exiting at vertices
% "through or around" allows the planner to go through all obstacles or around all
% "straight through" the planner only goes straight from the start to the goal, calculating the cost
%
% Examples:
%
%      Example 1:
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_generation'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_editing'])
%      addpath([parent_dir '\' main_folder '\Path_Planning\algorithm_setup'])
%      polytopes=fcn_polytope_generation_halton_voronoi_tiling(1,100,[100,100]);
%      trim_polytopes=fcn_polytope_editing_remove_edge_polytopes(polytopes,0,100,0,100);
%      shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);
%      A.x = 0; A.y = 50; B.x = 100; B.y = 50;
%      [path,cost,err]=fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B);
%      disp(['Path Cost: ' num2str(cost)])
%      fcn_plot_polytopes(shrunk_polytopes,100,'b-',2,[0 100 0 100],'square')
%      plot(path(:,1),path(:,2),'k-','linewidth',2)
%      plot([A.x B.x],[A.y B.y],'kx','linewidth',2)
%
% This function was written on 2019_06_13 by Seth Tau
% Questions or comments? sat5340@psu.edu
%

%% check if the start or end are now within combined polytopes
throw_error = 0; % only gives soft errors errors that don't stop the code
check_edge = 1; % checks for A or B on polytope edges
[err,Apoly,Bpoly] = fcn_polytope_calculation_points_in_polytopes(A,B,polytopes,throw_error,check_edge); err = 0;% check that start and end are outside of obstacles
if err == 0 % A and B outside the polytopes
    %% add points for start and finish if on an edge
    if Apoly ~= -1 % on obstacle edge
        vertices = polytopes(Apoly).vertices;
        A_gap = fcn_polytope_calculation_point_gap_location([A.x A.y],vertices(:,1:2));
        new_verts = [vertices(1:A_gap,:); A.x A.y; vertices(A_gap+1:end,:)];
        polytopes(Apoly).vertices = new_verts;
        polytopes(Apoly).xv = new_verts(1:end-1,1)';
        polytopes(Apoly).yv = new_verts(1:end-1,2)';
        polytopes(Apoly).distances = fcn_general_calculation_euclidean_point_to_point_distance(new_verts(1:end-1,:),new_verts(2:end,:));
    end
    if Bpoly ~= 0 % on obstacle edge
        vertices = polytopes(Bpoly).vertices;
        B_gap = fcn_polytope_calculation_point_gap_location([B.x B.y],vertices(:,1:2));
        new_verts = [vertices(1:B_gap,:); B.x B.y; vertices(B_gap+1:end,:)];
        polytopes(Bpoly).vertices = new_verts;
        polytopes(Bpoly).xv = new_verts(1:end-1,1)';
        polytopes(Bpoly).yv = new_verts(1:end-1,2)';
        polytopes(Bpoly).distances = fcn_general_calculation_euclidean_point_to_point_distance(new_verts(1:end-1,:),new_verts(2:end,:));
    end

    point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes

    %% information about each point
    beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    for poly = 1:size(polytopes,2) % check each polytope
        verts = unique(polytopes(poly).vertices,'stable','rows');
        num_verts = size(verts,1);
        polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
        polytopes(poly).xv = verts(:,1)';
        polytopes(poly).yv = verts(:,2)';
        polytopes(poly).vertices = [verts; verts(1,:)];
        polytopes(poly).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly).vertices(1:end-1,:),polytopes(poly).vertices(2:end,:));
        beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
        curpt = curpt+num_verts;
        polytopes(poly).perimeter = sum(polytopes(poly).distances);
    end
    obs_id = [polytopes.obs_id];
    point_tot = length([polytopes.xv]); % need to recheck total points
    beg_end = beg_end(1:point_tot); % remove any extra points

    all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]

    % give the same information to the starting and ending points
    if Apoly ~= -1 % on obstacle edge
        % find adjacent points
        adj_pts = all_pts(all_pts(:,4)==Apoly,:);

        A_id = adj_pts(A_gap+1,3);
        A_beg_end = all_pts(A_id,5);
%         if A_gap == size(adj_pts,1)
%             A_beg_end = 1;
%         else
%             A_beg_end = 0;
%         end
    else
        A_id = point_tot+1;
        A_beg_end = 0;
    end
    if Bpoly ~= 0 % on obstacle edge
        % find adjacent points
        adj_pts = all_pts(all_pts(:,4)==Bpoly);

        B_id = adj_pts(B_gap+1,3);
        B_beg_end = all_pts(B_id,5);
%         if B_gap == size(vertices,1)
%             B_beg_end = 1;
%         else
%             B_beg_end = 0;
%         end
    else
        B_id = point_tot+2;
        B_beg_end = 0;
    end
    start = [A.x A.y A_id Apoly A_beg_end];
    finish = [B.x B.y B_id Bpoly B_beg_end];

    if nargin > 4
        bounds = varargin{1};
        bound_pts = all_pts(inpolygon(all_pts(:,1),all_pts(:,2),bounds(:,1),bounds(:,2)),:); % bound points at the start
    else
        bound_pts = all_pts;
    end

    %% find valid points
    [~,ia,ic] = unique(bound_pts(:,1:2),'rows','stable');
    h = accumarray(ic, 1);
    valid_pts = bound_pts(ia(h==1),:);
%     plot(valid_pts(:,1),valid_pts(:,2),'mx','linewidth',2)




    %% create polytopes for finding ellipses %%%%%%%%%%%%%%%%%%%%%%% needs debugging for 3+ verts in a straight line
%     ellipse_polytopes =
%     fcn_polytope_editing_tiling_loop_polytopes(polytopes);
    ellipse_polytopes = polytopes;

    %% calculate path
    [cost,path] = fcn_algorithm_bound_Astar(start,finish,polytopes,all_pts,valid_pts,planner_mode,ellipse_polytopes);

else % A or B are in the polytopes
   path = [];
   cost = [];
end
