function [xings] = fcn_visibility_line_polytope_intersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes)
% FCN_VISIBILITY_LINE_POLTYOPE_INTERSECTIONS finds polytope-path intersection 
% information
%
% [XINGS]=FCN_VISIBILITY_LINE_POLYTOPE_INTERSECTIONS(XIP,YIP,XIQ,YIQ,XJP,YJP,D,DI,NUM_INT,POLYTOPES)
% returns:
% XINGS: a 1-by-f three field structure of intersection information, where
%   f = number of finish points, with fields:
% points: i-by-2 matrix of xy coordinates, where i=number of intersections  
% index: 1-by-i vector of intersecting line indices
% obstacles: 1-by-i vector of obstacles intersecting the line
%
% with inputs:
% XIP: m-by-1 vector of the x-coordinates of the starting point of 
%   potential paths
% YIP: m-by-1 vector of the y-coordinates of the starting point of 
%   potential paths
% XIQ: m-by-1 vector of the x-coordinates of the finishing points of 
%   potential paths
% YIQ: m-by-1 vector of the y-coordinates of the finishing points of 
%   potential paths
% XJP: 1-by-n vector of the x-coordinates of the starting point of the
%   polytope edge
% YJP: 1-by-n vector of the y-coordinates of the starting point of the
%   polytope edge
% NUM_INT: m-by-1 vector of the number of intersections between the START
%   and each point in FINISH 
% D: m-by-n intersection array, where m = number of finish points 
%   and n = number of polytope edges, where 1 indicates that an
%   intersection occures between the START and FINISH point on the
%   corresponding polytope edge and 0 indicates no intersection
% DI: m-by-n intersection array, where each value gives the percentage of
%   how far along the path from START to FINISH the intersection occurs
% POLYTOPES: a 1-by-p six field structure of combined polytopes, where 
%   p = number of polytopes, with fields:
% vertices: a v+1-by-2 matrix of xy points with row1 = rowv+1
% xv: a 1-by-v vector of vertice x-coordinates
% yv: a 1-by-v vector of vertice y-coordinates
% distances: a 1-by-v vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope
% max_radius: distance from the mean to the furthest vertex
%
% Examples:
%      
%      % BASIC example
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      addpath([parent_dir '\' main_folder '\Path_Planning\visibility'])
%      xv = [2 3 5 6 6 5 3 2];
%      yv = [3 2 2 3 5 6 6 5];
%      polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
%      polytopes.xv = xv;
%      polytopes.yv = yv;
%      polytopes.distances = sum((polytopes.vertices(1:end-1,:) - polytopes.vertices(2:end,:)).^2,2).^0.5;
%      [Cx,Cy,polytopes.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(sum((polytopes.vertices(1:end-1,:) - ones(length(xv),1)*polytopes.mean).^2,2).^0.5);
%      point_tot = length(xv);
%      start = [0 0 point_tot+1 -1 0];
%      finish = [8 8 point_tot+2 0 0];
%      [clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_visibility_clear_and_blocked_points(polytopes,start,finish);
%      xings = fcn_visibility_line_polytope_intersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes);
%      fcn_plot_polytopes(polytopes,[],'b-',2,[],'square')
%      for ints = 1:length(D)
%          if D(ints)~=0
%              plot([xjP(ints) xjQ(ints)],[yjP(ints) yjQ(ints)],'m-','linewidth',2)
%          end
%      end
%      plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
%      for xing = 1:length(xings(end).index)
%          plot(xings(end).points(xing,1),xings(end).points(xing,2),'kx','linewidth',1)
%      end
%
% 
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method in function usage examples

% check input arguments
if nargin ~= 10
    error('Incorrect number of arguments');
end

num_vecs = length(num_int);
xings(num_vecs) = struct('points',[],'index',[],'obstacles',[]);
verts = length([polytopes.xv]);
xing_ind = 1:verts; % create an array of all vector indices that could intersect the reference vector

obs_id = zeros(1,verts);
curpt = 0;
for poly = 1:size(polytopes,2)
    verts = length(polytopes(poly).xv);
    obs_id(curpt+1:curpt+verts) = poly;
    curpt = curpt+verts;
end

for vecs = 1:num_vecs % check each line for intersections
    if num_int(vecs)~=0 % if there are intersections
        % calculate intersection locations and corresponding vector indices
        dimid = di(vecs,D(vecs,:)==1);
        xjPmid = xjP(D(vecs,:)==1);
        yjPmid = yjP(D(vecs,:)==1);
        notNaN = ~isnan(dimid);
        dimod(notNaN) = dimid(notNaN);
        xings(vecs).points = [(xiP(vecs)+dimod.*(xiQ(vecs)-xiP(vecs)))', (yiP(vecs)+dimod.*(yiQ(vecs)-yiP(vecs)))'];
        xings(vecs).points(~notNaN,:) = [xjPmid(~notNaN)' yjPmid(~notNaN)'];
        xings(vecs).index = xing_ind(D(vecs,:)==1);
        xings(vecs).obstacles = obs_id(xings(vecs).index);
    %else leave blank
    end
end
end