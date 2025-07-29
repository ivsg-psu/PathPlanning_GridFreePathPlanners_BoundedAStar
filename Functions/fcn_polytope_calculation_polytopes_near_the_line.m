function [close_polytopes] = fcn_polytope_calculation_polytopes_near_the_line(start,finish,polytopes)
% FCN_POLYTOPE_CALCULATION_POLYTOPES_NEAR_THE_LINE find polytopes possibly 
% within reach of the line of interest
%
% [CLOSE_POLYTOPES]=FCN_POLYTOPE_CALCULATION_POLYTOPES_NEAR_THE_LINE(START,FINISH,POLYTOPES)
% returns:
% CLOSE_POLYTOPES: a 1-by-n seven field structure of polytopes near the 
%   line,where n = number of close polytopes in polytopes, with fields:
% vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%   the number of the individual polytope vertices
% xv: a 1-by-m vector of vertice x-coordinates
% yv: a 1-by-m vector of vertice y-coordinates
% distances: a 1-by-m vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope
% max_radius: distance from the mean to the farthest vertex
%
% with inputs:
% POLYTOPES: the original 1-by-i six field structure with the same fields,
%   where i = number of original polytopes
% START: a 1-by-5 vector of starting point information, including:
%   x-coordinate
%   y-coordinate
%   point id number
%   obstacle id number 
%   beginning/ending indication (1 if the point is a beginning or ending
%   point and 0 otherwise) 
%   Ex: [x y point_id obs_id beg_end]
% FINISH: a 1-by-5 vector of ending point information, including the same
%   information as START
%
% Examples:
%      
%      % BASIC example
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_generation'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_editing'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[],[100 100],-1);
%      trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0 0 100 100], (-1));
%      shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);
%      point_tot = length([shrunk_polytopes.xv]);
%      A = struct('x',0,'y',50);
%      B = struct('x',100,'y',50);
%      start = [A.x A.y point_tot+1 -1 0]; 
%      finish = [B.x B.y point_tot+2 0 0];
%      close_polytopes=fcn_polytope_calculation_polytopes_near_the_line(start,finish,shrunk_polytopes);
%      fcn_plot_polytopes(shrunk_polytopes,100,'b-',2,[0 100 0 100],'square');
%      fcn_plot_polytopes(close_polytopes,100,'g-',2);
%      plot([A.x B.x],[A.y B.y],'kx','linewidth',1)
%      plot([A.x B.x],[A.y B.y],'k--','linewidth',2)
%
% 
% This function was written on 2018_12_18 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 

% check input arguments
if nargin ~= 3
    error('Incorrect number of arguments');
end

% main code
radius_of_polys = [polytopes.max_radius]';
radius_of_polys_squared = radius_of_polys.^2;
centers_of_polys = reshape([polytopes.mean],2,length(polytopes))';

dist_from_line_to_polys_squared = fcn_general_calculation_point_to_line_distances_squared([centers_of_polys zeros(size(centers_of_polys,1),1)],[start(1:2) 0],[finish(1:2) 0]);

ref_vec = ones(size(centers_of_polys,1),1)*(finish(1:2) - start(1:2));
straight = sum((start(1:2) - finish(1:2)).^2,2).^0.5;
para_dists = (dot(ref_vec,(centers_of_polys-start(1:2)),2))/straight;

close_polytopes = polytopes((radius_of_polys_squared>dist_from_line_to_polys_squared).*(para_dists+radius_of_polys>=0).*(para_dists-radius_of_polys<=straight) == 1);

