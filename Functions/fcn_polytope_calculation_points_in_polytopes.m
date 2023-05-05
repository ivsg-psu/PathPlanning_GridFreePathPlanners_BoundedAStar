function [err,Apoly,Bpoly] = fcn_polytope_calculation_points_in_polytopes(A,B,polytopes,throw_error,varargin)
% FCN_POLYTOPE_CALCULATIONS_POINTS_IN_POLYTOPES see if point A or B is in one of the
% polytopes
%
% [ERR]=FCN_POLYTOPE_CALCULATIONS_POINTS_IN_POLYTOPES(A,B,POLYTOPES,THROW_ERROR,EDGE_CHECK)
% returns:
% ERR: value of 0 if no points in any polytopes or 1 if a point is withing
% a polytope
% APOLY: polytope A is on
% BPOLY: polytope B is on
%
% with inputs:
% A: two field structure of x and y coordinates of starting point A, with 
%   fields:
% x: x coordinate
% y: y coordinate
% B: two field structure of x and y coordinates of ending point B, with 
%   fields:
% x: x coordinate
% y: y coordinate
% polytopes: a 1-by-n seven field structure of combined polytopes, where 
%   p = number of polytopes, with fields:
% vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%   the number of the individual polytope vertices
% xv: a 1-by-m vector of vertice x-coordinates
% yv: a 1-by-m vector of vertice y-coordinates
% distances: a 1-by-m vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope
% max_radius: distance from the mean to the furthest vertex
% THROW_ERROR: flag determining whether an error should be thrown (1) for
% points inside any polytope or no error and value assinged to ERR (0) 
% EDGE_CHECK: flag indicating whether it should be checked whether the
% points are on any edges
%
% Examples:
%      
%      % Example 1
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      xv = [-2 -1 1 2 2 1 -1 -2];
%      yv = [-1 -2 -2 -1 1 2 2 1];
%      polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
%      polytopes.xv = xv;
%      polytopes.yv = yv;
%      polytopes.distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),polytopes.vertices(2:end,:));
%      [Cx,Cy,polytopes.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),ones(length(xv),1)*polytopes.mean));
%      A.x = 0;
%      A.y = 0;
%      B.x = 5;
%      B.y = 5;
%      fcn_plot_polytopes(polytopes,101,'b-',2,[-6 6 -6 6],'square')
%      plot([A.x B.x],[A.y B.y],'kx','linewidth',2)
%      try
%          fcn_polytope_calculation_points_in_polytopes(A,B,polytopes)
%      catch
%          plot([A.x B.x],[A.y B.y],'rx','linewidth',2)
%      end
%
%      % Example 2
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      xv = [-2 -1 1 2 2 1 -1 -2];
%      yv = [-1 -2 -2 -1 1 2 2 1];
%      polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
%      polytopes.xv = xv;
%      polytopes.yv = yv;
%      polytopes.distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),polytopes.vertices(2:end,:));
%      [Cx,Cy,polytopes.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),ones(length(xv),1)*polytopes.mean));
%      A.x = -5;
%      A.y = -5;
%      B.x = 0;
%      B.y = 0;
%      fcn_plot_polytopes(polytopes,102,'b-',2,[-6 6 -6 6],'square')
%      plot([A.x B.x],[A.y B.y],'kx','linewidth',2)
%      try
%          fcn_polytope_calculation_points_in_polytopes(A,B,polytopes)
%      catch
%          plot([A.x B.x],[A.y B.y],'rx','linewidth',2)
%      end
%
%      % Example 3
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      xv = [-2 -1 1 2 2 1 -1 -2];
%      yv = [-1 -2 -2 -1 1 2 2 1];
%      polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
%      polytopes.xv = xv;
%      polytopes.yv = yv;
%      polytopes.distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),polytopes.vertices(2:end,:));
%      [Cx,Cy,polytopes.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),ones(length(xv),1)*polytopes.mean));
%      A.x = -5;
%      A.y = -5;
%      B.x = 5;
%      B.y = 5;
%      fcn_plot_polytopes(polytopes,103,'b-',2,[-6 6 -6 6],'square')
%      plot([A.x B.x],[A.y B.y],'kx','linewidth',2)
%      try
%          fcn_polytope_calculation_points_in_polytopes(A,B,polytopes)
%      catch
%          plot([A.x B.x],[A.y B.y],'rx','linewidth',2)
%      end
% 
% This function was written on 2018_12_18 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%


% check input arguments
if nargin == 4
    edge_check = 0;
elseif nargin == 5
    edge_check = varargin{1};
else
    error('Incorrect number of arguments');
end

err = 0;
Apoly = -1;
Bpoly = 0;
for polys = 1:size(polytopes,2)
    [Ain,Aon]=inpolygon(A.x,A.y,polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2));
    [Bin,Bon]=inpolygon(B.x,B.y,polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2));
    if Ain*~Aon==1 % if A in polygon
        if throw_error == 1
            error('Point A within obstacle')
        else
            err = 1;
        end
    elseif Bin*~Bon==1 % if B in polygon
        if throw_error == 1
            error('Point B within obstacle')
        else
            err = 1;
        end
    end
    if edge_check == 1
        if Aon
            Apoly = polys;
        end
        if Bon
            Bpoly = polys;
        end
    end
                
end
end