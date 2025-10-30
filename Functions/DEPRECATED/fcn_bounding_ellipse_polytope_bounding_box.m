function [bound_polytopes,bound_box,bound_pts,all_bound]= fcn_bounding_ellipse_polytope_bounding_box(start,finish,polytopes,all_pts,bound_pts,offsetperp,offsetpara)

warning('on','backtrace');
warning(['fcn_boudning_ellipse_polytope_bounding_box is being deprecated. ' ...
    'Use fcn_BoundedAStar_calculateBoundingEllipsePolytope instead.']);
% FCN_BOUNDING_ELLIPSE_POLYTOPE_BOUNDING_BOX creates a bounding box around 
% the straight line between A and B and then finds the points within the 
% bounding box and their corresponding polytopes
%
% [BOUND_POLYTOPES,BOUND_BOX,BOUND_PTS,ALL_BOUND]=FCN_BOUNDING_ELLIPSE_POLYTOPE_BOUNDING_BOX(START,FINISH,POLYTOPES,ALL_PTS,BOUND_PTS,OFFSETPERP,OFFSETPARA)
% returns:
% BOUND_POLYTOPES: a 1-by-p seven field structure of bound polytopes, where  
%   p = number of bound polytopes, with fields:
% vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%   the number of the individual polytope vertices
% xv: a 1-by-m vector of vertice x-coordinates
% yv: a 1-by-m vector of vertice y-coordinates
% distances: a 1-by-m vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope 
% max_radius: distance from mean to the furthest vertex
% BOUND_BOX: a 4-by-2 matrix of xy corner points of the bounding box 
% BOUND_PTS: a b-by-5 matrix of bound point information that is a subset of
%   previous bound points, where b = number of bound points, including:
%   x-coordinate
%   y-coordinate
%   point id number
%   obstacle id number 
%   beginning/ending indication (1 if the point is a beginning or ending
%   point and 0 otherwise) 
%   Ex: [x y point_id obs_id beg_end]
% ALL_BOUND: w-by-5 matrix of bound point information that is a subset of
%   all map points, where w = number of all bound map points
%
% with inputs:
% START: a 1-by-5 vector of starting point information with the same
%   information as BOUND_PTS
% FINISH: a 1-by-5 vector of finishing point information with the same
%   information as BOUND_PTS
% POLYTOPES: the original polytopes with the same fields as BOUND_POLYTOPES 
% ALL_PTS: a-by-5 matrix of all map points, where a = number of map points
% BOUND_PTS: B-by-5 matrix of the original bound points, where B = number
%   of original bound points
% OFFSETPERP: 1-by-2 vector indicating how far to offset bounding box sides 
%   from the straight line [+offset, -offset]
% OFFSETPARA: 1-by-2 vectore indicating how far to offset bounding box 
%   along the the straight line [+offset, -offset]
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
%      addpath([parent_dir '\' main_folder '\Path_Planning\bounding_ellipse'])
%      polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[],[100 100],-1);
%      trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0 0 100 100], (-1));
%      shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);
%      xv = [shrunk_polytopes.xv];
%      yv = [shrunk_polytopes.yv];
%      point_tot = length(xv);
%      start = [0 50 point_tot+1 0 0];
%      finish = [100 50 point_tot+2 -1 0];
%      beg_end = zeros(point_tot,1);
%      obs_id = zeros(point_tot,1);
%      curpt = 0;
%      for poly = 1:size(shrunk_polytopes,2) % check each polytope
%          verts = length(shrunk_polytopes(poly).xv);
%          obs_id(curpt+1:curpt+verts) = ones(verts,1)*poly; % obs_id is the same for every vertex on a single polytope
%          beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
%          curpt = curpt+verts;
%      end
%      all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
%      bound_pts = all_pts;
%      offsetperp=[15 15];
%      offsetpara = [2 2];
%      [bound_polytopes,bound_box,bound_pts,all_bound]= fcn_bounding_ellipse_polytope_bounding_box(start,finish,shrunk_polytopes,all_pts,bound_pts,offsetperp,offsetpara);
%      fcn_plot_polytopes(shrunk_polytopes,100,'r-',2,[-5 105 0 100],'square')
%      plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
%      fcn_plot_polytopes(bound_polytopes,100,'b-',2)
%      plot([bound_box(:,1); bound_box(1,1)], [bound_box(:,2); bound_box(1,2)],'k--','linewidth',2)
%      plot(bound_pts(:,1),bound_pts(:,2),'go','linewidth',1)
%
%      
%
% This function was written on 2018_12_21 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

%% check input arguments
if nargin ~= 7
    error('Incorrect number of arguments');
end

%% calculate a bounding box around the region of interest
dx = finish(1)-start(1); % change in line
dy = finish(2)-start(2);
magx = dx/sqrt(dx^2+dy^2); % magnitude of change in both directions
magy = dy/sqrt(dx^2+dy^2);
% bounding box vertices
bound_box = [(start(1)+offsetperp(1)*magy-offsetpara(1)*magx) (start(2)-offsetperp(1)*magx-offsetpara(1)*magy); (finish(1)+offsetperp(1)*magy+offsetpara(2)*magx) (finish(2)-offsetperp(1)*magx+offsetpara(2)*magy); (finish(1)-offsetperp(2)*magy+offsetpara(2)*magx) (finish(2)+offsetperp(2)*magx+offsetpara(2)*magy); (start(1)-offsetperp(2)*magy-offsetpara(1)*magx) (start(2)+offsetperp(2)*magx-offsetpara(1)*magy)];

%% find vertices within bounding box
all_bound = all_pts(inpolygon(all_pts(:,1),all_pts(:,2),bound_box(:,1),bound_box(:,2)),:);
bound_pts = bound_pts(inpolygon(bound_pts(:,1),bound_pts(:,2),bound_box(:,1),bound_box(:,2)),:);

%% find the corresponding obstacles
bound_polytopes = polytopes(unique(all_bound(:,4)));

%% plot for troubleshooting
% fcn_plot_polytopes(comb_polytopes,101,'r-',2,[0 200 0 200],'square')
% plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
% fcn_plot_polytopes(bound_polytopes,101,'b-',2)
% plot(bound_box(:,1),bound_box(:,2),'k--','linewidth',2)
% plot(bound_pts(:,1),bound_pts(:,2),'go','linewidth',1)
