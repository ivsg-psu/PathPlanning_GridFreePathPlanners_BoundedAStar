function [max_dist] = fcn_bounding_ellipse_min_perimeter_path(int_polytopes,intersections,start,finish)
% FCN_BOUNDING_ELLIPSE_MIN_PERIMETER_PATH adds up all the distances
% between intersections along the start and finish and adds in the shortest
% perimeter distance around each intersected obstacle
%
% [MAX_DIST]=FCN_BOUNDING_ELLIPSE_MIN_PERIMETER_PATH(INT_POLYTOPES,INTERSECTIONS,START,FINISH)
% returns:
% MAX_DIST: sum of all the distances between intersections and along
%   minimum perimeters
%
% with inputs:
% INT_POLYTOPES: a 1-by-p seven field structure of intersected polytopes,
%   where p = number of intersected polytopes, with fields:
% vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%   the number of the individual polytope vertices
% xv: a 1-by-m vector of vertice x-coordinates
% yv: a 1-by-m vector of vertice y-coordinates
% distances: a 1-by-m vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope
% max_radius: distance from the mean to the furthest vertex
% INTERSECTIONS: a 1-by-f three field structure of intersection information, where
%   f = number of finish points, with fields:
% points: i-by-2 matrix of xy coordinates, where i=number of intersections
% index: 1-by-i vector of intersecting line indices
% obstacles: 1-by-i vector of obstacles intersecting the line
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
%      addpath([parent_dir '\' main_folder '\Path_Planning\bounding_ellipse'])
%      addpath([parent_dir '\' main_folder '\Path_Planning\visibility'])
%      polytopes=fcn_polytope_generation_halton_voronoi_tiling(1,100,[100,100]);
%      trim_polytopes=fcn_polytope_editing_remove_edge_polytopes(polytopes,0,100,0,100);
%      shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);
%      point_tot = length([shrunk_polytopes.xv]);
%      start = [0 50 point_tot+1 0 0];
%      finish = [100 50 point_tot+2 -1 0];
%      [clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_visibility_clear_and_blocked_points(shrunk_polytopes,start,finish);
%      intersections=fcn_visibility_line_polytope_intersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,shrunk_polytopes);
%      int_polytopes = shrunk_polytopes(intersections(end).obstacles);
%      max_dist=fcn_bounding_ellipse_min_perimeter_path(int_polytopes,intersections,start,finish);
%      fcn_plot_polytopes(shrunk_polytopes,99,'b-',2,[0 100 0 100],'square')
%      fcn_plot_polytopes(int_polytopes,99,'r-',2)
%      plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
%      for xing = 1:length(intersections(end).index)
%          plot(intersections(end).points(xing,1),intersections(end).points(xing,2),'kx','linewidth',1)
%      end
%
%
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu
%

%% check input arguments
if nargin ~= 4
    error('Incorrect number of arguments');
end
% TODO @sjharnett replace bug with estimated R_lc function to define boundary in
% new heuristic bounded A*
%% starting values
max_dist = 0; % start the max_dist at 0
points = intersections.points; % pull the intersection points from intersections
obstacles = intersections.obstacles; % pull the obstacles from the intersections

%% find if start and/or end points are on obstacles
startpt = start(1:2);
startobs = 0; % default value
for obs = 1:size(int_polytopes,2)
    [~,on] = inpolygon(start(1),start(2),int_polytopes(obs).vertices(:,1),int_polytopes(obs).vertices(:,2));
    if on % if we start on an obstacle we intersect
%         uobs = unique(intersections.obstacles); % intersected obstacles with no repeats
        startobs = intersections.obstacles(obs); % obstacle started on
        break
    end
end
finishpt = finish(1:2);
finishobs = 0; % default value
for obs = 1:size(int_polytopes,2)
    [~,on] = inpolygon(finish(1),finish(2),int_polytopes(obs).vertices(:,1),int_polytopes(obs).vertices(:,2));
    if on % if we start on an obstacle we intersect
        uobs = unique(intersections.obstacles); % intersected obstacles with no repeats
        finishobs = uobs(obs); % obstacle ended on
        break
    end
end
%% calculate the sum of distances between intersections and around perimeters
% find the distance to each intersection from the starting point
dist = round(fcn_general_calculation_euclidean_point_to_point_distance(ones(size(points,1),1)*startpt,points),10);
while ~isempty(points) % points not empty
    if startobs ~= 0 && max_dist == 0 % starting on an obstacle
        ind = find(obstacles==startobs,1); % index corresponding to that obstacle
%         min_dist = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        xing1 = startpt; % starting point is first perimeter point
        polytope = int_polytopes(ind);
        curobs = startobs;%% current obstacle
    else
        min_dist = min(dist); % find the minimum of the distances
        ind = find(abs(dist-min_dist) < 1e-15); % index of closest intersection
        if (length(ind) > 1) && (sum(obstacles(ind)==startobs)>0)
            ind = ind(obstacles(ind)==startobs);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        max_dist = max_dist + min_dist; % add the distance to the max_dist
        xing1 = points(ind,:); % set closest point as first perimeter point
        polytope = int_polytopes(ind); % polytope of interest
        int_polytopes(ind) = []; % remove polytope
        points(ind,:) = []; % remove point
        dist = dist - dist(ind); % remove extra distance
        dist(ind) = []; % remove distance
        curobs = obstacles(ind);
        obstacles(ind) = []; % remove obstacle
    end

    % find furthest point on current obstacle
    f_ind = find(dist == max(dist(obstacles == curobs)));
    if (length(f_ind) > 1) && (sum(obstacles(f_ind)==curobs)>0)
        ind2 = f_ind(obstacles(f_ind)==curobs);
    else
        ind2 = f_ind;
    end
    xing2 = points(ind2,:);

    % remove all points and obstacles in between
    rem_ind = (dist >= 0).*(dist < min(dist(f_ind)))==1;
    rem_ind(ind2) = true; % remove last point as well
    int_polytopes(rem_ind) = []; % remove polytope
    points(rem_ind,:) = []; % remove point
    dist = dist - dist(ind2); % remove extra distance
    dist(rem_ind) = []; % remove distance
    obstacles(rem_ind) = []; % remove obstacle

%     % assigne values to find the perimeter
%     if sum(obstacles == obstacles(ind)) == 1 % only intersecting an obstacle once
%         if obstacles(ind) == startobs % on starting obstacle away from the end
%             xing1 = startpt; % starting point is first perimeter point
%             xing2 = points(ind,:); % closest intersection is the next perimeter point
%             points(ind,:) = []; % remove point
%             dist = dist - dist(ind); % remove extra distance
%             dist(ind) = []; % remove distance
%             obstacles(ind) = []; % remove obstacle
%             polytope = int_polytopes(ind); % polytope of interest
%             int_polytopes(ind) = []; % remove polytope
%         elseif obstacles(ind) == finishobs % on ending obstacle
%             break % reached the last straight (this shouldn't occur)
%         else % some sort of single intersection
%             [in,on] = inpolygon(startpt(1),startpt(2),int_polytopes(ind).xv,int_polytopes(ind).yv);
%             if in && ~on % point within obstacle
%                 error('Possible point within obstacle') % removed to allow
%             else % hitting single vertex on obstacle
%                 max_dist = max_dist + min_dist; % distance to reach that point
%                 xing1 = points(ind,:); % closest intersection is the next perimeter point
%                 xing2 = points(ind,:); % closest intersection is the next perimeter point
%                 points(ind,:) = []; % remove point
%                 dist = dist - dist(ind); % remove extra distance
%                 dist(ind) = []; % remove distance
%                 obstacles(ind) = []; % remove obstacle
%                 polytope = int_polytopes(ind); % polytope of interest
%                 int_polytopes(ind) = []; % remove polytope
%             end
%         end
%     else % intersecting an obstacle multiple times
%         if obstacles(ind) == startobs % point of interest on starting obstacle
%             xing1 = startpt;
%             polytope = int_polytopes(startobs);
%             curobs = startobs;
%         else
%             max_dist = max_dist + min_dist; % add the distance to the max_dist
%             xing1 = points(ind,:); % set closest point as first perimeter point
%             polytope = int_polytopes(ind); % polytope of interest
%             int_polytopes(ind) = []; % remove polytope
%             points(ind,:) = []; % remove point
%             dist(ind) = []; % remove distance
%             curobs = obstacles(ind);
%             obstacles(ind) = []; % remove obstacle
%         end
%
%         % find furthest point on current obstacle
%         ind = find(dist == max(dist(obstacles == curobs)));
%         if (length(ind) > 1) && (sum(obstacles(ind)==curobs)>0)
%             ind = ind(obstacles(ind)==curobs);
%         end
%         xing2 = points(ind,:);
%
%         % remove all points and obstacles in between
%         rem_ind = (dist >= min_dist).*(dist <= dist(ind))==1;
%         int_polytopes(rem_ind) = []; % remove polytope
%         points(rem_ind,:) = []; % remove point
%         dist = dist - dist(ind); % remove extra distance
%         dist(rem_ind) = []; % remove distance
%         obstacles(rem_ind) = []; % remove obstacle
%     end
    % calculate the perimeters
    [perimeter1,perimeter2] = fcn_polytope_calculation_dual_perimeters(polytope,xing1,xing2);
    % add the smallest perimeter to the max_dist
    if perimeter1 < perimeter2
        max_dist = max_dist + perimeter1;
    else
        max_dist = max_dist + perimeter2;
    end
    startpt = xing2; % make xing2 the next starting point
end
% add the distance from the last intersection to the end point
max_dist = max_dist + fcn_general_calculation_euclidean_point_to_point_distance(startpt,finishpt);
