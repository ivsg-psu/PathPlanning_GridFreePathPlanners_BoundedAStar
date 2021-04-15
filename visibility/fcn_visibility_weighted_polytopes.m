function [clear_pts,blocked_pts,total_weighted_dist,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = fcn_visibility_weighted_polytopes(polytopes,polyweights,start,finish)
% FCN_VISIBILITY_WEIGHTED_POLYTOPES finds polytopes intersection points on 
% lines from start to finish points (The weight calculation assumes no 
% overlapping obstacles and that neither the start nor the finish are 
% within an obstacle)
%
% [CLEAR_PTS,BLOCKED_PTS,TOTAL_WEIGHTED_DIST,D,DI,DJ,NUM_INT,XIP,YIP,XIQ,YIQ,XJP,YJP,XJQ,YJQ]=FCN_VISIBILITY_WEIGHTED_POLYTOPES(POLYTOPES,POLYWEIGHTS,START,FINISH)
% returns:
% CLEAR_PTS: finish points that are not blocked by any obstacle with the
%   same information as FINISH
% BLOCKED_PTS: finish points that are blocked by any obstacle with the same 
%   information as FINISH 
% TOTAL_WEIGHTED_DIST: weighted distance of the direct path to each FINISH 
%   point 
% D: m-by-n intersection array, where m = number of finish points 
%   and n = number of polytope edges, where 1 indicates that an
%   intersection occures between the START and FINISH point on the
%   corresponding polytope edge and 0 indicates no intersection
% DI: m-by-n intersection array, where each value gives the percentage of
%   how far along the path from START to FINISH the intersection occurs
% DJ: m-by-n intersection array, where each value gives the percentage of
%   how far along the polytope edge the intersection occurs
% NUM_INT: m-by-1 vector of the number of intersections between the START
%   and each point in FINISH 
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
% XJQ: 1-by-n vector of the x-coordinates of the finishing point of the
%   polytope edge
% YJQ: 1-by-n vector of the y-coordinates of the finishing point of the
%   polytope edge
%
% with inputs:
% POLYTOPES: a 1-by-p seven field structure of polytopes, where  
%   p = number of polytopes, with fields:
% vertices: a v+1-by-2 matrix of xy points with row1 = rowv+1, where v is
%   the number of the individual polytope vertices
% xv: a 1-by-v vector of vertice x-coordinates
% yv: a 1-by-v vector of vertice y-coordinates
% distances: a 1-by-v vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope
% max_radius: distance from the mean to the furthest vertex
% POLYWEIGHTS: p-by-1 vector of weights associated with each polytope
% START: a 1-by-5 vector of starting point information, including:
%   x-coordinate
%   y-coordinate
%   point id number
%   obstacle id number 
%   beginning/ending indication (1 if the point is a beginning or ending
%   point and 0 otherwise) 
%   Ex: [x y point_id obs_id beg_end]
% FINISH: a m-by-5 vector of ending point information, including the same
%   information as START, where m is the number of finishing points
%
% Examples:
%      
%      This function is still in development
% 
% This function was written on 2020_02_04 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

%% check input arguments
if nargin ~= 4
    error('Incorrect number of arguments');
end

%% check orientation of start and finish
[row, col] = size(start);
if row == 5
    if col == 1 % oriented sideways
        start = start';
    else 
        error('Incorrect number of coordinates in start')
    end
elseif row == 1
    if col ~= 5
        error('Incorrect number of coordinates in start')
    end
else
    error('Incorrect number of coordinates in start')
end

[row, col] = size(finish);
if col ~= 5
    if row == 5 % oriented sideways
        finish = finish';
    else 
        error('Incorrect number of coordinates in finish')
    end
end

%% add necessary directories
cur_path = pwd;
main_folder = '!Voronoi Tiling Obstacles - Organized';
parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
addpath([parent_dir '\' main_folder '\General_Calculation'])

%% convert to vector points
[xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,obs_id] = fcn_convert_to_vector_points(polytopes,start,finish);

%% calculate intersection check matrices
[D,di,dj,collinear] = fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,start,finish);
 
%% find indices of clear and blocked paths
num_int = sum(D.*~collinear,2); % 0s should corespond with a clear path to a vertex (.*~collinear removes double counting)
clear_pts = finish(num_int<=1,:); % only hitting the desired point (can be 0 if the point is not on an obstacle (ie goal point))
blocked_pts = finish(num_int>1,:); % multiple intersections
 
%% find unweighted distances to each point
distances = fcn_general_calculation_euclidean_point_to_point_distance([xiP yiP],[xiQ yiQ]); 

goal_vec = finish(:,4)==0; % extra column added to di and D to ensure the goal point is properly detected
%%%%%%
collin_D = [D.*collinear zeros(size(collinear,1),1)]; % start points of collinear edges
extra = mod(sum(collin_D,2),2); % rows needing extra values
% collin2 = [collinear zeros(size(collinear,1),1)];
%%%%%%

D_dist = [(di.*D) goal_vec].*distances; % distances to each intersection

%%%%%
collin_dist = collin_D.*D_dist; % distance to start points on collinear edges
[max_dist, md_ind] = max(collin_dist,[],2); % maximum or each row
[e_Dist,e_ind] = sort([collin_dist max_dist.*extra],2); % extra distance to weight (weight of 1 will double the distance)
e_ind(extra==1,end) = md_ind(extra==1); % change index to match the added values
non_collin_dist = D_dist.*[~collinear ones(size(collinear,1),1)]; % remove the double counting
%%%%%

obs_id = [obs_id length(polyweights)]; % add unused id for the goal point
[D_dist_sort,sort_ind] = sort(non_collin_dist,2); % intersection distances sorted from least to most

sort_pos = find(D_dist_sort~=0,1)-1; % position immediately before first nonzero term
rows = size(D_dist_sort,1); % number of rows in D_dist_sort
first_keep = ((sort_pos-mod(sort_pos,rows))/rows); % first column to keep
Dds = D_dist_sort(:,first_keep:end); % D_dist_sort with only one zero column
si = sort_ind(:,first_keep:end); % corresponding sort_ind
id_mat = obs_id(si).*(Dds~=0); % obstacle each intersection corresponds with

start_in = nan;
% start_weight = polyweights(end);
% finish_in = nan(size(finish,1),1);
for polys = 1:size(polytopes,2)
    if max(inpolygon(start(1),start(2),polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2))) % if A in polygon
%         start_in = polys;
        id_mat(:,1) = polys;
%         start_weight = polyweights(polys);
%     elseif max(inpolygon(finish(:,1),finish(:,2),polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2))) % if B in polygon
%         finish_in = polys;
    end
end
% start_id = id_mat(:,2:end) == start_in;


[id_mat_sort,id_sort_ind] = sort(id_mat,2);
[row,~] = size(id_sort_ind);
id_sort_ind = (id_sort_ind-1)*row + (1:row)';
% non_zero_id = id_mat_sort~=0;

id_same = (id_mat_sort(:,2:end)-id_mat_sort(:,1:end-1))==0; % checks for same ids between each horizontal entry

Dds = Dds(id_sort_ind);
D_dist_change = Dds(:,2:end)-Dds(:,1:end-1); % unweighted distance between each pair
inner_dist = D_dist_change.*id_same;
% outer_dist = D_dist_change.*~id_same;
outer_dist = max(Dds,[],2)-sum(inner_dist,2); % this assumes the inner_dist is correct and does not include distance along edges


%%% find weights for inner_dist
sorted_id_same = id_mat_sort(:,2:end).*id_same;
sorted_id_same(sorted_id_same==0) = length(polyweights); % set all zeros to the default value
inner_weights = polyweights(sorted_id_same);
inner_weighted_dist = inner_weights.*inner_dist;
%%% weight for outer_dist is the default
outer_weighted_dist = polyweights(end).*outer_dist;
%%% weights for e_Dist should be the minimum of the two possibilities
obs_id2 = [obs_id obs_id(end)];
obs_ind = obs_id2(e_ind);
ind_weights = polyweights(obs_ind)-polyweights(end); % the default weight was already accounted for
weight_choice = min(ind_weights(:,1:2:end),ind_weights(:,2:2:end));
delta_e_Dist = e_Dist(:,2:2:end)-e_Dist(:,1:2:end);
weighted_e_Dist = delta_e_Dist.*weight_choice; 

%%%%%%%%%% not quite getting the edge distance correct when an edge is not
%%%%%%%%%% shared between two polytopes

% total_weighted_dist = sum(inner_weighted_dist,2) + sum(outer_weighted_dist,2);
total_weighted_dist = sum(inner_weighted_dist,2) + outer_weighted_dist + sum(weighted_e_Dist,2);


% weights = same_weights(:,2:end).*id_same + polyweights(end).*(~id_same.*~start_id) + start_weight.*(~id_same.*start_id); % make weights for each pair of nonzero ids (default is polyweights(end))
% 
% D_dist_change = Dds(:,2:end)-Dds(:,1:end-1); % unweighted distance between each pair
% 
% path_weights = D_dist_change.*weights; % weighted distance between each pair
% 
% total_weights = sum(path_weights,2); % total weight for each path

%%%%%%%%%%% The weight calculation assumes no overlapping obstacles and
%%%%%%%%%%% the finish is not within an obstacle

end

%% supporting functions

function [xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,obs_id] = fcn_convert_to_vector_points(polytopes,start,finish)
% FCN_CONVERT_TO_VECTOR_POINTS takes in polytope structures and converts
% them to vectors of starting and ending points of lines along the
% perimeter of the polytopes
%
% [XIP,YIP,XIQ,YIQ,XJP,YJP,XJQ,YJQ]=FCN_CONVERT_TO_VECTOR_POINTS(POLYTOPES,STARTX,STARTY,ENDX,ENDY)
% returns:
% XIP: a f-by-1 vector of starting point x positions, f = total finish
%     points
% YIP: a f-by-1 vector of starting point y positions
% XIQ: a f-by-1 vector of ending point x positions
% YIQ: a f-by-1 vector of ending point y positions
% XJP: a 1-by-n vector of starting point x positions, n = number of
%     polytope points
% YJP: a 1-by-n vector of starting point y positions
% XJQ: a 1-by-n vector of ending point x positions
% YJQ: a 1-by-n vector of ending point y positions
%
% with inputs:
% POLYTOPES: a 1-by-p seven field structure of polytopes, where  
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
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      addpath([parent_dir '\' main_folder '\Path_Planning\visibility'])
%      xv = [2 3 5 6 6 5 3 2];
%      yv = [3 2 2 3 5 6 6 5];
%      polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
%      polytopes.xv = xv;
%      polytopes.yv = yv;
%      polytopes.distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),polytopes.vertices(2:end,:));
%      [Cx,Cy,polytopes.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),ones(length(xv),1)*polytopes.mean));
%      point_tot = length(xv);
%      start = [0 0 point_tot+1 -1 0];
%      finish = [8 8 point_tot+2 0 0];
%      [xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_convert_to_vector_points(polytopes,start,finish);
%      fcn_plot_polytopes(polytopes,[],'b-',1,[0 8 0 8],'square')
%      plot([start(1) finish(1)],[start(2) finish(2)],'kx','linewidth',1)
%
% 
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

xjP = [polytopes.xv];
yjP = [polytopes.yv];

firsts = zeros(1,length(xjP));
lasts = firsts;
% weights = firsts;
obs_id = firsts;
cur_ind = 0;
for obs = 1:size(polytopes,2)
    jump = length(polytopes(obs).xv);
    firsts(cur_ind+1) = 1;
    lasts(cur_ind+jump) = 1;
%     weights(cur_ind+1:cur_ind+jump) = polyweights(obs);
    obs_id(cur_ind+1:cur_ind+jump) = obs;
    cur_ind = cur_ind+jump;
end
xjQ(1,lasts==1) = xjP(firsts==1);
xjQ(1,lasts==0) = xjP(firsts==0);
yjQ(1,lasts==1) = yjP(firsts==1);
yjQ(1,lasts==0) = yjP(firsts==0);

xiQ = finish(:,1); % multiple possible ending points
yiQ = finish(:,2);
lenQ = length(xiQ);
xiP = ones(lenQ,1)*start(1); % one starting point
yiP = ones(lenQ,1)*start(2);
end

function [D,di,dj,collinear] = fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,start,finish)
% FCN_CALCULATE_INTERSECTION_MATRICES calculates matrices of values
% indicating whether two lines intersect. Intersections occur when the
% value of di and corresponding value of dj are both in the interval [0,1]
%
% [D,DI,DJ]=FCN_CALCULATE_INTERSECTION_MATRICES(XIP,YIP,XIQ,YIQ,XJP,YJP,XJQ,YJQ)
% returns:
% D: m-by-n intersection matrix, where m = number of finish points and 
%   n = number of polytope edges, indicating whether a line intersects 
%   another line with a 0 (not intersecting) or 1 (intersecting)
% DI: m-by-n matrix indicating how far along the i line the j line would 
%   intersect it as a ratio of the length of line i   
% DJ: m-by-n matrix indicating how far along the j line the i line would 
%   intersect it as a ratio of the length of line j
%
% with inputs:
% XIP: a n-by-1 vector of starting point x positions, n = total polytope
%   vertices
% YIP: a n-by-1 vector of starting point y positions
% XIQ: a n-by-1 vector of ending point x positions
% YIQ: a n-by-1 vector of ending point y positions
% XJP: same as XIP
% YJP: same as YIP
% XJQ: same as XIQ
% YJQ: same as YIQ
%
%
% Examples:
%      
%      % BASIC example
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation])
%      addpath([parent_dir '\' main_folder '\Path_Planning\visibility'])
%      xv = [2 3 5 6 6 5 3 2];
%      yv = [3 2 2 3 5 6 6 5];
%      polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
%      polytopes.xv = xv;
%      polytopes.yv = yv;
%      polytopes.distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(1).vertices(1:end-1,:),polytopes(1).vertices(2:end,:));
%      [Cx,Cy,polytopes.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytopes.vertices(1:end-1,:),ones(length(xv),1)*polytopes.mean));
%      point_tot = length(xv);
%      start = [0 0 point_tot+1 -1 0];
%      finish = [8 8 point_tot+2 0 0];
%      [xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_convert_to_vector_points(polytopes,start,finish);
%      [D,di,dj]=fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ);
%      num_int = D;
%      fcn_plot_polytopes(polytopes,[],'b-',2,[],'square')
%      for ints = 1:length(num_int)
%          if num_int(ints)~=0
%              plot([xjP(ints) xjQ(ints)],[yjP(ints) yjQ(ints)],'m-','linewidth',2)
%          end
%      end
%      plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
%
%
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

dxi = xiQ-xiP;
dyi = yiQ-yiP;
dxj = xjQ-xjP;
dyj = yjQ-yjP;

% start_ind = start(3);
% start_obs = start(4);
% start_be = start(5);
% finish_ind = finish(:,3);
% finish_obs = finish(:,4);
% finish_be = finish(:,5);

den = dxj.*dyi - dxi.*dyj;
di = (dxj.*(yjP-yiP) - dyj.*(xjP-xiP))./den;
dj = (dxi.*(yjP-yiP) - dyi.*(xjP-xiP))./den;
%%% isnan(di).*isnan(dj) = lines are parallel and colinear
collinear = isnan(di).*isnan(dj);

di_dist = sqrt(((xjP-xiP).^2 + (yjP-yiP).^2)./(dxi.^2+dyi.^2));
di_dist(isnan(di_dist))= 0; % this occurs when on the point
di(isnan(di)) = di_dist(isnan(di));
dj(isnan(dj)) = 0;
%%% isinf(di).*isinf(dj) = lines are parallel but not colinear
di(isinf(di)) = 1e15;
dj(isinf(dj)) = 1e15;
% collin_check = (isnan(di).*isnan(dj).*((((yiP<=yjP).*(yjP<=yiQ))+((yiP>=yjP).*(yjP>=yiQ)))>0).*((((xiP<=xjP).*(xjP<=xiQ))+((xiP>=xjP).*(xjP>=xiQ)))>0));
% midpoints = isnan(di).*isnan(dj)+(di>=0).*(di<=1).*(dj==0) - (xjP==xiP).*(yjP==yiP) - (xjP==xiQ).*(yjP==yiQ);
% exceptions = ((start_obs==finish_obs).*((abs(start_ind-finish_ind)~=1)-(start_be.*finish_be))).*1;
acc = 1e-10;
% D = ((di<1-acc).*(di>acc)).*((dj<1-acc).*(dj>acc))+exceptions+midpoints;
D = ((di<=1).*(di>=0)).*((dj<1-acc).*(dj>=0)); % anywhere on the potential path and anywhere except the end of the potential intersecting line

end

            