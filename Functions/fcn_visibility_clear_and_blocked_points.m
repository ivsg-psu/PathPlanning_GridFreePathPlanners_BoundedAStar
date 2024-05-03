function [clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = fcn_visibility_clear_and_blocked_points(polytopes,start,finish,varargin)
% FCN_VISIBILITY_CLEAR_AND_BLOCKED_POINTS determines whether the points in
% finish are blocked by a poltyope or not
%
% [CLEAR_PTS,BLOCKED_PTS,D,DI,DJ,NUM_INT,XIP,YIP,XIQ,YIQ,XJP,YJP,XJQ,YJQ]=FCN_VISIBILITY_CLEAR_AND_BLOCKED_POINTS(POLYTOPES,START,FINISH)
% returns:
% CLEAR_PTS: finish points that are not blocked by any obstacle with the
%   same information as FINISH
% BLOCKED_PTS: finish points that are blocked by any obstacle with the same
%   information as FINISH
% D: m-by-n intersection array, where m = number of finish points
%   and n = number of polytope edges, where 1 indicates that an
%   intersection occures between the START and FINISH point on the
%   corresponding polytope edge and 0 indicates no intersection
% DI: m-by-n intersection array, where each value gives the percentage of
%   how far along the path from START to FINISH the intersection occurs
% DI: m-by-n intersection array, where each value gives the percentage of
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
% START: a 1-by-5 vector of starting point information, including:
%   x-coordinate
%   y-coordinate
%   point id number
%   obstacle id number
%   beginning/ending indication (1 if the point is a beginning or ending
%   point and 0 otherwise)
%   Ex: [x y point_id obs_id beg_end]
% FINISH: a m-by-5 vector of ending points information, including the same
%   information as START
% (optional inputs)
% is_concave: set a 1 to allow for concave (i.e. non-convex) obstacles.  If this is left
%   blank or set to anyting other than 1, the function defaults to the convex behavior
%   which is more conservative (i.e. setting the flag wrong incorrectly may result in
%   suboptimal paths but not collisions). For background on what this flag does, see slides 9-14 here:
%   https://pennstateoffice365.sharepoint.com/:p:/r/sites/IntelligentVehiclesandSystemsGroup-Active/Shared%20Documents/IVSG/Theses/2025_Harnett_PhD/Weekly%20Updates/HARNETT_WEEKLY_UPDATE_JAN08_2024.pptx?d=w4f5e75a3c5b343aab47b41d2b945075b&csf=1&web=1&e=5otpZ3
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
%      addpath([parent_dir '\' main_folder '\Path_Planning\visibility'])
%      polytopes=fcn_polytope_generation_halton_voronoi_tiling(1,100,[100,100]);
%      trim_polytopes=fcn_polytope_editing_remove_edge_polytopes(polytopes,0,100,0,100);
%      shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);
%      xvert = [shrunk_polytopes.xv];
%      yvert = [shrunk_polytopes.yv];
%      point_tot = length(xvert);
%      start = [0 50 point_tot+1 -1 0];
%      finish = [[100; xvert'] [50; yvert'] [point_tot+2; [1:point_tot]'] [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];
%      [clear_pts,blocked_pts]=fcn_visibility_clear_and_blocked_points(shrunk_polytopes,start,finish);
%      fcn_plot_polytopes(shrunk_polytopes,[],'b-',2,[0 100 0 100],'square');
%      plot([start(1) finish(1,1)],[start(2) finish(1,2)],'kx','linewidth',1)
%      plot(clear_pts(:,1),clear_pts(:,2),'go','linewidth',1)
%      plot(blocked_pts(:,1),blocked_pts(:,2),'rx','linewidth',1)
%
%
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu
%

%% check input arguments
if nargin < 3 || nargin > 4
    error('Incorrect number of arguments');
end
% if there is no value in varargin...
if nargin == 3
    % default is to assume convex obstacles as this is conservative
    is_concave = 0;
end
% if there is a value in varargin...
if nargin == 4
    % check what it is
    if varargin{1} == 1
        % set concave flag if it was passed in
        is_concave = 1;
    elseif varargin{1} == 0
        is_concave = 0;
    else
        % throw error if it was passed in with an incorrect value
        error('optional argument is the is_concave flag and can either be 1 or 0')
    end
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


%% convert to vector points
[xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = fcn_convert_to_vector_points(polytopes,start,finish);

%% calculate intersection check matrices
[D,di,dj] = fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,start,finish,is_concave);

%% find indices of clear and blocked paths
num_int = sum(D,2); % 0s should corespond with a clear path to a vertex
clear_pts = finish(num_int==0,:);
blocked_pts = finish(num_int~=0,:);

%% create combination variables to pass to the intersection function
% ivectors = [xiP yiP xiQ yiQ];
% jvectors = [xjP; yjP; xjQ; yjQ];
% [row,col] = size(D);
% Dmats(1:row,1:col,1) = D;
% Dmats(1:row,1:col,2) = di;
% Dmats(1:row,1:col,3) = dj;
end



%% supporting functions

function [xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = fcn_convert_to_vector_points(polytopes,start,finish)
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
cur_ind = 0;
for obs = 1:size(polytopes,2)
    jump = length(polytopes(obs).xv);
    firsts(cur_ind+1) = 1;
    lasts(cur_ind+jump) = 1;
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

function [D,di,dj] = fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,start,finish,is_concave)
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

start_ind = start(3);
start_obs = start(4);
start_be = start(5);
finish_ind = finish(:,3);
finish_obs = finish(:,4);
finish_be = finish(:,5);

den = dxj.*dyi - dxi.*dyj;
di = (dxj.*(yjP-yiP) - dyj.*(xjP-xiP))./den;
dj = (dxi.*(yjP-yiP) - dyi.*(xjP-xiP))./den;


% Dpoints = sum((((dj<acc).*(dj>-acc))+((dj<1+acc).*(dj>1-acc))).*((di<=1).*(di>=0)),2);
% if start_obs > 0
%     Dpoints = Dpoints - 4; % subtract 2 points 2 for start point and 1 for finish point
%     Dpoints(end) = Dpoints(end)+2; % should only subtract 2 from the finish point since it's not on an obstacle
% else
%     Dpoints = Dpoints - 2; % subtract 1 points for finish point
%     Dpoints(end) = Dpoints(end)+2; % should only subtract 1 from the finish point since it's not on an obstacle
% end
%%%%%%%%%%%%%%%%%%%%% still struggling with NaN which occurs with lines
%%%%%%%%%%%%%%%%%%%%% parallel or perpendicular to the path
% midpoints = sum(isnan(di).*isnan(dj)+(di>=0).*(di<=1).*(dj==0),2) - (start_obs>0) - [ones(size(di,1)-1,1);0];
% midpoints = isnan(di).*isnan(dj)+(di>=0).*(di<=1).*(dj==0) - (xjP==xiP).*(yjP==yiP) - (xjP==xiQ).*(yjP==yiQ);
midpoints = (isnan(di).*isnan(dj)+(di>=0).*(di<=1).*(dj==0)) .* ((xiP~=xjP).*(yiP~=yjP) .* (xiP~=xjQ).*(yiP~=yjQ) .* (xiQ~=xjP).*(yiQ~=yjP) .* (xiQ~=xjQ).*(yiQ~=yjQ));
if is_concave
    % for nonconvex obstacles, we don't want to assume points are the same obstacle are blocked if not adjacent
    exceptions = 0;
else
    %remove if both on same obstacle and not adjacent points
    exceptions = ((start_obs==finish_obs).*((abs(start_ind-finish_ind)~=1)-(start_be.*finish_be))).*1;
end
acc = 1e-10;
% removing "exceptions" could allow for self-blocked points to be visible...
% but if a distinction is necessary between self-blocked points and points...
% visible through free space then "fcn_visibility_self_blocked_pts" is better
D = ((di<1-acc).*(di>acc)).*((dj<=1).*(dj>=0))+exceptions+midpoints;
% see page 8 of notes 10_04 here:
% https://www.me.psu.edu/sommer/me581/
% ((di<1-acc).*(di>acc)).*((dj<1-acc).*(dj>acc))
% from Seth:
% The first part is the actual "visibility" portion of identifying which points
% can be connected by a straight line without intersecting any other points.
% The exceptions part is where I make an exception for adjacent points
% that are on the same obstacle.
% The midpoints part is where I tried to handle situations where the starting
% point is on the line between two points.
end

