function [clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = fcn_Visibility_clearAndBlockedPoints(polytopes,start,finish,varargin)
% fcn_Visibility_clearAndBlockedPoints 
% 
% determines whether the points in
% finish are blocked by a polytope or not
%
% FORMAT: 
% [clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = fcn_Visibility_clearAndBlockedPoints(polytopes, start, finish, (isConcave), (fig_num))
%
% INPUTS:
%
%   polytopes: a 1-by-p seven field structure of polytopes, where
%   p = number of polytopes, with fields:
%       vertices: a v+1-by-2 matrix of xy points with row1 = rowv+1, where v is
%           the number of the individual polytope vertices
%       xv: a 1-by-v vector of vertice x-coordinates
%       yv: a 1-by-v vector of vertice y-coordinates
%       distances: a 1-by-v vector of perimeter distances from one point to the
%           next point, distances(i) = distance from vertices(i) to vertices(i+1)
%       mean: average xy coordinate of the polytope
%       area: area of the polytope
%       max_radius: distance from the mean to the furthest vertex
%
%   start: a 1-by-5 vector of starting point information, including:
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number
%       beginning/ending indication (1 if the point is a beginning or ending
%           point and 0 otherwise)
%       Ex: [x y point_id obs_id beg_end]
%
%   finish:  a m-by-5 vector of ending points information, including the same
%   information as START
%
%   (optional inputs)
%
%   isConcave: set a 1 to allow for concave (i.e. non-convex) obstacles.  If this is left
%       blank or set to anyting other than 1, the function defaults to the convex behavior
%       which is more conservative (i.e. setting the flag wrong incorrectly may result in
%       suboptimal paths but not collisions). For background on what this flag does, see slides 9-14 here:
%       https://pennstateoffice365.sharepoint.com/:p:/r/sites/IntelligentVehiclesandSystemsGroup-Active/Shared%20Documents/IVSG/Theses/2025_Harnett_PhD/Weekly%20Updates/HARNETT_WEEKLY_UPDATE_JAN08_2024.pptx?d=w4f5e75a3c5b343aab47b41d2b945075b&csf=1&web=1&e=5otpZ3
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose
%
% OUTPUTS:
%   
%   clear_pts: finish points that are not blocked by any obstacle with the
%   same information as FINISH
%
%   blocked_pts: finish points that are not blocked by any obstacle with the
%   same information as FINISH
%
%   D: m-by-n intersection array, where m = number of finish points
%       and n = number of polytope edges, where 1 indicates that an
%       intersection occures between the START and FINISH point on the
%       corresponding polytope edge and 0 indicates no intersection
%
%   di: m-by-n intersection array, where each value gives the percentage of
%   how far along the path from START to FINISH the intersection occurs
%
%   dj: m-by-n intersection array, where each value gives the percentage of
%   how far along the polytope edge the intersection occurs
%
%   num_int: m-by-1 vector of the number of intersections between the START
%   and each point in FINISH
%
%   xiP: m-by-1 vector of the x-coordinates of the starting point of
%   potential paths
%
%   yiP: m-by-1 vector of the y-coordinates of the starting point of
%   potential paths
%
%   xiQ: m-by-1 vector of the x-coordinates of the finishing points of
%   potential paths
%
%   yiQ: m-by-1 vector of the y-coordinates of the finishing points of
%   potential paths
%
%   xjP: 1-by-n vector of the x-coordinates of the starting point of the
%   polytope edge
%
%   yjP: 1-by-n vector of the y-coordinates of the starting point of the
%   polytope edge
%
%   xjQ: 1-by-n vector of the x-coordinates of the finishing point of the
%   polytope edge
%
%   yjQ:  1-by-n vector of the y-coordinates of the finishing point of the
%   polytope edge
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_Visibility_clearAndBlockedPoints
% for a full test suite.
%
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method in function usage examples
% 2025_07_17 - K. Hayes
% -- copied to new function from fcn_visibility _clear_and_blocked_points
%    to follow library convention
% 2025_07_25 - K. Hayes
% -- updated function formatting and header info
% -- added debug and input checking capabilities
%
% TO DO:
% (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 5; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS");
    MATLABFLAG_MAPGEN_FLAG_DO_DEBUG = getenv("MATLABFLAG_MAPGEN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
end

%% check input arguments?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if 0==flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(3,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

    end
end


% Does user want to specify the isConcave input?
isConcave = 0; % Default is 0
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        isConcave = temp;
    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1;
    end
end


%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%See: http://patorjk.com/software/taag/#p=display&f=Big&t=Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§
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
[D,di,dj] = fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ,start,finish,isConcave);

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

%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

end



%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

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
%      polytopes.distances = sum((polytopes.vertices(1:end-1,:) - polytopes.vertices(2:end,:)).^2,2).^0.5;
%      [Cx,Cy,polytope.area] = fcn_MapGen_polytopeCentroidAndArea ([[xv xv(1)]',[yv yv(1)]']);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(sum((polytopes.vertices(1:end-1,:) - ones(length(xv),1)*polytopes.mean).^2,2).^0.5);
%      point_tot = length(xv);
%      start = [0 0 point_tot+1 -1 0];
%      finish = [8 8 point_tot+2 0 0];
%      [xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_convert_to_vector_points(polytopes,start,finish);
%      fcn_BoundedAStar_plotPolytopes(polytopes,[],'b-',1,[0 8 0 8],'square')
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
%      polytopes.distances = sum((polytopes.vertices(1:end-1,:) - polytopes.vertices(2:end,:)).^2,2).^0.5;
%      [Cx,Cy,polytope.area] = fcn_MapGen_polytopeCentroidAndArea ([[xv xv(1)]',[yv yv(1)]']);
%      polytopes.mean = [Cx, Cy];
%      polytopes.max_radius = max(sum((polytopes.vertices(1:end-1,:) - ones(length(xv),1)*polytopes.mean).^2,2).^0.5);
%      point_tot = length(xv);
%      start = [0 0 point_tot+1 -1 0];
%      finish = [8 8 point_tot+2 0 0];
%      [xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_convert_to_vector_points(polytopes,start,finish);
%      [D,di,dj]=fcn_calculate_intersection_matrices(xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ);
%      num_int = D;
%      fcn_BoundedAStar_plotPolytopes(polytopes,[],'b-',2,[],'square')
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
% visible through free space then "fcn_Visibility_selfBlockedPoints" is better
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

