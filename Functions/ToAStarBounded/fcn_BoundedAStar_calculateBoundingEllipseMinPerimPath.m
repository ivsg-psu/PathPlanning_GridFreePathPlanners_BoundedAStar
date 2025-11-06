function [max_dist] = fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath(int_polytopes,intersections,start,finish,varargin)
% fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath 
% 
% adds up all the distances
% between intersections along the start and finish and adds in the shortest
% perimeter distance around each intersected obstacle
%
% FORMAT:
%
% [max_dist] = fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath(int_polytopes,intersections,start,finish,(fig_num))
%
% INPUTS:
% 
%   int_polytopes: a 1-by-p seven field structure of intersected polytopes,
%   where p = number of intersected polytopes, with fields:
%
%   intersections: a 1-by-f three field structure of intersection information, where
%   f = number of finish points, with fields:
%       points: i-by-2 matrix of xy coordinates, where i=number of intersections
%       index: 1-by-i vector of intersecting line indices
%       obstacles: 1-by-i vector of obstacles intersecting the line
%
%   start:  a 1-by-5 vector of starting point information, including:
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number
%       beginning/ending indication (1 if the point is a beginning or ending
%           point and 0 otherwise)
%       Ex: [x y point_id obs_id beg_end]
%
%   finish: a 1-by-5 vector of ending point information, including the same
%   information as START
%
%   (optional inputs)
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
% OUTPUTS: 
%   max_dist: sum of all the distances between intersections and along
%   minimum perimeters
%
% DEPENDENCIES:
%   
%   fcn_DebugTools_checkInputsToFunctions
%
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method
% -- copied to new function from fcn_bounding_ellipse_min_perimeter_path.m
%    to follow library convention
% 2025_07_25 - K. Hayes
% -- fixed function format 
% -- added input checking and debugging
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
        narginchk(4,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

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

try % try block the entire function so if it fails for being inside a polytope, we can just assume
    % a large boundary

% TODO @sjharnett replace bug planner with estimated R_lc function to define boundary in
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
dist = round(sum((ones(size(points,1),1)*startpt - points).^2,2).^0.5,10);
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
    [perimeter1,perimeter2] = fcn_BoundedAStar_polytopeCalculateDualPerimeters(polytope,xing1,xing2);
    % add the smallest perimeter to the max_dist
    if perimeter1 < perimeter2
        max_dist = max_dist + perimeter1;
    else
        max_dist = max_dist + perimeter2;
    end
    startpt = xing2; % make xing2 the next starting point
end
% add the distance from the last intersection to the end point
max_dist = max_dist + sum((startpt - finishpt).^2,2).^0.5;
catch % just assume a maximum distance of double the distance between start and goal (equivalent
      % to routing around the obstacle field) if finding the boundary fails because we are starting
      % inside a polytope
    max_dist = 2*sum((startpt - finishpt).^2,2).^0.5;
end
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