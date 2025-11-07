function [currentObstacleID, selfBlockedCost, pointsWithDataBlockedBySelf] = ...
    fcn_VGraph_selfBlockedPoints(polytopes, testPointData, pointsWithData, varargin)
% fcn_VGraph_selfBlockedPoints
%
% determines the points blocked by the obstacle that the planner is currently
% at a vertex of
%
% FORMAT:
%
% [currentObstacleID, selfBlockedCost, pointsWithDataBlockedBySelf] = ...
% fcn_VGraph_selfBlockedPoints(polytopes, testPointData, pointsWithData, (figNum))
%
% INPUTS:
%
%   polytopes: a 1-by-n seven field structure of shrunken polytopes,
%       where n <= number of polytopes with fields:
%           vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%           the number of the individual polytope vertices
%           xv: a 1-by-m vector of vertice x-coordinates
%           yv: a 1-by-m vector of vertice y-coordinates
%           distances: a 1-by-m vector of perimeter distances from one point to the
%           next point, distances(i) = distance from vertices(i) to vertices(i+1)
%           mean: centroid xy coordinate of the polytope
%           area: area of the polytope
%   
%   testPointData: the 1x5 array representing the current point, expected to be a vertex of a polytope
% 
%   pointsWithData: p-by-5 matrix of all the points except start and finish
%
%   (optional inputs)
%
%   figNum: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose
%
% OUTPUTS:
%
%   currentObstacleID: obstacle ID of the polytope that testPointData is a vertex of
%
%   selfBlockedCost: polytope traversal scaling of the polytope the testPointData is a vertex of
% 
%   pointsWithDataBlockedBySelf: the other vertices on the polytope that testPointData is a vertex of
%       that cannot be seen from testPointData (i.e. neither neighboring vertex which would be visible
%       by looking down the side of a convex polytope)
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% For additional examples, see implementation of this in the through planner in
% fcn_BoundedAStar_AstarBounded.m
%
% This function was written in 2022_05 by Steve Harentt
% Questions or comments? sjh6473@psu.edu
%

% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_visibility_self_blocked_pts to
%    follow library convention
% 2025_07_31 - K. Hayes
% -- reformatted function and updated header
% -- added input and debug capabilities
% 2025_08_05 - K. Hayes
% -- added plotting capabilities to fcn debug section
% 2025_11_03 - S. Brennan
% -- cleaned up variable naming:
%    % * all_pts to pointsWithData
%    % * cur_pt to testPointData
%    % * cur_obs_id to currentObstacleID
%    % * fig_num to figNum
%    % * self_blocked_cost to selfBlockedCost
%    % pts_blocked_by_self to pointsWithDataBlockedBySelf
% 2025_11_07 - S. Brennan, sbrennan@psu.edu
% -- Changed global flags from _MAPGEN_ to _VGRAPH_
%
% As: fcn_VGraph_selfBlockedPoints
% 2025_11_07 - S. Brennan
% -- Renamed fcn_Visibility_selfBlockedPoints to fcn_VGraph_selfBlockedPoints
% -- Cleared extra figure command out of Inputs section

% TO DO
% 2025_11_03 - S. Brennan
% -- clean up the before/after if/then logic. This can easily be replaced
% by using a mod operation on the query point +/- 1 with the length of the
% points.
% -- make sure first/last point in poly is not repeated
% -- implement contingency for concave polytopes. An easy way to do this is
% to find the polytope related to the test point, project outward slightly,
% then check visibility of the projected point to the other vertices in the
% polytope.


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 4; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS");
    MATLABFLAG_VGRAPH_FLAG_DO_DEBUG = getenv("MATLABFLAG_VGRAPH_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_VGRAPH_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_VGRAPH_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_figNum = 999978; %#ok<NASGU>
else
    debug_figNum = []; %#ok<NASGU>
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

        % Check the testPointData input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            testPointData, '5column_of_numbers');

        % Check the pointsWithData input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            pointsWithData, '5column_of_numbers');

    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
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

currentObstacleID = testPointData(4);
if currentObstacleID == -1
    % this isn't on any polytope
    pointsWithDataBlockedBySelf = [];
    selfBlockedCost = 0;
    return
end

% find points with current point's obstacle id
[row, ~] = find(pointsWithData(:,4)==currentObstacleID);
pts_on_cur_poly = pointsWithData(row(1):row(end),:);

% find polytope that current point is on
cur_poly = polytopes(currentObstacleID);

% check that the testPointData is indeed a vertex of cur_poly
assert(ismember(testPointData(1),cur_poly.xv))
assert(ismember(testPointData(2),cur_poly.yv))


% find testPointData's position in vertices
[~,testPointData_indx]=ismember(testPointData,pts_on_cur_poly,'rows');

% initialize array of self blocked pts
pts_blocked_by_cur_poly = pts_on_cur_poly;

% remove testPointData
pts_blocked_by_cur_poly(testPointData_indx,:) = NaN(1,5);

% remove neighbor before and neighbor after
if testPointData_indx-1 == 0
    % remove the end if testPointData is the first point
    pts_blocked_by_cur_poly(end,:) = NaN(1,5);
else
    % or just remove whatever is before testPointData
    pts_blocked_by_cur_poly(testPointData_indx-1,:) = NaN(1,5);
end
if testPointData_indx  == size(pts_blocked_by_cur_poly,1)
    % remove the beginning if testPointData is the last point
    pts_blocked_by_cur_poly(1,:) = [];
else
    % or just remove whatever is after testPointData
    pts_blocked_by_cur_poly(testPointData_indx+1,:) = NaN(1,5);
end
pointsWithDataBlockedBySelf = rmmissing(pts_blocked_by_cur_poly);
selfBlockedCost = cur_poly.cost;

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

if flag_do_plots
    figure(figNum)
    hold on
    
    % Plot current polytope
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;
    plotFormat.DisplayName = 'Current Polytope';
    fcn_MapGen_plotPolytopes(polytopes(currentObstacleID),plotFormat,[1 0 0 1 0.5],(figNum));
    
    % Plot current point
    plot(testPointData(1),testPointData(2),'gx','MarkerSize',15,'LineWidth',2,'DisplayName','Current point')

    % Plot blocked points
    plot(pointsWithDataBlockedBySelf(:,1),pointsWithDataBlockedBySelf(:,2),'rx','MarkerSize',15,'LineWidth',2,'DisplayName','Self-blocked points');

    legend
end


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
