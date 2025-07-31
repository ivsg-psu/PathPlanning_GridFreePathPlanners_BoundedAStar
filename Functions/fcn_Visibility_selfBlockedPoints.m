function [cur_obs_id, self_blocked_cost, pts_blocked_by_self] = ...
    fcn_Visibility_selfBlockedPoints(polytopes,cur_pt,all_pts,varargin)
% fcn_Visibility_selfBlockedPoints
%
% determines the points blocked by the obstacle that the planner is currently
% at a vertex of
%
% FORMAT:
%
% [cur_obs_id, self_blocked_cost, pts_blocked_by_self] = ...
% fcn_Visibility_selfBlockedPoints(polytopes,cur_pt,all_pts,(fig_num))
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
%   cur_pt: the 1x5 array representing the current point, expected to be a vertex of a polytope
% 
%   all_pts: p-by-5 matrix of all the points except start and finish
%
%   (optional inputs)
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose
%
% OUTPUTS:
%
%   cur_obs_id: obstacle ID of the polytope that cur_pt is a vertex of
%
%   self_blocked_cost: polytope traversal scaling of the polytope the cur_pt is a vertex of
% 
%   pts_blocked_by_self: the other vertices on the polytope that cur_pt is a vertex of
%       that cannot be seen from cur_pt (i.e. neither neighboring vertex which would be visible
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

% TO DO
% -- implement contingency for concave polytopes

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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

        % Check the cur_pt input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            cur_pt, '5column_of_numbers');

        % Check the all_pts input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '5column_of_numbers');

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

cur_obs_id = cur_pt(4);
if cur_obs_id == -1
    % this isn't on any polytope
    pts_blocked_by_self = [];
    self_blocked_cost = 0;
    return
end
% find points with current point's obstacle id
[row, col] = find(all_pts(:,4)==cur_obs_id);
pts_on_cur_poly = all_pts(row(1):row(end),:);

% find polytope that current point is on
cur_poly = polytopes(cur_obs_id);

% check that the cur_pt is indeed a vertex of cur_poly
assert(ismember(cur_pt(1),cur_poly.xv))
assert(ismember(cur_pt(2),cur_poly.yv))


% find cur_pt's position in vertices
[~,cur_pt_indx]=ismember(cur_pt,pts_on_cur_poly,'rows');

% initialize array of self blocked pts
pts_blocked_by_cur_poly = pts_on_cur_poly;

% remove cur_pt
pts_blocked_by_cur_poly(cur_pt_indx,:) = NaN(1,5);

% remove neighbor before and neighbor after
if cur_pt_indx-1 == 0
    % remove the end if cur_pt is the first point
    pts_blocked_by_cur_poly(end,:) = NaN(1,5);
else
    % or just remove whatever is before cur_pt
    pts_blocked_by_cur_poly(cur_pt_indx-1,:) = NaN(1,5);
end
if cur_pt_indx  == size(pts_blocked_by_cur_poly,1)
    % remove the beginning if cur_pt is the last point
    pts_blocked_by_cur_poly(1,:) = [];
else
    % or just remove whatever is after cur_pt
    pts_blocked_by_cur_poly(cur_pt_indx+1,:) = NaN(1,5);
end
pts_blocked_by_self = rmmissing(pts_blocked_by_cur_poly);
self_blocked_cost = cur_poly.cost;

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
