function [visibility_matrix_new, all_pts_new, start_new, finish_new, polytopes_after] = ...
fcn_Visibility_removeObstacle(...
visibility_matrix, all_pts, start, finish, polytopes_before, idx_of_polytope_for_removal, varargin)
% fcn_Visibility_removeObstacle
%   this function recalculates the visibility graph after deleting a polytope without recalculating
%   the entire visibility graph.  This is accomplished using an AABB check as a coarse check.
%   This function also recalculates the all_pts, start, finish, and polytopes data structures
%   as these are also affected by the removal of an obstacle.
% see vgraph_modification section of Documentation/bounded_astar_documentation.pptx for pseudocode and algorithm description
%
%
% FORMAT:
% [visibility_matrix_new, all_pts_new, start_new, finish_new, polytopes_after] = ...
%     fcn_Visibility_removeObstacle(...
%     visibility_matrix, all_pts, start, finish, polytopes_before, idx_of_polytope_for_removal, (fig_num))
%
% INPUTS:
%
%     visibility_matrix: nxn matrix, where n is the number of points in all_pts
%       a 1 in column i and row j indicates that all_pts(i,:) is visible from
%       all_pts(j,:).  This matrix is therefore symmetric
%     
%     all_pts: p-by-5 matrix of all the possible start points
%       the information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id number
%         obstacle id number
%         beginning/ending indication (1 if the point is a beginning or ending
%         point and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%     
%     start: 1-by-5 vector with the same info as route for the starting point
%     
%     finish: same as start for the finish point
%
%     polytopes_before: the polytope struct array prior to modification INCLUDING the polytope for removal
%     
%     idx_of_polytope_for_removal: the index of the polytope to be removed in the polytopes_before struct array
%
%     (optional inputs)
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%         input checking or debugging, no figures will be generated, and sets
%         up code to maximize speed. As well, if given, this forces the
%         variable types to be displayed as output and as well makes the input
%         check process verbose
%
%
% OUTPUTS:
%     visibility_matrix_new: same as the visiblity_matrix input but modified so that the removed
%         removed polytope no longer affects the visibility.  Note this may have fewer points than
%         the input matrix as points on the removed polytope are deleted.
%
%     all_pts_new: same as the all_pts input but with points on the removed polytope deleted.
%         May be reindexed
%
%     start_new: same as start input but reindexed to account for removed points
%     
%     finish_new: same as finish input but reindexed to account for removed points
%     
%     polytopes_after:  the polytope struct array after modification no longer including
%         the polytope for removal
%
% DEPENDENCIES:
%     fcn_MapGen_isCrossingAABB from the MapGen repo
%     fcn_Visibility_clearAndBlockedPoints
%
% EXAMPLES:
%
% See the script: script_visibility_graph_modification
% for a full test suite.
%
% Questions or comments? contact sjh6473@psu.edu

% REVISION HISTORY:
% 2024_03
% -- first written by Steve Harnett
% Questions? sjh6473@psu.edu
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_visibility_graph_remove_obstacle
%    to follow library convention
% 2025_07_31 - K. Hayes
% -- updated function formatting and header
% -- added input and debug checks

% TO DO:
% (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 7; % The largest Number of argument inputs to the function
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
        narginchk(6,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

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


%% initialize the modified vgraph, pts, start, and finish to the old values
visibility_matrix_new = visibility_matrix;
all_pts_new = all_pts;
start_new = start;
finish_new = finish;
all_pts_with_start_and_fin = [all_pts; start; finish];

%% remove polytope from vgraph and all_pts
polytope_of_interest = polytopes_before(idx_of_polytope_for_removal); % get polytope for removal
% get AABB of polytope for removal
polytope_vertices = [[polytope_of_interest.xv]',[polytope_of_interest.yv]'];
AABB = [min(polytope_of_interest.xv) min(polytope_of_interest.yv) max(polytope_of_interest.xv) max(polytope_of_interest.yv)];
% delete polytope
polytopes_after = polytopes_before;
polytopes_after(idx_of_polytope_for_removal) = [];
% find the verts of that polytope in all_pts table
[~, xloc] = ismember(polytope_vertices(:,1), all_pts(:,1));
[~, yloc] = ismember(polytope_vertices(:,2), all_pts(:,2));
idx_of_points_on_polytope = union(xloc,yloc);
% vgraph edges that start or end on this obstacle should be removed
visibility_matrix_new(idx_of_points_on_polytope,:) = [];
visibility_matrix_new(:,idx_of_points_on_polytope) = [];
% remove points on that polytope from all_pts table
all_pts_new(idx_of_points_on_polytope,:) = [];
% size of vgraph has now changed so points need to be re-indexed
% reindex all_pts, start, and finish
num_pts_after_removal = size(all_pts_new,1);
all_pts_new(:,3) = [1:num_pts_after_removal]';
% if the user gave a start or finish, reindex it
if ~isempty(start_new)
    start_new(3) = num_pts_after_removal+1;
end
if ~isempty(finish_new)
    finish_new(3) = num_pts_after_removal+2;
    assert(finish_new(3) == size(visibility_matrix_new,1))
end
% find which new points cross the AABB
isInside = fcn_MapGen_isCrossingAABB(AABB, [all_pts_new; start; finish]);
% only want possible edges that are not already edges as deleting the obstacle adds edges, it does
% not remove existing edges
[r,c] = find(isInside & ~visibility_matrix_new);
%% check only specific edges method
for i = 1:length(r)
    [~,~,D] = fcn_Visibility_clearAndBlockedPoints(polytopes_after, all_pts_new(r(i),:), all_pts_new(c(i),:));
    visibility_scalar = sum(D);
    assert(isequal(size(visibility_scalar),[1 1]))
    if ~visibility_scalar
        visibility_matrix_new(r(i),c(i)) = 1;
    end
end
sprintf('num checks was %i',length(r))
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

