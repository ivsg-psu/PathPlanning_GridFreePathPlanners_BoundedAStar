function [all_pts, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start_xy, finish_xy, varargin)

warning('on','backtrace');
warning(['fcn_BoundedAStar_polytopesGenerateAllPtsTable is being deprecated. ' ...
    'Use fcn_Visibility_polytopesGenerateAllPtsTable within the Visibility Graph library instead.']);


% fcn_BoundedAStar_polytopesGenerateAllPtsTable
%
% A short function to turn polytope vertices into an nx5 table of points of
% the form used by:
%   % -fcn_BoundedAStar_AstarBounded 
%   % -fcn_Visibility_clearAndBlockedPointsGlobal
%   % -fcn_Visibility_clearAndBlockedPoints
%
%
% FORMAT:
% [all_pts, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start_xy, finish_xy, (fig_num))
%
%
% INPUTS:
%
%     start_xy: the start point vector (x,y)
%
%     finish_xy: the finish point vector (x,y)
%
%     polytopes: the polytope struct array
%
%     (optional inputs)
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
%
% OUTPUTS:
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
%     start: the start point vector as a 1x5 array with the same information as all_pts
%
%     finish: the finish point vector  as a 1x5 array with the same information as all_pts
%
%
% DEPENDENCIES:
%
% none but MapGen library may be useful for creating polytopes
%
% EXAMPLES:
%
% See the script: script_demo_fcn_BoundedAStar_Astar and
% script_test_fcn_BoundedAStar_polytopesGenerateAllPtsTable
% for demonstration of this function in use.
%
% This function was written on 8 May 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
% As: fcn_BoundedAStar_polytopesGenerateAllPtsTable
% 2024_05_08, by Steve Harnett
% -- first write of function
% 2025_07_07 S. Brennan and K. Hayes
% -- changed demo script 
%    from: script_test_fcn_algorithm_Astar
%    to:   script_demo_fcn_BoundedAStar_Astar
% 2025_07_25 - K. Hayes
% -- copied to new function from fcn_polytopes_generate_all_pts_table to
%    follow library convention
% 2025_08_05 - K. Hayes
% -- updated function formatting and header
%
% As: fcn_Visibility_polytopesGenerateAllPtsTable
% 2025_11_01 - S. Brennan
% -- renamed function to fcn_Visibility_polytopesGenerateAllPtsTable

%
% TO DO:
% -- fill in to-do items here.

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

        % Check the polytopes input, make sure it is a structure
        assert(isstruct(polytopes));

        % Check the start_xy input, make sure it has 2 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start_xy, '2column_of_numbers');

        % Check the finish_xy input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish_xy, '2column_of_numbers');

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
    point_tot = length([polytopes.xv]); % total number of vertices in the polytopes
    beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    for poly = 1:size(polytopes,2) % check each polytope
        verts = length(polytopes(poly).xv);
        polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
        beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
        curpt = curpt+verts;
    end
    obs_id = [polytopes.obs_id];
    all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
    start = [start_xy size(all_pts,1)+1 -1 1];
    finish = [finish_xy size(all_pts,1)+2 -1 1];

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
    % Plot all points
    figure(fig_num)
    plot(all_pts(:,1),all_pts(:,2),'b.','MarkerSize',10)
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