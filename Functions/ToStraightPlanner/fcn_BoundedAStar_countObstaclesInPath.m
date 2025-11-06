function [obs_around, obs_through] = ...
    fcn_BoundedAStar_countObstaclesInPath(path, varargin)

% fcn_BoundedAStar_countObstaclesInPath
% counts the number of unique obstacles encountered in a path
% tallying obstacle routed through separately from obstacles routed around
% for use wtih a planner that routes through or around obstacles, only at vertices
%
% FORMAT:
%
% [obs_around, obs_through] = ...
%     fcn_BoundedAStar_countObstaclesInPath(path, (polytopes), (fig_num))
%
% INPUTS:
%
% path: p-by-5 matrix of all the points in the path
%
% (optional inputs)
%
% fig_num: a figure number to plot results. If set to -1, skips any
% input checking or debugging, no figures will be generated, and sets
% up code to maximize speed. As well, if given, this forces the
% variable types to be displayed as output and as well makes the input
% check process verbose
%
%
% OUTPUTS:
%
% obs_around: the integer number obstacles that were routed around by the planner
% obs_through: the integer number of obstacles routed through by the planner
%
% DEPENDENCIES:
% 
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% For additional examples, see:
% script_planning_performed_at_multiple_costs.m 
% and script_test_fcn_BoundedAStar_countObstaclesInPath.m
%
% This function was written in 2022_05 by Steve Harentt
% Questions or comments? sjh6473@psu.edu
%
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from
%    fcn_general_calculation_count_obs_in_path to follow library
%    convention
% 2025_08_25 - K. Hayes
% -- updated fcn header and formatting

% TO DO

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 2; % The largest Number of argument inputs to the function
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
        narginchk(1,MAX_NARGIN);

        % Check the path input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            path, '5column_of_numbers');
        
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

    % extract obstacle IDs from points data
    obs_ids = path(:,4);
    % the start point must have obs_id of -1 (i.e. it's not on an obstacle)
    assert(obs_ids(1)==-1)
    % initialize counters
    obs_around = 0;
    obs_through = 0;
    obs_id_idx = 2;
    while obs_id_idx < length(obs_ids)
        % if the obstacle ID doesn't increment, we are on the same obstacle, i.e.
        % we went through from vertex to vertex
        if obs_ids(obs_id_idx) == obs_ids(obs_id_idx+1)
            obs_through = obs_through + 1;
            obs_id_idx = obs_id_idx + 2;
        % if the obstacle ID increments, we are on a new obstacle and therefore
        % went around the obstacle
        elseif obs_ids(obs_id_idx) ~= obs_ids(obs_id_idx+1)
            obs_around = obs_around + 1;
            obs_id_idx = obs_id_idx + 1;
        end
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

if flag_do_plots
    figure(fig_num)
    hold on
    
    for i = 1:obs_around
        
    end

end 


end % end function

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
