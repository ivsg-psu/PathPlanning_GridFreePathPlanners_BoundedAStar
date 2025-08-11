function trajectory = fcn_BoundedAStar_calcControlInput(startPoint, timeLength, x, y, windFieldU, windFieldV, varargin)
% fcn_BoundedAStar_sampleWindField
% samples a wind field near a given point and outputs the resulting wind
% vector
%
% FORMAT:
% [windVector] = fcn_BoundedAStar_sampleWindField(samplePoint, x, y, windFieldU, windFieldV, (fig_num))
%
% INPUTS:
%
%     startPoint: the [x, y] point to drop a particle into the wind field 
%
%     timeLength: the length of time to propagate the simulation for
%
%     x: a 1xn vector containing the x values assigned to each wind field
%     grid point 
%
%     y: a 1xn vector containing the y values assigned to each wind field
%     grid point
%
%     windFieldU: a nxn matrix containing the u-direction components of the
%     wind velocity at each grid point
%
%     windFieldV: a nxn matrix containing the v-direction components of the
%     wind velocity at each grid point
%
%     (optional inputs)
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     trajectory: the stored trajectory of the point over time
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_streamlineFollowing
% for a full test suite.
%
% This function was written on 2025_07_30 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_07_30 by K. Hayes
% -- first write of function using fcn_BoundedAStar_sampleWindField as a
%    starter

% TO-DO
% -- (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN =7; % The largest Number of argument inputs to the function
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
        narginchk(0,MAX_NARGIN);
        
        %%%%% No required inputs, delete these?
        % % Check the XY_range input, make sure it is '4column_of_numbers'
        % % type with exactly 1 row
        % fcn_DebugTools_checkInputsToFunctions(...
        %     XY_range, '4column_of_numbers',[1 1]);
        % 
        % % Check the radius input, make sure it is '1column_of_numbers'
        % % type, 1 row
        % fcn_DebugTools_checkInputsToFunctions(...
        %     radius, '1column_of_numbers',[1 1]);
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

% Create state matrices
deltat = 0.1;
numSteps = timeLength/deltat;
trajectory = nan*ones(numSteps,2);
trajectory(1,:) = startPoint;
currentPoint = startPoint;

for k = 1:numSteps
    % Find wind vector at point and calculate disturbance
    windVector = fcn_BoundedAStar_sampleWindField(currentPoint, x, y, windFieldU, windFieldV, 0, 1, -1);
    disturbance = (windVector-currentPoint)/numSteps;
    
    % Add disturbance to postion
    trajectory(k+1,:) = currentPoint + disturbance;
    
    % Update position
    currentPoint = trajectory(k+1,:);
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
    % Plot setup
    figure(fig_num)
    hold on
    grid on
    axis equal

    % Plot wind field
    fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, x, y, 'default',(fig_num))
    
    % Plot trajectory
    plot(trajectory(:,1), trajectory(:,2), 'k.', 'MarkerSize', 20, 'DisplayName', 'Trajectory')
    
    % Display legend
    legend

end % Ends the flag_do_plot if statement

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end


end % Ends the main function



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

