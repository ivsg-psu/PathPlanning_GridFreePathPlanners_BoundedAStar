function cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, varargin)
% fcn_BoundedAStar_plotWindField
% plots a previously generated wind field with [u v] velocity at [x y]
% coordinates
%
% FORMAT:
% cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(...
%     windFieldU, windFieldV, windFieldX, windFieldY, (plotType), (figNum))
%
% INPUTS:
%
%     windFieldU: a nxn matrix containing the u-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     windFieldV: a nxn matrix containing the v-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     windFieldX: a 1xn vector containing the x values assigned to each
%     grid point within the specfied XY_range
%
%     windFieldY: a 1xn vector containing the y values assigned to each
%     grid point within the specified XY_range
%
%     (optional inputs)
%
%     plotType: a string indicating what type of wind field plot to create.
%      -- 'default' creates the default plot type, which is a filled contour
%         plot representing wind speed magnitude with white streamlines
%         indicating direction.
%      -- 'layer' creates a colored streamline layer, compressing the two
%         aspects of the default plot
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     cellArrayOfPlotHandles: a cell array of plot handle to the plotting
%     outputs.
%      -- 'default'
%         cellArrayOfPlotHandles{1} is the contour plot
%         cellArrayOfPlotHandles{2} is the color bar
%         cellArrayOfPlotHandles{3} is the streamslice
%      -- 'layer' 
%         cellArrayOfPlotHandles{1} is empty
%         cellArrayOfPlotHandles{2} is the color bar
%         cellArrayOfPlotHandles{3} is the Streamcolor
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_plotWindField
% for a full test suite.
%
% This function was written on 2025_07_21 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_07_21 by K. Hayes
% -- first write of function using fcn_BoundedAStar_fillWindField as a
%    starter
% 2025_07_22 by K. Hayes
% -- added 'default' and 'layer' settings
% 2025_07_31 by S. Brennan
% - Added h_plot output to provide access to figure handle
% - Updated function format to match standard form
% - Updated variable names for x and y to be more clear
%   % * Changed to windFieldX, windFieldY
% - Updated variable names for fig_num to remove underscore style
%   % * Changed to figNum

% TO-DO
% -- input checks?


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 6; % The largest Number of argument inputs to the function
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
        narginchk(0,MAX_NARGIN);
        
        % %%%%% No required inputs, delete these?
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
flag_do_plots = 1; % Default is to show plots, as this is a plotting fcn
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        figure(figNum);
        flag_do_plots = 1;
    else
        figNum = 1;
    end
end

% Does user want to change plotting mode?
plotType = 'default'; 
if 1 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        plotType = temp;
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

% Preliminary calculations for plotting
magField = nan*ones(size(windFieldU));

% Get magnitude for plot
for i = 1:size(windFieldU,1)
    for j = 1:size(windFieldU, 2)
        magField(i,j) = ((windFieldU(i,j)).^2 + (windFieldV(i,j)).^2).^0.5;
    end
end


% Initialize the outputs
cellArrayOfPlotHandles = cell(3,1);


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

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end

    switch plotType
        case 'default'
            % Plot magnitude layer
            colormap turbo
            [~, cellArrayOfPlotHandles{1}] = contourf(windFieldX, windFieldY, magField, 20, 'EdgeColor', 'none','HandleVisibility','off');
            cellArrayOfPlotHandles{2} = colorbar;
            cellArrayOfPlotHandles{2}.Label.String = 'Wind Speed (knots)';

            % Plot streamlines
            h_streamslice = streamslice(windFieldY,windFieldX,windFieldU,windFieldV);
            set(h_streamslice,'Color','white', 'LineWidth', 0.5,'HandleVisibility','off')
            cellArrayOfPlotHandles{3} = h_streamslice;
            
        case 'layer'
            cellArrayOfPlotHandles{1} = [];

            % Create x, y grids for Streamcolor
            [X, Y] = meshgrid(windFieldX, windFieldY);
            sx = linspace(min(windFieldX),max(windFieldX),15);
            sy = linspace(min(windFieldY),max(windFieldY),15);
            [SX, SY] = meshgrid(sx, sy);

            % Set up color formatting
            colormap turbo
            cellArrayOfPlotHandles{2} = colorbar;
            cellArrayOfPlotHandles{2}.Label.String = 'Wind Speed (knots)';

            % Call streamcolor
            cellArrayOfPlotHandles{3} = Streamcolor(X, Y, windFieldU', windFieldV', SX, SY, magField);

    end

    % Shut the hold off?
    if flag_shut_hold_off
        hold off;
    end

end % Ends the flag_do_plot if statement

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends the main function

