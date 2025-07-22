function [] = fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, x, y, varargin)
% fcn_BoundedAStar_plotWindField
% plots a previously generated wind field with [u v] velocity at [x y]
% coordinates
%
% FORMAT:
% [] = fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, x, y, (plotType), (fig_num))
%
% INPUTS:
%
%     windFieldU: a nxn matrix containing the u-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     windFieldV: a nxn matrix containing the v-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     x: a 1xn vector containing the x values assigned to each grid point
%     within the specfied XY_range
%
%     y: a 1xn vector containing the y values assigned to each grid point
%     within the specified XY_range
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
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
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

% TO-DO
% -- input checks?


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1;
    else
        fig_num = 1;
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

figure(fig_num)


    magField = nan*ones(size(windFieldU));
    
% Get magnitude for plot 
for i = 1:size(windFieldU,1)
    for j = 1:size(windFieldU, 2)
        magField(i,j) = ((windFieldU(i,j)).^2 + (windFieldV(i,j)).^2).^0.5; 
    end
end

 switch plotType
    case 'default'      
        % Plot magnitude layer
        colormap turbo
        contourf(x, y, magField, 20, 'EdgeColor', 'none')
        cb = colorbar;
        
        hold on
        
        % Plot streamlines
        obj = streamslice(x,y,windFieldU,windFieldV);
        set(obj,'Color','white', 'LineWidth', 0.5)
        cb.Label.String = 'Wind Speed (knots)';
    case 'layer'
        % Create x, y grids for Streamcolor
        [X, Y] = meshgrid(x, y);
        sx = linspace(min(x),max(x),15);
        sy = linspace(min(y),max(y),15);
        [SX, SY] = meshgrid(sx, sy)
        
        % Set up color formatting
        colormap turbo
        cb = colorbar;
        cb.Label.String = 'Wind Speed (knots)'

        % Call streamcolor
        h = Streamcolor(X, Y, windFieldU, windFieldV, SX, SY, magField);
        
end
end % Ends the main function

