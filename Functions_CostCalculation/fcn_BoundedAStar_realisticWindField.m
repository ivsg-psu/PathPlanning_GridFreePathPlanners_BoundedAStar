function [windFieldU, windFieldV, windFieldX, windFieldY] = fcn_BoundedAStar_realisticWindField(varargin)
% fcn_BoundedAStar_realisticWindField
% fills in a windField grid with random [U V] components (updated from
% fillWindField)
%
% FORMAT:
% [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_BoundedAStar_realisticWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (fig_num))
%
% INPUTS:
%
%     (optional inputs)
%
%     XY_range: a 1x4 vector in the "axis" format of [Xmin Ymin Xmax Ymax]
%     that defines the extent of the grid in X and Y. 
%
%     NpointsInSide: The number of partitions on each side of the grid. For
%     example, NpointsInSide=10 produces a 10x10 grid
%
%     windMagnitude: the largest value of wind to use, in knots
%  
%     randomSeed: the random number generation seed to use
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     normalizedEastWind: a nxn matrix containing the u-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     normalizedNorthWind: a nxn matrix containing the v-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     windFieldX: a 1xn vector containing the x values assigned to each grid point
%     within the specfied XY_range
%
%     windFieldY: a 1xn vector containing the y values assigned to each grid point
%     within the specified XY_range
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%     fcn_GridMapGen_generateRandomOccupancyMap
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_realisticWindField
% for a full test suite.
%
% This function was written on 2026_02_03 by Kaelea Hayes
% Questions or comments? contact kaeleahayes@psu.edu

% REVISION HISTORY:
% 2026_02_03 by Kaelea Hayes
% -- first write of function using fcn_BoundedAStar_fillWindField as a
%    starter

% TO-DO
% -- add hardcoded occupancy grid parameters as inputs

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

% Does user want to specify the XY_range input?
XY_range = [-10 -10 10 10]; % Default is 10 (units) away from origin in each direction
if 1 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        XY_range = temp;
    end
end

% Does user want to specify the NpointsInSide input?
NpointsInSide = 200; % Default is 200 on each side
if 2 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        NpointsInSide = temp;
    end
end

% Does user want to specify the NpointsInSide input?
windMagnitude = 1; % Default is nondimensional 1
if 3 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        windMagnitude = temp;
    end
end

% Does user want to specify the randomSeed input?
randomSeed = 4; % Default is 4
if 4 <= nargin
    temp = varargin{4};
    if ~isempty(temp)
        randomSeed = temp;
    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        figure(figNum);
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

% Set random seed
rng(randomSeed);

% Set map size
mapSize = [NpointsInSide NpointsInSide];
nRows = mapSize(1);
mColumns = mapSize(2);

% Set other hardcoded parameters (to be updated to function inputs at a
% later date, but hardcoded for now)
occupancyRatio = 0.2;
dilationLevel = 6400;
seedMap = rand(nRows,mColumns);
leftDilationMultiplier = [];
rightDilationMultiplier = [];
optimizedThreshold = [];

% Call function to generate occupancy map (basis for wind field)
[~, randomMatrixDilated, ~, ~, ~] = ...
    fcn_GridMapGen_generateRandomOccupancyMap(...
    'mapSize', (mapSize),... % [nRows mCols])
    'occupancyRatio',(occupancyRatio),... % [1x1] value between 0 and 1
    'dilationLevel',(dilationLevel),.... % [1x1] strictly positive int
    'seedMap', (seedMap),... % [1x1] integer to be a random seed or NxM matrix of random numbers
    'leftDilationMultiplier', (leftDilationMultiplier),... %  [nRows nRows], ...
    'rightDilationMultiplier', (rightDilationMultiplier),... % [mCols mCols], ...
    'thresholdForced', (optimizedThreshold), ... % [1x1] scalar
    'flagSkipThresholdOptimization',(0),...% [1x1] scalar
    'figNum',(-1));

% Use the gradient to estimate wind direction
[px,py] = gradient(randomMatrixDilated);
eastWind  = py;
northWind = -px;

% Solve for the wind magnitude
wMag = (eastWind.^2+northWind.^2).^0.5;
maxWind = max(wMag,[],'all');
normalizedEastWind = eastWind./maxWind;
normalizedNorthWind = northWind./maxWind;

windFieldX = linspace(XY_range(1), XY_range(3), nRows);
windFieldY = linspace(XY_range(2), XY_range(4), mColumns);

windFieldU = normalizedEastWind*windMagnitude;
windFieldV = normalizedNorthWind*windMagnitude;

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

    figure(figNum);
    clf;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end   

    % Turn on legend
    legend('Interpreter','none','Location','best');

    % Plot wind field
    cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU, windFieldV,windFieldX,windFieldY,'default',figNum);
    set(cellArrayOfPlotHandles{3},'Color',[0.6 0.6 0.6]);

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

%% fcn_INTERNAL_plotCircle
