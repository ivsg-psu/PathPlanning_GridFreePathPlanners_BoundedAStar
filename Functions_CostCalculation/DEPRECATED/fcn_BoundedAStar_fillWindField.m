function [windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField(varargin)
% fcn_BoundedAStar_fillWindField
% fills in a windField grid with random [U V] components
%
% FORMAT:
% [windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (fig_num))
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
%     peaksMode: a logical input indicating whether to randomly generate a
%     wind field (== 0) or to use the standard 'peaks' function as a base
%     (==1)
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
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
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_fillWindField
% for a full test suite.
%
% This function was written on 2025_07_11 by Sean Brennan
% Questions or comments? contact sbrennan@psu.edu

% REVISION HISTORY:
% 2025_07_11 by Sean Brennan
% -- first write of function using fcn_MapGen_generatePolysFromTiling in
%    MapGen library as a starter
% 2025_07_14 by K. Hayes, kxh1031@psu.edu
% -- added gaussian surface option to field generation
% -- changed default wind magnitude to 10 knots
% -- cleaned function formatting and description
% -- fixed bug where function would only generate 20x20 plots with -10:10
%    range
% 2025_07_15 by K. Hayes
% -- fixed plotting bug for coarse grids

% TO-DO
% -- current renormalization method results in very homogenous random maps
%    for small NpointsInSide

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
windMagnitude = 10; % Default is 10 knots
if 3 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        windMagnitude = temp;
    end
end

% Does user want to specify the randomSeed input?
randomSeed = 1; % Default is 1
if 4 <= nargin
    temp = varargin{4};
    if ~isempty(temp)
        randomSeed = temp;
    end
end

% Does user want to specify peaksMode input?
peaksMode = 0; % Default is off/0
if 5 <= nargin
    temp = varargin{5};
    if ~isempty(temp)
        peaksMode = temp;
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



% Set random seed
rng(randomSeed);

% Check for peaksMode flag
if peaksMode == 1
    % Generate wind map using peaks()
    
    % Set up smaller grid to crop to 2x2 center of peaks()
    design_min_x = -2;
    design_max_x = 2;
    design_min_y = -2;
    design_max_y = 2;

    % Set up initial x and y vectors, mesh with correct number of grid elements
    xraw = linspace(design_min_x,design_max_x,NpointsInSide);
    yraw = linspace(design_min_y,design_max_y,NpointsInSide);
    [Xraw,Yraw] = meshgrid(xraw,yraw);

    % Assign u,v values to discretized wind field
    windFieldU = peaks(Xraw,Yraw);
    windFieldV = windFieldU';
    
    % Rescale smaller X and Y to fit within the specified XY range
    X = (Xraw - design_min_x)*(XY_range(3)-XY_range(1))/(design_max_x - design_min_x) + XY_range(1);
    Y = (Yraw - design_min_y)*(XY_range(4)-XY_range(2))/(design_max_y - design_min_y) + XY_range(2);
    x = (xraw - design_min_x)*(XY_range(3)-XY_range(1))/(design_max_x - design_min_x) + XY_range(1);
    y = (yraw - design_min_x)*(XY_range(3)-XY_range(1))/(design_max_x - design_min_x) + XY_range(1);

else
    % Randomly generate a wind field using a Gaussian smoothing filter

    % Get x, y vectors for wind field sizing
    x = linspace(XY_range(1), XY_range(3), NpointsInSide);
    y = linspace(XY_range(2), XY_range(4), NpointsInSide);

    % Choose smoothing factor based on number of points in side
    sg = NpointsInSide/10;

    % Generate a random mesh to represent u direction
    [X, Y] = meshgrid(x, y); % Create a 2D grid of x and y coordinates
    ufieldRaw = imgaussfilt(randn(NpointsInSide), sg,'Padding','circular'); % Generate random heights and smooth them
    
    % renormalize z
    ufieldRaw_max = abs(max(ufieldRaw,[],'all'));
    ufieldRaw_min = abs(min(ufieldRaw,[],'all'));
    
    maxRange = max(ufieldRaw_max, ufieldRaw_min);
    
    % Make the wind have the right magnitude
    windFieldU = ufieldRaw*(windMagnitude/maxRange);
    
    % Do the same for the v direction
    % Generate a random mesh to represent v direction
    vfieldRaw = imgaussfilt(randn(NpointsInSide), sg,'Padding','circular'); % Generate random heights and smooth them
    
    % renormalize z
    vfieldRaw_max = max(vfieldRaw,[],'all');
    vfieldRaw_min = abs(min(vfieldRaw,[],'all'));
    
    maxRange = max(vfieldRaw_max, vfieldRaw_min);
    
    % Make the wind have the right magnitude
    windFieldV = vfieldRaw*(windMagnitude/maxRange);
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

    figure(fig_num);
    clf;
    subplot(1,3,1);
    mesh(X,Y,windFieldU);
    title('X-direction magnitude');

    subplot(1,3,2);
    mesh(X,Y,windFieldV);
    title('Y-direction magnitude');

    subplot(1,3,3);
    % Plot the wind field
    if numel(windFieldU) > 1000
        NpointsInSide = length(windFieldU(:,1));
        indices = (1:NpointsInSide); % Row vector
        Xindices = repmat(indices,NpointsInSide,1);
        Yindices = repmat(indices',1,NpointsInSide);
    
        moduloX = mod(Xindices,10); % Keep only 1 of every 10
        moduloY = mod(Yindices,10); % Keep only 1 of every 10
        
        moduloXreshaped = reshape(moduloX,[],1);
        moduloYreshaped = reshape(moduloY,[],1);
    
        indicesX = find(moduloXreshaped==1);
        indicesY = find(moduloYreshaped==1);
    
        [X,Y] = meshgrid(x,y);
    
        indicesToPlot = intersect(indicesX,indicesY);
        quiver(X(indicesToPlot),Y(indicesToPlot),windFieldU(indicesToPlot),windFieldV(indicesToPlot));
    else 
        quiver(x,y,windFieldU,windFieldV);
    end

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
function circle_points = fcn_INTERNAL_plotCircle(centers,radii)
% COPIED OUT OF: fcn_geometry_plotCircle in GeomClass library

% Use number of radii to calculate the number of centers
Ncircles = length(centers(:,1));

% Set angles for plotting
angles = (0:0.01:2*pi)';

% Loop through the arcs, prepping data for plotting each
if Ncircles>1
    circle_points{Ncircles} = [];
end
for ith_circle = 1:Ncircles 

    xdata = centers(ith_circle,1)+radii(ith_circle)*cos(angles);
    ydata = centers(ith_circle,2)+radii(ith_circle)*sin(angles);

    x_arc = xdata; % [x_arc; NaN; xdata]; %#ok<AGROW>
    y_arc = ydata; %[y_arc; NaN; ydata]; %#ok<AGROW>

    if Ncircles==1
        circle_points = [x_arc y_arc];
    else
        circle_points{ith_circle} = [x_arc y_arc];
    end
end
end % Ends fcn_INTERNAL_plotCircle