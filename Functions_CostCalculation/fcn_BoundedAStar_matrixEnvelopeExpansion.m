function [expandedSets] = fcn_BoundedAStar_matrixEnvelopeExpansion(radius, windFieldU, windFieldV, x, y, varargin)
% fcn_BoundedAStar_matrixEnvelopeExpansion
% calculates the resulting reachable radius from a point at the center of a
% circle, where the circle has a radius equal to the zero-wind distance.
% Uses a changing wind field and does line integrals from each starting
% direction.
%
% FORMAT:
% windRadius = fcn_BoundedAStar_matrixEnvelopeExpansion(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num))
%
% INPUTS:
%
%     radius: a 1x1 scalar representing the radius of travel without wind
%
%     windFieldU:  a matrix containing the u-direction components of the
%     wind velocity at each grid point
%
%     windFieldV:  a matrix containing the v-direction components of the
%     wind velocity at each grid point
%
%     x: a vector containing the x values assigned to each grid point
% 
%     y: a vector containing the y values assigned to each grid point  
%
%
%     (optional inputs)
%
%     startPoint: a 1x2 vector representing the [x,y] values of the start
%     point. Defaults to [0,0] if no value is entered
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     windRadius: a set of points defining the distance conversion of the
%     original travel locations to locations with wind
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_setExpansion
% for a full test suite.
%
% This function was written on 2025_07_23 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_07_23 by K. Hayes
% -- first write of function using fcn_BoundedAStar_calcCostChangingWind as
%    a starter
% 2025_07_24 by K. Hayes
% -- fixed bug with incorrect expansion set
% -- fixed function documentation
% -- added plotting capability
% 2025_07_29 by K. Hayes
% -- fixed bug in wind sampling

% TO-DO
% -- add flag_do_animation to turn animation off
% -- vectorize the inner for loop in MAIN

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
        narginchk(5,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        fcn_DebugTools_checkInputsToFunctions(...
            radius, '1column_of_numbers',[1 1]);
    end
end

% Does user want to specify startPoint input?
startPoint = [0 0]; % Default is origin
if 1 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        startPoint = temp;
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

% Generate circle points and assign them to x0 initial set
centers = startPoint;
circle_points = fcn_INTERNAL_plotCircle(centers,radius);
x0 = circle_points;
n = length(x0);

% Create state matrices
numSteps = 100;
stepLength = radius/numSteps;
angles = (0:0.01:2*pi);
A = eye(n);
% [X,Y] = meshgrid(x,y,size(windFieldU,1));
expandedSets = nan*ones(n,2,numSteps);
expandedSets(:,:,1) = startPoint.*ones(n,2,1);
indices = nan*ones(n,2);
heading = nan*ones(n,2);
for k = 1:numSteps
    for i = 1:n
        xIndex = find(x>expandedSets(i,1,k),1,'first');
        yIndex = find(y>expandedSets(i,2,k),1,'first');
        if isempty(xIndex)
            xIndex = find(x<expandedSets(i,1,k),1,'first');
        end
        if isempty(yIndex)
            yIndex = find(y<expandedSets(i,2,k),1,'first');
        end
        indices(i,:,k) = [xIndex yIndex];
        heading(i,:) = [cos(angles(i)) sin(angles(i))]*stepLength;
    end
    
    % Convert indices into linear indices for indexing wind fields
    linearInd = sub2ind(size(windFieldU),indices(:,2,k),indices(:,1,k));

    Wu = windFieldU(linearInd);
    Wv = windFieldV(linearInd);
    % windVector = fcn_BoundedAStar_sampleWindField(expandedSets(i,:,k),x,y,windFieldU,windFieldV,(-1));
    W = [Wu Wv]/numSteps + heading;

    % Debug options:
    % uncomment (and comment out W above) to make sure that not adding
    % disturbances results in a circle
    %W = heading;
    % uncomment (and comment out W above) to make sure that a point follows
    % the streamlines appropriately
    %W = [Wu Wv]/numSteps;
    
    expandedSets(:,:,k+1) = A*expandedSets(:,:,k) + W;
    linearInd = [];
    Wu = [];
    Wv = [];
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
    % Prep the figure for plotting
    temp_h = figure(fig_num);
    flag_rescale_axis = 0;
    if isempty(get(temp_h,'Children'))
        flag_rescale_axis = 1;
    end      
    
    % Is this 2D or 3D?
    dimension_of_points = 2; 

    % Find size of plotting domain
    allPointsBeingPlotted = [circle_points; nan nan];

    max_plotValues = max(allPointsBeingPlotted);
    min_plotValues = min(allPointsBeingPlotted);
    sizePlot = max(max_plotValues) - min(min_plotValues);
    nudge = sizePlot*0.006; %#ok<NASGU>

    % Find size of plotting domain
    if flag_rescale_axis
        percent_larger = 0.3;
        axis_range = max_plotValues - min_plotValues;
        if (0==axis_range(1,1))
            axis_range(1,1) = 2/percent_larger;
        end
        if (0==axis_range(1,2))
            axis_range(1,2) = 2/percent_larger;
        end
        if dimension_of_points==3 && (0==axis_range(1,3))
            axis_range(1,3) = 2/percent_larger;
        end

        % Force the axis to be equal?
        if 1==1
            min_valuesInPlot = min(min_plotValues);
            max_valuesInPlot = max(max_plotValues);
        else
            min_valuesInPlot = min_plotValues;
            max_valuesInPlot = max_plotValues;
        end

        % Stretch the axes
        stretched_min_vertexValues = min_valuesInPlot - percent_larger.*axis_range;
        stretched_max_vertexValues = max_valuesInPlot + percent_larger.*axis_range;
        axesTogether = [stretched_min_vertexValues; stretched_max_vertexValues];
        newAxis = reshape(axesTogether, 1, []);
        axis(newAxis);

    end
    goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end
    
    % Get meshgrid for streamline plotting
    [X,Y] = meshgrid(x,y);

    hold on;
    grid on;

    % Plot expanded sets
    figure(fig_num)
    axis([min(x), max(x), min(y), max(y)])
    for j = 1:size(expandedSets,3)
        axis equal
        hold on
        grid on
        % Visually expand envelope
        s = streamslice(X,Y,windFieldU,windFieldV);
        set(s,'Color',[0.6 0.6 0.6])
        plot(expandedSets(:,1,j),expandedSets(:,2,j),'LineWidth',2,'Color','black')
        drawnow
        clf
    end


    % Plot the wind field
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,x,y,'default',fig_num)
    
    % Plot the inputs
    plot(centers(:,1),centers(:,2),'k.','MarkerSize',30, 'DisplayName','Input: origin')
    plot(circle_points(:,1),circle_points(:,2),'k--','LineWidth',2,'DisplayName','Input: original radius')

    % Plot the final output
    plot(expandedSets(:,1,size(expandedSets,3)),expandedSets(:,2,size(expandedSets,3)),'LineWidth',2,'Color','black','DisplayName','Output: wind radius')

    % Turn on legend
    legend

    % Shut the hold off?
    if flag_shut_hold_off
        hold off;
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