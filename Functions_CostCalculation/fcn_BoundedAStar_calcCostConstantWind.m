function windRadius = fcn_BoundedAStar_calcCostConstantWind(windVector, radius, varargin)
% fcn_BoundedAStar_calcCostConstantWind
% calculates the resulting reachable radius from a point at the center of a
% circle, where the circle has a radius equal to the zero-wind distance.
%
% FORMAT:
% windRadius = fcn_BoundedAStar_calcCostConstantWind(windVector, radius, (fig_num))
%
% INPUTS:
%
%     windVector: a 1x2 vector of [U V] data where U is wind speed in the
%     East direction and V is wind speed in the N direction
%
%     radius: a 1x1 scalar representing the radius of travel without wind
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
% See the script: script_test_fcn_BoundedAStar_calcCostConstantWind
% for a full test suite.
%
% This function was written on 2025_07_11 by Sean Brennan
% Questions or comments? contact sbrennan@psu.edu

% REVISION HISTORY:
% 2025_07_11 by Sean Brennan
% -- first write of function using fcn_MapGen_generatePolysFromTiling in
% MapGen library as a starter
% 2025_07_14 by K. Hayes, kxh1031@psu.edu
% -- added support for different start points

% TO-DO
% (none)

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
        narginchk(2,MAX_NARGIN);

        % Check the windVector input, make sure it is '2column_of_numbers'
        % type with exactly 1 row
        fcn_DebugTools_checkInputsToFunctions(...
            windVector, '2column_of_numbers',[1 1]);

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

centers = startPoint;
circle_points = fcn_INTERNAL_plotCircle(centers,radius);

% Solution is trivial: take the final position, add the "push" direction
% from the wind. That's the final boundary. This is simply using vector
% addition of the heading vector added to the local wind vector. The reason
% for using this method is that it is very easy, later, to add changing
% wind fields and integrate along them.
windRadius = circle_points + windVector;

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

    hold on;
    grid on;

    % Plot the inputs:
    plot(centers(:,1),centers(:,2),'b.','MarkerSize',30, 'DisplayName','Input: origin')
    plot(circle_points(:,1),circle_points(:,2),'b-','DisplayName','Input: original radius')

    % Plot the outputs:
    plot(windRadius(:,1),windRadius(:,2),'-','DisplayName','Output: windRadius')

    legend('Interpreter','none');
    xlabel('X-East');
    ylabel('Y-North');

    axis(goodAxis);
    axis equal;

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