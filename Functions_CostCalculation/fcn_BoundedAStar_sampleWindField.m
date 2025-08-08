function [resampledPoints, windFieldUc, windFieldVc] = fcn_BoundedAStar_sampleWindField(samplePoints, windFieldX, windFieldY, windFieldU, windFieldV, varargin)
% fcn_BoundedAStar_sampleWindField
% samples a wind field near a given point and outputs the resulting wind
% vector
%
% FORMAT:
% [windVector] = fcn_BoundedAStar_sampleWindField(samplePoint, windFieldX, windFieldY, windFieldU, windFieldV, (slowMode), (fig_num))
%
% INPUTS:
%
%     samplePoints: the nx2 matrix containing the [x, y] points at which
%     the wind field will be sampled
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
%     slowMode: a flag that makes the function use the slower 'for loop'
%     method of sampling multiple points. slowMode = 1 will enable slow
%     mode. any other input will use the default.
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     resampledPoints: an nx2 vector containing the [U, V] velocities at the
%     selected point
%
%     windFieldUc: an nxn matrix that is a transposed version of the wind
%     field. This allows the wind field to be sampled correctly with linear
%     indices in the format [x,y].
%
%     windFieldVc: an nxn matrix that is a transposed version of the wind
%     field. This allows the wind field to be sampled correctly with linear
%     indices in the format [x,y].
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_sampleWindField
% for a full test suite.
%
% This function was written on 2025_07_30 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_07_30 by K. Hayes
% -- first write of function using fcn_BoundedAStar_fillWindField as a
%    starter
% 2025_08_08 - K. Hayes
% -- updated input variable names and header info
% -- updated input checking
% -- added ability to pass more than one point
% -- moved previous method to 'slow' mode for potential comparisons
% -- updated default sampling method to match
%    fcn_INTERNAL_sampleWindAtPoints's fast method
% -- fixed coordinate definition within this fcn: the function now outputs
%    re-formatted versions of the wind fields that follow the [x, y]
%    conventions. Sampling the wind field also no longer requires the
%    'flip' fix

% TO-DO
% -- coordinate definition fixes
% -- time steps

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

% Does user want to specify the sampling mode
slowMode = 0; % Default is not to use slow mode
if 6 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        slowMode = temp;
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

if slowMode == 1
    indices = nan*size(samplePoints);
    for i = 1:size(samplePoints,1)
        thisPoint = samplePoints(i);
        thisX = thisPoint(1);
        thisY = thisPoint(2);

        % Find x and y indices in wind field near selected point
        xIndex = find(windFieldX>samplePoints(1),1,'first');
        yIndex = find(windFieldY>samplePoints(2),1,'first');
    
        % Handle out of bounds situations
        if isempty(xIndex)
            if thisX>windFieldX(1,end)
                xIndex = length(windFieldX(1,:));
            elseif thisX<windFieldX(1,1)
                xIndex = 1;
            else
                error('unknown situation occurred matching x value: %.2f to windFieldX',thisX);
            end
        end
        if isempty(yIndex)
         if thisY>windFieldY(1,end)
                yIndex = length(windFieldY(1,:));
            elseif thisY<windFieldY(1,1)
                yIndex = 1;
            else
                error('unknown situation occurred matching y value: %.2f to windFieldY',thisY);
            end
        end
        
        % Write indices
        indices(i,:) = [xIndex yIndex];
    end
    
else % default faster mode taken from fcn_INTERNAL_sampleWindAtPoints
    % Get discretization step size
    spatialStep = windFieldX(2) - windFieldX(1);

    % Determine x and y index of each point by subracting the first x/y
    % point from the sample points and dividing by the spatial step
    xIndices = floor((samplePoints(:,1)-windFieldX(1,1))/spatialStep)+1;
    yIndices = floor((samplePoints(:,2)-windFieldY(1,1))/spatialStep)+1;
    
    % Determine the highest possible index number for x or y (the length of the wind
    % field in each direction
    highestXindex = length(windFieldX);
    highestYindex = length(windFieldY);

    xIndices = max(1,min(highestXindex,xIndices));
    yIndices = max(1,min(highestYindex,yIndices));
    
    % Write indices
    indices = [xIndices yIndices];

end

% Use indices to sample wind field
linearInd = sub2ind(size(windFieldU),indices(:,1),indices(:,2));

windFieldUc = transpose(windFieldU);
windFieldVc = transpose(windFieldV);

resampledPoints = [windFieldUc(linearInd) windFieldVc(linearInd)];

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

    % Plot streamlines
    s = streamslice(windFieldX,windFieldY,windFieldU,windFieldV);
    set(s, 'Color', [0.6 0.6 0.6], 'HandleVisibility','off')
    
    % Plot sample point
    plot(samplePoints(1), samplePoints(2), 'rx', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'Sample Point')

    % Plot wind at sample point
    quiver(samplePoints(1),samplePoints(2),resampledPoints(1),resampledPoints(2),'DisplayName','Wind at Point','Color','blue','LineWidth',3,'AutoScaleFactor',1.25)
    
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

