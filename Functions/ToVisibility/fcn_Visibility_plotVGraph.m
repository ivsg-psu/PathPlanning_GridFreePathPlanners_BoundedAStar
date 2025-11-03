function h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts,  styleString, varargin)
% fcn_Visibility_plotVGraph
%
% Plots a visibility graph given a generated vgraph and list of all points.
% Allows user to specify the plot styleString and optional from/to indices
% of the vertices to plot.
%
% FORMAT:
%
% fcn_Visibility_plotVGraph(vgraph, all_pts,  styleString, (selectedFromToIndices), (figNum))
%
% INPUTS:
%
%    vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%    A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%    all_pts: the nx5 list of all points in the space to be searched, with
%    the exception of the start and finish, with columns containing the
%    following information
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number (-1 if none)
%       is beginning/end of obstacle (1 if yes, 0 if no)
%
%    styleString: string of format '-g' indicating the line style to be
%    used when plotting the visibility graph
%
% (OPTIONAL INPUTS)
%
%    selectedFromToIndices: default is [] (empty) which plots all to/from
%    combinations. If user enters a 1x1 integer, the integer is taken to be
%    the index number to plot the "from" index range. If entered as a 1x2
%    vector, the first value is the "from" index, the second value is the
%    "to" index.
%
%    saveFile: a string specifying the file where an animated GIF is saved.
%
%    figNum: a figure number to plot results. If set to -1, skips any
%    input checking or debugging, uses current figure for plotting, and
%    sets up code to maximize speed.
%
% OUTPUTS:
%
%    h_plot: a handle to the plot handle
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_Visibility_plotVGraph
% for a full test suite.
%

% REVISION HISTORY:
%
% 2025_10_06 - S. Brennan, sbrennan@psu.edu
% -- first write of the function
% 2025_10_08 - K. Hayes, kaeleahayes@psu.edu
% -- added function header and standard formatting
% 2025_10_10 - S. Brennan, sbrennan@psu.edu
% -- added options to include selectedFromToIndices, figNum
% -- updated docstrings in header
% -- added test script
% 2025_10_28 - S. Brennan, sbrennan@psu.edu
% -- updated docstrings in header
% -- fixed fig_num to figNum
% -- added options to include saveFile for animated GIF

% TO DO:
%

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
    MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS");
    MATLABFLAG_VGRAPH_FLAG_DO_DEBUG = getenv("MATLABFLAG_VGRAPH_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_VGRAPH_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_VGRAPH_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS);
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

        % Check the all_pts input, make sure it has 5 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     all_pts, '5column_of_numbers');

        % Check that all_pts is same size as visibility
        assert(length(all_pts(:,1))==length(vgraph(:,1)));

    end
end

% Does user want to specify selectedFromToIndices?
selectedFromToIndices = []; % initialize default values
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        selectedFromToIndices = temp;
    end
end

% Does user want to specify saveFile?
flag_saveMovie = 0;
saveFile = [];
if 5 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        saveFile = temp;
        flag_saveMovie = 1;
    end
end

% Does user want to show the plots?
flag_do_plots = 1; % Default is to ALWAYS show plots
figNum = []; % Default is empty - which is filled in later
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        figure(figNum);
        flag_do_plots = 1;
    end
end

if isempty(figNum)
    figNum = gcf;
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

Npoints = size(vgraph,1);

% Define fromRange and toRange
if isempty(selectedFromToIndices)
    fromRange = 1:Npoints;
    toRange = 1:Npoints;
else
    fromRange = selectedFromToIndices(1):selectedFromToIndices(1);
    if length(selectedFromToIndices)>1
        toRange = selectedFromToIndices(2):selectedFromToIndices(2);
    else
        toRange = 1:Npoints;
    end
end

% Load up data arrays for plotting
allPointsToPlot = [];
allThisFromPointsToPlot = cell(1,1);
for ith_fromIndex = fromRange
    thisFromPointsToPlot = [];
    for jth_toIndex = toRange
        if vgraph(ith_fromIndex,jth_toIndex) == 1
            thisFromPointsToPlot = [thisFromPointsToPlot; [all_pts(ith_fromIndex,1:2); all_pts(jth_toIndex,1:2); nan(1,2)]]; %#ok<AGROW>
        end
    end
    allPointsToPlot = [allPointsToPlot; thisFromPointsToPlot]; %#ok<AGROW>
    allThisFromPointsToPlot{ith_fromIndex,1} = thisFromPointsToPlot;
    
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

    % check whether the figure already has data
    temp_h = figure(figNum);
    flag_rescale_axis = 0; 
    if isempty(get(temp_h,'Children'))
        flag_rescale_axis = 1; % Set to 1 to force rescaling
        axis equal
    end        

    hold on;


    % Make axis slightly larger?
    if flag_rescale_axis
        fcn_INTERNAL_rescaleAxis;
    end

    if 1~=flag_saveMovie % To save movie
        h_plot = plot(allPointsToPlot(:,1),allPointsToPlot(:,2),styleString);
    else
        delayTime = 0.5; % Delay between frames in seconds
        loopCount = Inf; % Loop indefinitely (0 for no loop)

        h_plot = zeros(length(fromRange),1);
        for ith_fromIndex = fromRange
            % Grab the index we should plot
            thisIndex = fromRange(ith_fromIndex);

            % Grab the data for this plotting index
            thisFromPointsToPlot = allThisFromPointsToPlot{thisIndex,1}; 
            h_plot(ith_fromIndex) = plot(thisFromPointsToPlot(:,1),thisFromPointsToPlot(:,2),styleString);
            drawnow;

            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image

            % Write the frame to the GIF
            if ith_fromIndex == 1
                % Create a new GIF file for the first frame
                imwrite(imind, cm, saveFile, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
            else
                % Append subsequent frames
                imwrite(imind, cm, saveFile, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end % Ends for loop
    end % Ends if making movie

end % Ends if flag_do_plot

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends the function

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

%% fcn_INTERNAL_rescaleAxis
function fcn_INTERNAL_rescaleAxis
temp = axis;
%     temp = [min(points(:,1)) max(points(:,1)) min(points(:,2)) max(points(:,2))];
axis_range_x = temp(2)-temp(1);
axis_range_y = temp(4)-temp(3);
percent_larger = 0.3;
axis([temp(1)-percent_larger*axis_range_x, temp(2)+percent_larger*axis_range_x,  temp(3)-percent_larger*axis_range_y, temp(4)+percent_larger*axis_range_y]);

end % Ends fcn_INTERNAL_rescaleAxis