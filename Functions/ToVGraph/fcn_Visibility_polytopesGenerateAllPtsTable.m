function [pointsWithData, startPointData, finishPointData] = ...
    fcn_Visibility_polytopesGenerateAllPtsTable(polytopes, varargin)
% fcn_Visibility_polytopesGenerateAllPtsTable
%
% A short function to turn polytope verticies, as well as user-defined
% start and finish vertices, into an Nx5 table of pointsWithData of the
% form used by visibility calculations and path planners.
% 
% The intent in creating this pointsWithData list is to avoid repeatedly
% looping through polytopes to check collisions. Note that visibility
% calculations must distinguish between points that are on the same
% polytope versus those that are not - for example, two points on the same
% obstacle may be visible to each other if adjacent, but not if their
% connectivity passes "through" the obstacle. This list of points, for each
% point, includes also the the obstacle ID number (or number of the
% polytope). As well, some calculations for polytopes must consider if the
% point starts or ends a looping around the perimeter of the polytope. Such
% points that start or end a "loop" must also be tagged accordingly. In
% summary, the following information is needed: the point location (XY),
% the unique numbering of the point (e.g. "point 8"), the numbering
% associated with the polytope/obstacle to which the point belongs (e.g.
% "obstacle 3", or in the case of "pure" points that have no associated
% obstacle (start and end for example), "obstacle -1". And finally, each
% point must be tagged whether the point starts (1) or ends (2) the
% perimeter of the obstacle (or is a start or end point, respectively).
% 
% The function allows start/end points to be added or omitted. If the
% user only gives polytopes, then the vertices of the polytopes are
% returned in matrix form. If the user gives the start and/or finish
% points, these points are appended to the polytope vertex list with index
% N+1 and N+2 respectively (for start and finish) where N is the number of
% vertices inside the polytope field.
%
% FORMAT:
% [pointsWithData, startPointData, finishPointData] = ...
%     fcn_Visibility_polytopesGenerateAllPtsTable( ...
%     polytopes, ....
%     (startXY), (finishXY), (figNum))
%
% INPUTS:
%
%     polytopes: the polytope struct array
%
%     (optional inputs)
%
%     startXY: the start point vector (x,y)
%
%     finishXY: the finish point vector (x,y)
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
% OUTPUTS:
%
%     pointsWithData: p-by-5 matrix of all the possible origination points for
%     visibility calculations including the vertex points on each obstacle,
%     and if the user specifies, the start and/or end points. If the
%     start/end points are omitted, the value of p is the same as the
%     number of points within the polytope field, numPolytopeVertices.
%     Otherwise, p is 1 or 2 larger depending on whether start/end is
%     given.
%
%    The information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id number
%         obstacle id number (-1 for start/end points)
%         beginning/ending indication (1 if the point is a beginning or
%         start point, 2 if ending point or finish point, and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%
%     startPointData: the start point vector as a 1x5 array with the same
%     information as pointsWithData. Returns empty if user does not
%     specify.
%
%     finishPointData: the finish point vector  as a 1x5 array with the same
%     information as pointsWithData. Returns empty if user does not
%     specify.
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_demo_fcn_BoundedAStar_Astar and
% script_test_fcn_Visibility_polytopesGenerateAllPtsTable
% for demonstration of this function in use.
%
% This function was written on 2024_05_08 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
% As: fcn_polytopes_generate_pointsWithData_table
% 2024_05_08, by Steve Harnett
% -- first write of function
% 2025_07_07 S. Brennan and K. Hayes
% -- changed demo script
%    from: script_test_fcn_algorithm_Astar
%    to:   script_demo_fcn_BoundedAStar_Astar
%
% As: fcn_BoundedAStar_polytopesGenerateAllPtsTable
% 2025_07_25 - K. Hayes
% -- copied to new function from fcn_polytopes_generate_pointsWithData_table
%    % to follow library convention
% 2025_08_05 - K. Hayes
% -- updated function formatting and header
%
% As: fcn_Visibility_polytopesGenerateAllPtsTable
% 2025_11_01 - S. Brennan
% -- renamed function 
%    % from: fcn_BoundedAStar_polytopesGenerateAllPtsTable
%    % to: fcn_Visibility_polytopesGenerateAllPtsTable
%    % moving the function out of BoundedAStar into Visibility library
% -- improved header docstrings to better explain the function
% -- made startXY and finishXY optional inputs, instead of required
% -- changed figNum to figNum
% -- changed startXY and finishXY to startXY and finishXY
% -- changed _MAPGEN_ to _VGRAPH_
% -- changed all_pts to pointsWithData
% -- refactored the function for clarity and to avoid changing the polytope
%    % input variable as an internal data source. Too easy to create a bug if
%    % inputs are changed within the function

%
% TO DO:
% -- fill in to-do items here.

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
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
        narginchk(1,MAX_NARGIN);

        % Check the polytopes input, make sure it is a structure
        assert(isstruct(polytopes));
    end
end

% Does user want to specify startXY?
startXY = []; % Default is to NOT specify startXY
% Check for user input
if 2 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        startXY = temp;
    end
end

% Does user want to specify finishXY?
finishXY = []; % Default is to NOT specify finishXY
% Check for user input
if 3 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        finishXY = temp;
    end
end

if ~isempty(startXY)
    % Check the startXY input, make sure it has 2 columns
    fcn_DebugTools_checkInputsToFunctions(...
        startXY, '2column_of_numbers');
end

if ~isempty(finishXY)
    % Check the finishXY input, make sure it has 2 columns
    fcn_DebugTools_checkInputsToFunctions(...
        finishXY, '2column_of_numbers');
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin)
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
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
Npolys = size(polytopes,2);
numPolytopeVertices = length([polytopes.xv]); % total number of vertices in the polytopes

% Initialize values
pointIDs = (1:numPolytopeVertices)';
obstacleIDs = zeros(numPolytopeVertices,1); % The ID number for the polytope
flagEndType_1isStart2isEnd = zeros(numPolytopeVertices,1); % is the point the start/end of an obstacle?

% Loop through polytopes, filling in data
currentPoint = 0;
for ith_poly = 1:Npolys % check each polytope
    numVerticesThisPoly = length(polytopes(ith_poly).xv);

    % obs_id is the same for every vertex on a single polytope
    obstacleIDs(currentPoint+1:currentPoint+numVerticesThisPoly,1) = ith_poly; 
    
    % the first vertices are marked with 1, last are marked with 2, and all others are 0
    flagEndType_1isStart2isEnd(currentPoint+1,1) = 1;
    flagEndType_1isStart2isEnd(currentPoint+numVerticesThisPoly,1) = 2;

    currentPoint = currentPoint+numVerticesThisPoly;
end

% Save results for polytopes
% FORMAT: [x y point_id obs_id beg_end]
polytopePointsWithData = [...
    [polytopes.xv]', ...
    [polytopes.yv]', ...
    pointIDs, ...
    obstacleIDs, ...
    flagEndType_1isStart2isEnd]; 


% Did user give start/finish points?
if ~isempty(startXY)
    startPointData = [startXY numPolytopeVertices+1 -1 1];
else
    startPointData = [];
end
if ~isempty(finishXY)
    finishPointData = [finishXY numPolytopeVertices+2 -1 2];
else
    finishPointData = [];
end

pointsWithData = [polytopePointsWithData; startPointData; finishPointData];

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

    allPoints = [[[polytopes.xv]' [polytopes.yv]']; startXY; finishXY];
    [newAxis, addNudge] = fcn_INTERNAL_rescaleAxis(allPoints);
    
    % Make axis slightly larger?
    if flag_rescale_axis
        axis(newAxis);
    end

    %%%%%%%%%%%%%
    % Plot the inputs

    % Plot the polytopes
    fcn_INTERNAL_plotPolytopes(polytopes, figNum)
    axis(newAxis);

    % Plot the start and end points?
    if ~isempty(startXY)
        plot(startXY(1),startXY(2),'.','Color',[0 0.5 0],'MarkerSize',20);
        text(startXY(:,1),startXY(:,2)+addNudge,'Start');
    end
    if ~isempty(finishXY)
        plot(finishXY(1),finishXY(2),'r.','MarkerSize',20);
        text(finishXY(:,1),finishXY(:,2)+addNudge,'Finish');
    end

    %%%%
    % Outputs
    colorOrdering = gca().ColorOrder;
    Ncolors = length(colorOrdering(:,1));
    for ith_poly = 1:Npolys
        thisColorIndex = mod(ith_poly-1,Ncolors)+1;
        thisColor = colorOrdering(thisColorIndex,:);
        indicesToPlot = polytopePointsWithData(:,4)==ith_poly;
        plot(polytopePointsWithData(indicesToPlot,1), polytopePointsWithData(indicesToPlot,2),...
            'LineStyle','none','Marker','o','MarkerFaceColor',thisColor);
        text(polytopePointsWithData(indicesToPlot,1)+addNudge,polytopePointsWithData(indicesToPlot,2)+addNudge,...
            string(polytopePointsWithData(indicesToPlot,3)),...
            'Color',thisColor);
    end
end

end

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
function [newAxis, addNudgeX] = fcn_INTERNAL_rescaleAxis(points)
% temp = axis;
temp = [min(points(:,1)) max(points(:,1)) min(points(:,2)) max(points(:,2))];
axis_range_x = temp(2)-temp(1);
axis_range_y = temp(4)-temp(3);

addNudgeX = axis_range_x*0.03;

percent_larger = 0.3;
newAxis = [temp(1)-percent_larger*axis_range_x, temp(2)+percent_larger*axis_range_x,  temp(3)-percent_larger*axis_range_y, temp(4)+percent_larger*axis_range_y];

end % Ends fcn_INTERNAL_rescaleAxis


%% fcn_INTERNAL_plotPolytopes
function fcn_INTERNAL_plotPolytopes(polytopes, figNum)
% A wrapper function for plotPolytopes, to plot the polytopes with same
% format

% axes_limits = [0 1 0 1]; % x and y axes limits
% axis_style = 'square'; % plot axes style
plotFormat.Color = 'Blue'; % edge line plotting
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 2; % linewidth of the edge
fillFormat = [1 0 0 1 0.4];
% FORMAT: fcn_MapGen_plotPolytopes(polytopes,fig_num,line_spec,line_width,axes_limits,axis_style);
fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum));
hold on
box on
% axis([-0.1 1.1 -0.1 1.1]);
xlabel('x [m]');
ylabel('y [m]');
end % Ends fcn_INTERNAL_plotPolytopes