function [path,cost,flagsAtLeastOnePointIsInPoly] = fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes(polytopes, startXY, finishXY, plannerMode, varargin)
% fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes 
% 
% sets up the information needed for the Dijkstra- Astar hybrid function
% and calls the function when the input polytopes are generated from the
% voronoi diagram
%
% FORMAT:
%
% [path,cost,err] = fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes(polytopes, startXY, finishXY, plannerMode, (bounds), (figNum))
%
% INPUTS:
%
%     polytopes: 
%
%     startXY: the start point vector (x, y, id, obs_id, beg_end)
%
%     finishXY: the finish point matrix of all valid finishes where each row
%     is a single finish point vector (x, y, id, obs_id, beg_end)
%
%     plannerMode: string containing option for planner behavior that indicates the planner mode
%       "legacy" only goes around obstacles
%       "through at vertices" allows the planner to go through or around each obstacle
%           but only entering or exiting at vertices
%       "through or around" allows the planner to go through all obstacles or around all
%       "straight through" the planner only goes straight from the start to the goal, calculating the cost
%     
%     (optional inputs)
%
%     bounds: b-by-2 matrix of xy coordinates of the boundaries the path
%       planner must stay within, where b is the number of boundary points and
%       b>=3. If this argument not specified, there are no bounds.
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
% OUTPUTS:
%
%    path: the mx5 matrix as produced by consisting of waypoints.  Each row
%    is a waypoint, and columns follow format of
%    (fcn_VGraph_polytopesGenerateAllPtsTable), namely:
%    [x-coordinate, y-coordinate, pointID, obstacleID, beginning or end of an obstacle set (1 or 0)]
%
%    cost: the total cost of the selected route
%    
%    err: error flag for reporting errors without stopping the code
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script:
% script_test_fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes
% for a full test suite.
%
% This function was written on 2019_06_13 by Seth Tau
% Questions or comments? sat5340@psu.edu

% Revision history:
% As: fcn_algorithm_setup_bound_Astar_for_tiled_polytopes
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
%
% As: fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes
% 2025_07_17 - K. Hayes
% -- Copied function from
%    fcn_algorithm_setup_bound_Astar_for_tiled_polytopes.m to follow library
%    conventions
% 2025_07_25 - K. Hayes
% -- fixed function formatting
% -- added input and debug checks
% 2025_07_29 - S. Brennan
% -- added plotting as part of standard debug output
% -- fixed incorrect argument list
% 2025_10_22 - K. Hayes
% -- changed all_pts generation to use
%    fcn_BoundedAStar_polytopesGenerateAllPtsTable instead of manually
% 2025_11_02 - S. Brennan
% -- changed fcn_BoundedAStar_polytopesGenerateAllPtsTable 
%    % to fcn_Visibility_polytopesGenerateAllPtsTable
%    % WARNING: inputs/outputs to this changed slightly. Function needs to 
%    % be rechecked
% 2025_11_13 - S. Brennan, sbrennan@psu.edu
% - fixed file name error:
%    % * from fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes
%    % * to fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes
%    % * from start to startXY (for inputs to function)
%    % * from finish to finishXY (for inputs to function)
%    % * from start to startPointData for vertex vector of start point
%    % * from finish to finishPointData for vertex vector of finish point
% - fixed header strings

% TO DO:
% -- input checking and description in header for polytopes

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
    MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS");
    MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG = getenv("MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS);
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
        narginchk(4,MAX_NARGIN);

        % % Check the startXY input, make sure it has 2 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     startXY, '2column_of_numbers');
        % 
        % % Check the finishXY input, make sure it has 2 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     finishXY, '2column_of_numbers');
    end
end

% Does user want to specify the bounds input?
bounds = []; % Default is 1
if 5 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        bounds = temp;
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

% check if the start or end are now within combined polytopes
flagThrowError = 0; % only gives soft errors errors that don't stop the code
flagEdgeCheck = 1; % checks for start or finish on polytope edges

 % check that start and end are outside of obstacles
[flagsAtLeastOnePointIsInPoly, startPolys, finishPolys] = ...
    fcn_BoundedAStar_polytopePointsInPolytopes(startXY, finishXY, polytopes, flagThrowError, flagEdgeCheck, 9999); 

% flagsAtLeastOnePointIsInPoly = 0; 

% start and finish outside the polytopes?
if ~any(flagsAtLeastOnePointIsInPoly) 

    % add points for start and finish if on an edge
    if ~isnan(startPolys) % startXY is on obstacle
        vertices = polytopes(startPolys).vertices;
        start_gap = fcn_BoundedAStar_polytopePointGapLocation(startXY,vertices(:,1:2));
        new_verts = [vertices(1:start_gap,:); startXY; vertices(start_gap+1:end,:)];
        polytopes(startPolys).vertices = new_verts;
        polytopes(startPolys).xv = new_verts(1:end-1,1)';
        polytopes(startPolys).yv = new_verts(1:end-1,2)';
        polytopes(startPolys).distances = sum((new_verts(1:end-1,:) - new_verts(2:end,:)).^2,2).^0.5;
    end

    if ~isnan(finishPolys) % finishXY is on obstacle
        vertices = polytopes(finishPolys).vertices;
        finish_gap = fcn_BoundedAStar_polytopePointGapLocation(finishXY,vertices(:,1:2));
        new_verts = [vertices(1:finish_gap,:); finishXY; vertices(finish_gap+1:end,:)];
        polytopes(finishPolys).vertices = new_verts;
        polytopes(finishPolys).xv = new_verts(1:end-1,1)';
        polytopes(finishPolys).yv = new_verts(1:end-1,2)';
        polytopes(finishPolys).distances = sum((new_verts(1:end-1,:) - new_verts(2:end,:)).^2,2).^0.5;
    end


    % Generate all point information for path planning
    [pointsWithData, startPointData, finishPointData]  = ...
        fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY);
    
    %%%%%
    % Commented out this section on 2025_11_14 - no idea why it's needed
    %%%%%
    % point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes
    % % give the same information to the starting and ending points
    % if startPolys ~= -1 % on obstacle edge
    %     % find adjacent points
    %     adj_pts = pointsWithData(pointsWithData(:,4)==startPolys,:);
    % 
    %     start_id = adj_pts(start_gap+1,3);
    %     start_beg_end = pointsWithData(start_id,5);
    % else
    %     start_id = point_tot+1;
    %     start_beg_end = 0;
    % end
    % if finishPolys ~= 0 % on obstacle edge
    %     % find adjacent points
    %     adj_pts = pointsWithData(pointsWithData(:,4)==finishPolys);
    % 
    %     finish_id = adj_pts(finish_gap+1,3);
    %     finish_beg_end = pointsWithData(finish_id,5);
    % else
    %     finish_id = point_tot+2;
    %     finish_beg_end = 0;
    % end
    % 
    % % Set up start and finish points with IDs for planner processing
    % startPointData = [startXY start_id startPolys start_beg_end];
    % finishPointData = [finishXY finish_id finishPolys finish_beg_end];
    %%%%%

    % Impose bounds, if they exist
    if ~isempty(bounds)
        flagWithinBound = inpolygon(pointsWithData(:,1),pointsWithData(:,2),bounds(:,1),bounds(:,2));
        boundedPoints = pointsWithData(flagWithinBound,:); % bound points at the start
    else
        boundedPoints = pointsWithData;
    end

    % find valid points
    [~,ia,ic] = unique(boundedPoints(:,1:2),'rows','stable');
    h = accumarray(ic, 1);
    validPoints = boundedPoints(ia(h==1),:);

    % Create polytopes for finding ellipses %%%%%%%% needs debugging for 3+
    % verts in a straight line
    ellipse_polytopes = polytopes;
    
    % calculate path
    [cost,path] = fcn_BoundedAStar_AStarBounded(startPointData,finishPointData,polytopes,pointsWithData,validPoints,plannerMode,ellipse_polytopes, (-1));

else % start or finish are inside the polytopes! cannot do anything
   path = [];
   cost = [];
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

    figure(figNum);
    hold on

    allPointsPlotted = pointsWithData(:,1:2);
    minX = min(allPointsPlotted(:,1));
    minY = min(allPointsPlotted(:,2));
    maxX = max(allPointsPlotted(:,1));
    maxY = max(allPointsPlotted(:,2));

    scaleX = maxX - minX;
    scaleY = maxY - minY;
    percentLarger = 0.3;
    new_axis = [minX-scaleX*percentLarger maxX+scaleX*percentLarger minY-scaleY*percentLarger maxY+scaleY*percentLarger];
    axis(new_axis);


    % Plot results
    plotFormat.LineWidth = 3;
    plotFormat.MarkerSize = 10;
    plotFormat.LineStyle = '-';
    plotFormat.Color = [0 0 1];

    % fillFormat = [1 0 0 0 0.5];
    fillFormat = [];
    h_plot = fcn_MapGen_plotPolytopes(polytopes, (plotFormat),(fillFormat),(figNum));
    set(h_plot,'DisplayName','Input: polytopes');
    plot([startXY(1) finishXY(1)],[startXY(2) finishXY(2)],'kx','LineWidth',2,'DisplayName','Input: start and end pts')
    plot(path(:,1),path(:,2),'k-','linewidth',2,'DisplayName','Output: route');
    box on
    xlabel('X Position')
    ylabel('Y Position')

    legend('Interpreter','none','Location','best');

end % Ends the flag_do_plot if statement



if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends main function



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
