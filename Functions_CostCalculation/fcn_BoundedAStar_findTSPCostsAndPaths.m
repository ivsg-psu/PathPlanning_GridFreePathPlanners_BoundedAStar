function [feasibleAllPoints, feasibleGoalIndices, ...
    feasibleCostsFromTo, pathsFromToFeasible] = ...
    fcn_BoundedAStar_findTSPCostsAndPaths(...
    radius, ...
    windFieldU, ...
    windFieldV, ... 
    windFieldX, ...
    windFieldY, ...
    startPoint, ...
    goalPoints, ...
    varargin)
% fcn_BoundedAStar_findTSPCostsAndPaths generates the cost and path matrix
% used for solving the Traveling Salesperson Problem.
%
% Uses
%
% FORMAT:
% [feasibleAllPoints, feasibleGoalIndices, ...
%     feasibleCostsFromTo, pathsFromToFeasible] = ...
%     fcn_BoundedAStar_findTSPCostsAndPaths(...
%     radius, ... 
%     windFieldU, ...
%     windFieldV, ...
%     windFieldX, ...
%     windFieldY, ...
%     startPoint, ...
%     goalPoints, ...
%     (cellArrayOfSearchOptions),...
%     (figNum));
%
% INPUTS:
%
%     radius: a 1x1 scalar representing the radius of travel without wind.
%     This is usually the travel speed multiplied by the time step.
%
%     windFieldU:  a matrix containing the u-direction components of the
%     wind velocity at each grid point
%
%     windFieldV:  a matrix containing the v-direction components of the
%     wind velocity at each grid point
%
%     windFieldX: a vector containing the x values assigned to each grid
%     point
% 
%     windFieldY: a vector containing the y values assigned to each grid
%     point
%
%     startPoint: a 1x2 vector representing the [x,y] values of the start
%     point, which is also the end point
%
%     goalPoints: a Nx2 vector representing the [x,y] values of the goal
%     points
%
%     (optional inputs)
%
%     cellArrayOfSearchOptions: allows the user to specify the TSP search
%     options. 
%     * Exit if infeasible goals
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     feasibleAllPoints: an Mx2 array of the "city" points, with no
%     repeats. Each must be reachable from at least one other point. First
%     point in the list is startPoint, points in rows 2+ are goalPoints.
% 
%     feasibleGoalIndices: the M indices, taken from the N input goal
%     points (with M<=N) that indicate which are feasbile. An infeasible
%     point is one where no other goalPoints or startPoint can reach it.
%
%     feasibleCostsFromTo: an MxM matrix specifying the costs to traverse
%     from a point (row) to another point (column). 
% 
%     pathsFromToFeasible: an MxM cell array of XYUV points that lead from
%     a row point to a column point.
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_findTSPCostsAndPaths
% for a full test suite.
%
% This function was written on 2025_08_28 by S. Brennan
% Questions or comments? contact S. Brennan sbrennan@psu.edu 
% or K. Hayes, kxh1031@psu.edu

% REVISION HISTORY:
% 2025_08_28 by S. Brennan
% - in fcn_BoundedAStar_findTSPCostsAndPaths
%   % * first write of function using fcn_BoundedAStar_findTSPCostsAndPaths
%   %   % as a starter

% TO-DO
% (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 9; % The largest Number of argument inputs to the function
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
    debug_figNum = 999978; 
else
    debug_figNum = []; 
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
        narginchk(7,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        fcn_DebugTools_checkInputsToFunctions(...
            radius, '1column_of_numbers',[1 1]);
    end
end

% Does user want to specify cellArrayOfSearchOptions input?
% Defaults
cellArrayOfSearchOptions = cell(5,1);
cellArrayOfSearchOptions{1} = 100; % Nsteps
%cellArrayOfSearchOptions{2} = 1;   % flagStopIfEntireFieldCovered
%cellArrayOfSearchOptions{3} = 4*(windFieldX(2)-windFieldX(1)); % toleranceToStopIfSameResult
%cellArrayOfSearchOptions{4} = [];  % allGoalPointsList
%cellArrayOfSearchOptions{5} = 0;   % flagStopIfHitOneGoalPoint
if 8 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        cellArrayOfSearchOptions = temp;
    end
end
Nsteps = cellArrayOfSearchOptions{1};
% flagStopIfEntireFieldCovered = cellArrayOfSearchOptions{2};
% toleranceToStopIfSameResult = cellArrayOfSearchOptions{3};
% temp = cellArrayOfSearchOptions{4};
% flagStopIfHitOneGoalPoint = cellArrayOfSearchOptions{5};

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

% Save the original goal points, in case some are infeasible. This helps
% with debugging later
NumOriginalPoints = length(goalPoints(:,1));
originalGoalPoints = goalPoints;

% DEBUGGING: Plot the input situation: wind field, startPoint, and goalPoints
if flag_do_debug
    fcn_INTERNAL_debugPlotInputs(windFieldU,windFieldV,windFieldX,windFieldY, ...
        startPoint, originalGoalPoints, debug_figNum)
end

%%%%
% Make sure no points are repeated, especially the start/end point.
% Warn user if goal points are repeated and remove repeats. Throw error if
% start/end point is repeated.
allPointsOriginal = [startPoint; goalPoints];
[~, IA] = unique(allPointsOriginal,'rows','last');
allPointsOriginalUnique = unique(allPointsOriginal,'rows','stable');
if length(allPointsOriginalUnique(:,1))~=length(allPointsOriginal(:,1))
    pointNumbersMissing = ones(length(allPointsOriginal(:,1)),1);
    pointNumbersMissing(IA) = 0;
    pointsMissing = find(pointNumbersMissing==1);
    
    if any(1==pointsMissing)
        warning('on','backtrace');
        warning('Catastrophic error detected where start point is also a goal point');
        error('The start/end point is repeated as a goal point. Unable to continue, as start/end point cannot be visited both once and twice.');
    end

    fprintf(1,'The following goal points were found as repeated:\n');
    for ith_fromPoint = 1:length(pointsMissing)
        thisPointIndex = pointsMissing(ith_fromPoint);
        fprintf(1,'\tPoint %.0d, with [X Y] of: %.2f %.2f\n',thisPointIndex-1,allPointsOriginal(thisPointIndex,1),allPointsOriginal(thisPointIndex,2));
    end
    warning('on','backtrace');
    warning('Some goal points were repeated. Repeats listed above have been deleted, but erroneous results may occur.');
end

%%%%
% Number all the points (now unique)
NUniqueGoals = length(allPointsOriginalUnique(:,1));
pointNumbers = (1:NUniqueGoals)';
if flag_do_debug
    figure(debug_figNum);
    % Number the unique points
    for ith_fromPoint = 1:NUniqueGoals
        text(allPointsOriginalUnique(ith_fromPoint,1)+0.2,allPointsOriginalUnique(ith_fromPoint,2),sprintf('%.0f',pointNumbers(ith_fromPoint)));
    end
end

%%%%
% For each point, find costs to go from that point to the others
% Costs are saved as "from" as rows, and "to" as columns. Note: self costs
% are -1. Costs that are not possible (infeasible) are NaN.
[costsFromTo, pathsFromTo, reachableSet] = fcn_INTERNAL_findCostsFromTo(...
    NUniqueGoals, allPointsOriginalUnique, pointNumbers, Nsteps,...
    radius, windFieldU, windFieldV, windFieldX, windFieldY,...
    flag_do_debug, debug_figNum);

%%%%%
% Remove infeasible (unreachable) goal points
% Save indices of original costs, so that when the rows/cols are deleted,
% we can remap the cell array of paths correctly
previousRows = nan(NUniqueGoals,NUniqueGoals);
previousCols = nan(NUniqueGoals,NUniqueGoals);
for ith_row = 1:NUniqueGoals
    for jth_col = 1:NUniqueGoals
        previousRows(ith_row,jth_col) = ith_row;
        previousCols(ith_row,jth_col) = jth_col;
    end
end

% Delete infeasible goal points from cost matrix
feasibleCostsFromTo = costsFromTo;
if any(isnan(costsFromTo),'all')

    % Find all the nan values in the cost matrix
    notReachable = isnan(costsFromTo);

    % Add up the columns via a row-sum, these are the "to" locations
    rowSum = sum(notReachable,1);

    % If row sum is Npoints-1, the every other "from" point found this "to"
    % point not reachable. So this goal is not reachable. Find these "bad"
    % goals
    badGoalIndices = find(rowSum==(NUniqueGoals-1));

    pointsToKeep = fcn_INTERNAL_flagAllTrueBut(NUniqueGoals,badGoalIndices);

    % Delete rows/columns
    feasibleCostsFromTo(badGoalIndices,:) = [];
    feasibleCostsFromTo(:,badGoalIndices) = [];
    previousRows(badGoalIndices,:) = [];
    previousRows(:,badGoalIndices) = [];
    previousCols(badGoalIndices,:) = [];
    previousCols(:,badGoalIndices) = [];

    % Keep good points
    feasibleAllPoints = allPointsOriginalUnique(pointsToKeep,:);
    feasiblePointIndices = pointNumbers(pointsToKeep,:);
    notFeasiblePointNumbers = pointNumbers(~pointsToKeep,:);

    if ~any(1==feasiblePointIndices,'all')
        warning('on','backtrace');
        warning('Catastrophic error detected where start point is not reachable from goal points');
        error('The start/end point is not reachable!');
    end
    feasibleGoalIndicesWithStart = feasiblePointIndices-1;
    feasibleGoalIndices = feasibleGoalIndicesWithStart(2:end,:);

    NbadGoals = length(notFeasiblePointNumbers);
    if NbadGoals>0
        fprintf(1,'The following goal points were found as infeasible:\n');
        for ith_fromPoint = 1:NbadGoals
            thisPointIndex = notFeasiblePointNumbers(ith_fromPoint);
            fprintf(1,'\tPoint %.0d, with [X Y] of: %.2f %.2f\n',thisPointIndex,allPointsOriginalUnique(thisPointIndex,1), allPointsOriginalUnique(thisPointIndex,2));
        end
    end
else
    % All points are feasible
    feasibleAllPoints = allPointsOriginalUnique; 
    feasibleGoalIndices = (1:NumOriginalPoints)';
end

NFeasibleGoals = size(feasibleCostsFromTo,1);

%%%%%
% Save the paths to the feasible goal points
pathsFromToFeasible = cell(NFeasibleGoals,NFeasibleGoals);
for ith_row = 1:NFeasibleGoals
    for jth_col = 1:NFeasibleGoals
        sourceRow = previousRows(ith_row,jth_col);
        sourceCol = previousCols(ith_row,jth_col);
        pathsFromToFeasible{ith_row, jth_col} = pathsFromTo{sourceRow, sourceCol};
    end
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
    temp_h = figure(figNum); %#ok<NASGU>
    % flag_rescale_axis = 0;
    % if isempty(get(temp_h,'Children'))
    %     flag_rescale_axis = 1;
    % end      
    
    % Is this 2D or 3D?
    dimension_of_points = 2; %#ok<NASGU>

    % Find size of plotting domain
    allPointsBeingPlotted = [reachableSet; nan nan];

    max_plotValues = max(allPointsBeingPlotted);
    min_plotValues = min(allPointsBeingPlotted);
    sizePlot = max(max_plotValues) - min(min_plotValues);
    nudge = sizePlot*0.006; 

    % % Find size of plotting domain
    % if flag_rescale_axis
    %     % NO NEED TO RESIZE THE AXIS FOR IMAGE PLOTTING
    %     % percent_larger = 0.3;
    %     % axis_range = max_plotValues - min_plotValues;
    %     % if (0==axis_range(1,1))
    %     %     axis_range(1,1) = 2/percent_larger;
    %     % end
    %     % if (0==axis_range(1,2))
    %     %     axis_range(1,2) = 2/percent_larger;
    %     % end
    %     % if dimension_of_points==3 && (0==axis_range(1,3))
    %     %     axis_range(1,3) = 2/percent_larger;
    %     % end
    %     % 
    %     % % Force the axis to be equal?
    %     % if 1==1
    %     %     min_valuesInPlot = min(min_plotValues);
    %     %     max_valuesInPlot = max(max_plotValues);
    %     % else
    %     %     min_valuesInPlot = min_plotValues;
    %     %     max_valuesInPlot = max_plotValues;
    %     % end
    %     % 
    %     % % Stretch the axes
    %     % stretched_min_vertexValues = min_valuesInPlot - percent_larger.*axis_range;
    %     % stretched_max_vertexValues = max_valuesInPlot + percent_larger.*axis_range;
    %     % axesTogether = [stretched_min_vertexValues; stretched_max_vertexValues];
    %     % newAxis = reshape(axesTogether, 1, []);
    %     % axis(newAxis);
    % 
    % end
    % % goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end   

    % Turn on legend
    legend('Interpreter','none','Location','northeast');

    %%%%%%%
    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    % Get meshgrid for streamline plotting
    [meshX,meshY] = meshgrid(windFieldX,windFieldY);
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6],'HandleVisibility','off')

    colorOrder = get(gca, 'ColorOrder');
    Ncolors = length(colorOrder(:,1));

    %%%%%%%
    % Plot the start point
    plot(startPoint(:,1),startPoint(:,2),'.', ...
        'Color',colorOrder(1,:),'MarkerSize',30, ...
        'DisplayName','Input: startPoint');

    %%%%%%%
    % Plot the goal points in different . Circle in red any that are not
    % feasible. 

    flagPlottedInfeasible = 0;
    for ith_fromPoint = 1:NumOriginalPoints
        thisColorRow = mod(ith_fromPoint,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        h_plot = plot(originalGoalPoints(ith_fromPoint,1),originalGoalPoints(ith_fromPoint,2),'.',...
            'Color',thisColor,'MarkerSize',30,'LineWidth', 2);
        if ith_fromPoint ==1
            set(h_plot, 'DisplayName','Input: goalPoints');
        else
            set(h_plot,'HandleVisibility','off');
        end

        %%%%%%%%%%
        % Number the points
        text(originalGoalPoints(ith_fromPoint,1)+nudge,originalGoalPoints(ith_fromPoint,2),...
            sprintf('%.0f',ith_fromPoint));

        %%%%%%%%%%
        % Circle the infeasible points
        % Are there any infeasible points? If there are, the length of
        % feasibleGoalIndices will be different than the number of input
        % originalGoalPoints
        if NumOriginalPoints~=length(feasibleGoalIndices(:,1)) && ~any(ith_fromPoint==feasibleGoalIndices)
            if flagPlottedInfeasible==0
                flagPlottedInfeasible = 1;
                plot(originalGoalPoints(ith_fromPoint,1),originalGoalPoints(ith_fromPoint,2),'o',...
                    'Color',[1 0 0],'LineWidth',3,'MarkerSize',15,'DisplayName','Infeasible Point');
            else
                plot(originalGoalPoints(ith_fromPoint,1),originalGoalPoints(ith_fromPoint,2),'o',...
                    'Color',[1 0 0],'LineWidth',3,'MarkerSize',15,'HandleVisibility','off');
            end
        end
    end

    %%%%%%%%%
    % Plot the paths to feasible goal
    flagWasPlotted = 0;
    for ith_fromIndex = 1:size(pathsFromTo,1)
        thisColorRow = mod(ith_fromIndex-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);
        for ith_goal = 1:size(pathsFromTo,2)
            thisPath =  pathsFromTo{ith_fromIndex,ith_goal};
            if ~isempty(thisPath)
                if 0==flagWasPlotted
                    flagWasPlotted = 1;
                    plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth', 2,'DisplayName','Output: pathsFromToFeasible');
                else
                    plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth', 2,'HandleVisibility','off');
                end
            end
        end
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

%% fcn_INTERNAL_debugPlotInputs
function fcn_INTERNAL_debugPlotInputs(windFieldU,windFieldV,windFieldX,windFieldY, startPoint, originalGoalPoints, debug_figNum)
figure(debug_figNum);
clf;
hold on;

%%%%
% Plot the windfield as an image
fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',debug_figNum);
% Get meshgrid for streamline plotting
[meshX,meshY] = meshgrid(windFieldX,windFieldY);
s = streamslice(meshX,meshY,windFieldU,windFieldV);
set(s,'Color',[0.6 0.6 0.6])

%%%%
% Set colors
colorOrder = get(gca, 'ColorOrder');
Ncolors = length(colorOrder(:,1));

% Plot the start point
plot(startPoint(:,1),startPoint(:,2),'.','Color',colorOrder(1,:),'MarkerSize',30,'DisplayName','Input: startPoint');

% Plot the goal points
for ith_fromPoint = 1:length(originalGoalPoints(:,1))
    thisColorRow = mod(ith_fromPoint,Ncolors)+1;
    plot(originalGoalPoints(ith_fromPoint,1),originalGoalPoints(ith_fromPoint,2),'.',...
        'Color',colorOrder(thisColorRow,:),'MarkerSize',20,'LineWidth', 2, 'DisplayName','Input: goalPoints');
end

end % Ends fcn_INTERNAL_debugPlotInputs

%% fcn_INTERNAL_flagAllTrueButIth
function flags = fcn_INTERNAL_flagAllTrueBut(Npoints,butIndices)
% Creates an array of "true" flags, shutting off the "but" points
flags = true(Npoints,1);
flags(butIndices,1) = 0;
end % Ends fcn_INTERNAL_flagAllTrueButIth


%% fcn_INTERNAL_findCostsFromTo
function [costsFromTo, pathsFromTo, reachableSet] = fcn_INTERNAL_findCostsFromTo(...
    NUniqueGoals, allPoints, pointNumbers, Nsteps,...
    radius, windFieldU, windFieldV, windFieldX, windFieldY,...
    flag_do_debug, debug_figNum)
%%%%
% For each point, find costs to go from that point to the others
% Costs are saved as "from" as rows, and "to" as columns. Note: self costs
% are -1. Costs that are not possible (infeasible) are NaN.
costsFromTo = nan(NUniqueGoals,NUniqueGoals);
pathsFromTo = cell(NUniqueGoals,NUniqueGoals);

% Loop through the goals, doing expansions on each
for ith_fromPoint = 1:NUniqueGoals
    % For each source point, find the costs to all the other points. For
    % the self-to-self cost, we'll treat this as infinite to avoid pushing
    % points to loop back onto themselves

    startPoint = allPoints(ith_fromPoint,:);
    notThisPoint = fcn_INTERNAL_flagAllTrueBut(NUniqueGoals,ith_fromPoint);

    tempGoalPoints = allPoints(notThisPoint,:);
    tempGoalPointIDs = pointNumbers(notThisPoint,:);

    %%%%
    % Check to see if the goal points are feasible
    flagWindRoundingType = 1;
    cellArrayOfWindExitConditions = cell(5,1);
    cellArrayOfWindExitConditions{1} = Nsteps; % Nsteps
    cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
    cellArrayOfWindExitConditions{3} = [];   % toleranceToStopIfSameResult
    cellArrayOfWindExitConditions{4} = tempGoalPoints;  % allGoalPointsList
    cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint

    if 1==0
        save('BUG_90003_fcn_BoundedAStar_expandReachabilityWithWind.mat',...
            'radius', 'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY', ...
            'startPoint', 'flagWindRoundingType','cellArrayOfWindExitConditions');
    end

    % Call function to expand outward into wind field
    % figure(222222);
    % clf;
    try
        [reachableSet, exitCondition, cellArrayOfExitInfo, ...
            reachableSetExactCosts, cellArrayOfReachableSetPaths] = fcn_BoundedAStar_expandReachabilityWithWind(...
            radius, windFieldU, windFieldV, windFieldX, windFieldY,...
            (startPoint), (flagWindRoundingType), (cellArrayOfWindExitConditions), [], (-1));
    catch
        disp('Stop here');
    end

    if 4~=exitCondition
        warning('Not all goal points are reachable from other goal points');
    end
    reachableFlags = cellArrayOfExitInfo{2};

    % Save the costs and paths
    if 1==1
        costsFromTo(ith_fromPoint,tempGoalPointIDs) = reachableSetExactCosts';
        for ith_goal = 1:length(tempGoalPointIDs)
            thisGoalID = tempGoalPointIDs(ith_goal);
            pathsFromTo{ith_fromPoint,thisGoalID} = cellArrayOfReachableSetPaths{ith_goal};
        end
    else
        % Use approximate costs
        costsFromTo(ith_fromPoint,tempGoalPointIDs) = reachableFlags';
    end

    costsFromTo(ith_fromPoint,ith_fromPoint) = -1; % Self costs are -1

    if flag_do_debug
        figure(debug_figNum);

        colorOrder = get(gca, 'ColorOrder');
        Ncolors = length(colorOrder(:,1));

        thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        reachableIndices = tempGoalPointIDs(reachableFlags>0);
        feasibleGoalPoints = allPoints(reachableIndices,:);

        % Plot the reachableSet output for this source point
        if 1==0
            plot( reachableSet(:,1), reachableSet(:,2), 'LineWidth', 3, 'Color', thisColor, 'HandleVisibility', 'off'); % ,'DisplayName','Output: finalReachableSet')
        end

        % Plot the feasible goal points
        plot(feasibleGoalPoints(:,1),feasibleGoalPoints(:,2), 'o', 'Color', thisColor,...
            'MarkerSize', 5+5*ith_fromPoint, 'LineWidth', 2, 'HandleVisibility', 'off'); %'DisplayName','feasibleGoalPoints');

        % Plot the paths to feasible goal points
        for ith_goal = 1:length(tempGoalPointIDs)
            thisPath =  pathsFromTo{ith_fromPoint,ith_goal};
            if ~isempty(thisPath)
                plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth', 2,'HandleVisibility','off');
            end
        end

    end % Ends if flag_do_debug

end % Ends looping through "from" points to generate entire matrix of costs
end % Ends fcn_INTERNAL_findCostsFromTo