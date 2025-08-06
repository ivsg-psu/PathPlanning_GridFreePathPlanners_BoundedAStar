function [orderedVisitSequence] = ...
    fcn_BoundedAStar_solveTSPwithWind(...
    radius, ...
    windFieldU, ...
    windFieldV, ... 
    windFieldX, ...
    windFieldY, ...
    startPoint, ...
    goalPoints, ...
    varargin)
% fcn_BoundedAStar_solveTSPwithWind solves the Traveling Salesman Problem
% within a wind field
%
% Uses
%
% FORMAT:
% [finalReachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_solveTSPwithWind(...
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
%     options. (Not coded yet)
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
%     orderedVisitSequence: the set of points, starting and ending with the
%     startPoint, containing the visit sequence for goalPoints.
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_solveTSPwithWind
% for a full test suite.
%
% This function was written on 2025_08_05 by S. Brennan
% Questions or comments? contact S. Brennan sbrennan@psu.edu 
% or K. Hayes, kxh1031@psu.edu

% REVISION HISTORY:
% 2025_08_05 by S. Brennan
% - first write of function using fcn_BoundedAStar_solveTSPwithWind
%   % as a starter
% - got TSP solution intialization working with greedy method, which
%   % produces a quick upper bound estimate on sim time
% - still need to get TSP working with Djkstra's method

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

flag_do_debug = 1;

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
        narginchk(5,MAX_NARGIN);

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
Ngoals = length(goalPoints(:,1));

% Initialize output
orderedVisitSequence = nan(Ngoals+2,2);

% Save the original goal points, in case some are infeasible
originalGoalPoints = goalPoints;

% Get meshgrid for streamline plotting
[meshX,meshY] = meshgrid(windFieldX,windFieldY);

% Plot the wind field, startPoint, and goalPoints?
if flag_do_debug
    figure(debug_figNum);
    clf;
    hold on;
    
    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',debug_figNum);
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6])

    colorOrder = get(gca, 'ColorOrder');

    % Plot the start point
    plot(startPoint(:,1),startPoint(:,2),'.','Color',colorOrder(1,:),'MarkerSize',30,'DisplayName','Input: startPoint');

    % Plot the goal points
    for ith_point = 1:length(originalGoalPoints(:,1))
        plot(originalGoalPoints(ith_point,1),originalGoalPoints(ith_point,2),'.','Color',colorOrder(ith_point+1,:),'MarkerSize',20,'LineWidth', 2, 'DisplayName','Input: goalPoints');
    end
end

%%%%
% Make sure no points are repeated, especially the start/end point.
% Warn user if goal points are repeated and remove repeats. Throw error if
% start/end point is repeated.
allPointsOriginal = [startPoint; goalPoints];
[~, IA] = unique(allPointsOriginal,'rows','last');
allPoints = unique(allPointsOriginal,'rows','stable');
if length(allPoints(:,1))~=length(allPointsOriginal(:,1))
    pointNumbersMissing = ones(length(allPointsOriginal(:,1)),1);
    pointNumbersMissing(IA) = 0;
    pointsMissing = find(pointNumbersMissing==1);
    
    if any(1==pointsMissing)
        warning('on','backtrace');
        warning('Catastrophic error detected where start point is also a goal point');
        error('The start/end point is repeated as a goal point. Unable to continue, as start/end point cannot be visited both once and twice.');
    end

    fprintf(1,'The following goal points were found as repeated:\n');
    for ith_point = 1:length(pointsMissing)
        thisPointIndex = pointsMissing(ith_point);
        fprintf(1,'\tPoint %.0d, with [X Y] of: %.2f %.2f\n',thisPointIndex-1,allPointsOriginal(thisPointIndex,1),allPointsOriginal(thisPointIndex,2));
    end
    warning('on','backtrace');
    warning('Some goal points were repeated. Repeats listed above have been deleted, but erroneous results may occur.');
end

%%%%
% Number all the points (now unique)
Npoints = length(allPoints(:,1));
pointNumbers = (1:Npoints)';
if flag_do_debug
    figure(debug_figNum);
    for ith_point = 1:Npoints
        text(allPoints(ith_point,1),allPoints(ith_point,2),sprintf('%.0f',pointNumbers(ith_point)));
    end
end


%%%%
% For each point, find costs to go from that point to the others
% Costs are saved as "from" as rows, and "to" as columns. Note: self costs
% are -1. Costs that are not possible (infeasible) are NaN.
costsFromTo = nan(Npoints,Npoints);

% Make a matrix that is all true values. We use this to trigger the points
% that are NOT the source point as temporary goalPoints.
allTrue = true(Npoints,1);

for ith_point = 1:Npoints
    % For each source point, find the costs to all the other points. For
    % the self-to-self cost, we'll treat this as infinite to avoid pushing
    % points to loop back onto themselves

    startPoint = allPoints(ith_point,:);
    notThisPoint = allTrue;
    notThisPoint(ith_point,1) = 0;
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

    % Call function
    [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY,...
        (startPoint), (flagWindRoundingType), (cellArrayOfWindExitConditions), (-1));

    if 4~=exitCondition
        warning('Not all goal points are reachable from other goal points');
    end
    reachableFlags = cellArrayOfExitInfo{2};
    costsFromTo(ith_point,tempGoalPointIDs) = reachableFlags';
    costsFromTo(ith_point,ith_point) = -1; % Self costs are -1

    if flag_do_debug
        figure(debug_figNum);

        thisColor = colorOrder(ith_point,:);
        reachableIndices = tempGoalPointIDs(reachableFlags>0);
        feasibleGoalPoints = allPoints(reachableIndices,:); 

        % Plot the reachableSet output
        plot(reachableSet(:,1),reachableSet(:,2),'LineWidth',3,'Color',thisColor,'DisplayName','Output: reachableSet')

        % Plot the feasible goal points
        plot(feasibleGoalPoints(:,1),feasibleGoalPoints(:,2),'o','Color',thisColor,'MarkerSize',5+5*ith_point,'LineWidth', 2, 'DisplayName','feasibleGoalPoints');

    end

end

%%%%%
% Remove infeasible points
feasibleCostsFromTo = costsFromTo;
if any(isnan(costsFromTo),'all')
    notReachable = isnan(costsFromTo);
    rowSum = sum(notReachable,1);
    badGoals = find(rowSum==(Npoints-1));
    NbadGoals = length(badGoals);
    pointsToKeep = allTrue;
    for ith_bad = 1:NbadGoals
        % Work from outside inward, since we're removing rows and columns
        thisBad = badGoals(NbadGoals-ith_bad+1);
        pointsToKeep(thisBad) = false;

        % Delete rows/columns
        feasibleCostsFromTo(thisBad,:) = [];
        feasibleCostsFromTo(:,thisBad) = [];

    end
    feasibleAllPoints = allPoints(pointsToKeep,:);
    feasiblePointNumbers = pointNumbers(pointsToKeep,:);
    notFeasiblePointNumbers = pointNumbers(~pointsToKeep,:);
    
    if ~any(1==feasiblePointNumbers,'all')
        warning('on','backtrace');
        warning('Catastrophic error detected where start point is not reachable from goal points');
        error('The start/end point is not reachable!');
    end

    if NbadGoals>0
        fprintf(1,'The following goal points were found as infeasible:\n');
        for ith_point = 1:length(notFeasiblePointNumbers)
            thisPointIndex = notFeasiblePointNumbers(ith_point);
            fprintf(1,'\tPoint %.0d, with [X Y] of: %.2f %.2f\n',thisPointIndex,allPoints(thisPointIndex,1), allPoints(thisPointIndex,2));
        end
    end

end

NgoodGoals = length(feasiblePointNumbers(:,1));


%%%%%
% Run a "greedy" algorithm to estimate number of steps to simulate
accumulatedCosts = nan(NgoodGoals+1,1);
flagsCityWasVisited = zeros(NgoodGoals,1);
visitSequence = nan(NgoodGoals+1,1);

% Initialize values
visitSequence(1,1) = 1;
flagsCityWasVisited(1,1) = 1;
accumulatedCosts(1,1) = 0;


for ith_visit = 2:NgoodGoals
    previousCity = visitSequence(ith_visit-1,1);
    unvisitedCities = find(flagsCityWasVisited==0);
    previousCosts = feasibleCostsFromTo(previousCity,:);
    unvisitedCosts = previousCosts(:,unvisitedCities);
    [minCost,indexMin] = min(unvisitedCosts);
    accumulatedCosts(ith_visit,1) = accumulatedCosts(ith_visit-1,1)+minCost;
    thisCity = unvisitedCities(indexMin);
    visitSequence(ith_visit,1) = thisCity;
    flagsCityWasVisited(thisCity,1) = 1;
end

% Connect back to start
visitSequence(end,:) = 1;
accumulatedCosts(end,1) = accumulatedCosts(end-1,1) + feasibleCostsFromTo(thisCity,1);

URHERE - why is accumulated cost here lower than best TSP solution later?

maximumSimLength = accumulatedCosts(end,1);

% Plot the greedy result
if flag_do_debug
    figure(debug_figNum);

    pointSequence = feasibleAllPoints(visitSequence,:);
    % Plot the ordered visit sequence
    for ith_point = 1:NgoodGoals
        arrowMagnitude = pointSequence(ith_point+1,:)-pointSequence(ith_point,:);
        quiver(pointSequence(ith_point,1),pointSequence(ith_point,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
            'LineWidth',3);
    end
end

%%%%
% Run the TSP solver
% The maximum problem size is the max number of sim steps times the number
% of cities, e.g. a sim starting for every city, at every time step
maxSolutions = maximumSimLength*NgoodGoals;

% Solutions have the form:
% 1x1 (accumulated cost) Ngx1 (flags city was visited)  Ngx1 (visit sequence) 1x1 (flagHeadingHome) 
solutions = nan(maxSolutions,2+NgoodGoals*2);

% Initialize values
currentBestExpansionSolution = 1;
currentBestCity = 1;
solutions(currentBestExpansionSolution,1) = 0;
solutions(currentBestExpansionSolution,1+NgoodGoals+1) = currentBestCity;

flagKeepGoing = 1;
% Set an upper bound on allowable searches. Once a viable solution is
% found, it sets an upper cost limit. There's no reason to keep branches in
% the search that are higher than this limit. This costCropLimit gets
% updated as new solutions found.
costCropLimit = inf; 

while 1==flagKeepGoing

    % Pull out all the details from this solutions row
    previousCost                = solutions(currentBestExpansionSolution,1);
    previousFlagsCityWasVisited = solutions(currentBestExpansionSolution,2:1+NgoodGoals);
    previousVisitSequence       = solutions(currentBestExpansionSolution,2+NgoodGoals:(1+2*NgoodGoals));
    flagHeadingHome             = solutions(currentBestExpansionSolution,end);

    visitSequence = previousVisitSequence(~isnan(previousVisitSequence));
    currentBestCity = visitSequence(end);
    
    % Update the previous city's flag to indicate that city was visited
    previousFlagsCityWasVisited(currentBestCity) = 1; % Flag update

    % Find which cities were not visited
    unvisitedCities = find(isnan(previousFlagsCityWasVisited));
    Nunvisited = length(unvisitedCities);
    Nvisited   = NgoodGoals-Nunvisited;

    % Find the visit list. This is the list of cities that have been
    % visited thus far
    if length(visitSequence)~=Nvisited
        error('Discrepancy found in TSP solver');
    end

    if Nunvisited==0 && 1==flagHeadingHome
        flagKeepGoing = 0;
        % This solution is the best one as it has minimum time, and
        % completes all the circuit
    else
        if Nunvisited==0
            flagHeadingHome = 1;
            % Force the costs to be calculated to go back to "home"
            unvisitedCities = 1;
            Nunvisited = 1; % Need to update this so that the repmat operation that follows works.

        end
        % Pull costs to go from previous city to all the next ones
        costsPreviousCityToTheseCities = feasibleCostsFromTo(currentBestCity,:);
        unvisitedCosts = previousCost + costsPreviousCityToTheseCities(:,unvisitedCities);

        if flagHeadingHome==1
            % Update the cost crop limit
            costCropLimit = min(costCropLimit,unvisitedCosts);
        end

        % Convert the cost search options into rows to add to solutionRows
        % queue. Start by creating a "copy" of all the current city
        solutionRows = repmat([nan previousFlagsCityWasVisited previousVisitSequence flagHeadingHome], Nunvisited,1);

        % Fill in the costs for the cities that can be visited
        solutionRows(:,1) = unvisitedCosts';
        
        % Set the visit sequence to indicate which cities were added. NOTE:
        % for the last "home" city, this will put 1 into the last column
        solutionRows(:,1+NgoodGoals+Nvisited+1) = unvisitedCities';

        % Push results into solutions for searching
        rowStartToFill = find(isnan(solutions(:,1)),1);
        rowEndToFill   = rowStartToFill+Nunvisited-1;
        solutions(rowStartToFill:rowEndToFill,:) = solutionRows;

        % Remove the row that was just searched
        solutions(currentBestExpansionSolution,:) = [];

        % Remove any queued solutions higher than the current completed
        % cost
        if ~isinf(costCropLimit)
            solutionRowsToRemove = solutions(:,1)>costCropLimit;
            solutions(solutionRowsToRemove,:) = [];
        end

        
        % Find next best city
        [~,currentBestExpansionSolution] = min(solutions(:,1));
    end
end

orderedVisitSequence = [visitSequence'; 1];

% Plot the TSP result
if flag_do_debug
    figure(debug_figNum);

    pointSequence = feasibleAllPoints(orderedVisitSequence,:);
    % Plot the ordered visit sequence
    for ith_point = 1:NgoodGoals
        arrowMagnitude = pointSequence(ith_point+1,:)-pointSequence(ith_point,:);
        quiver(pointSequence(ith_point,1),pointSequence(ith_point,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
            'LineWidth',5,'Color',[0 1 0]);
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

    URHERE
    % Prep the figure for plotting
    temp_h = figure(figNum);
    flag_rescale_axis = 0;
    if isempty(get(temp_h,'Children'))
        flag_rescale_axis = 1;
    end      
    
    % Is this 2D or 3D?
    dimension_of_points = 2; %#ok<NASGU>

    % Find size of plotting domain
    allPointsBeingPlotted = [reachableSet; nan nan];

    max_plotValues = max(allPointsBeingPlotted);
    min_plotValues = min(allPointsBeingPlotted);
    sizePlot = max(max_plotValues) - min(min_plotValues);
    nudge = sizePlot*0.006; %#ok<NASGU>

    % Find size of plotting domain
    if flag_rescale_axis
        % NO NEED TO RESIZE THE AXIS FOR IMAGE PLOTTING
        % percent_larger = 0.3;
        % axis_range = max_plotValues - min_plotValues;
        % if (0==axis_range(1,1))
        %     axis_range(1,1) = 2/percent_larger;
        % end
        % if (0==axis_range(1,2))
        %     axis_range(1,2) = 2/percent_larger;
        % end
        % if dimension_of_points==3 && (0==axis_range(1,3))
        %     axis_range(1,3) = 2/percent_larger;
        % end
        % 
        % % Force the axis to be equal?
        % if 1==1
        %     min_valuesInPlot = min(min_plotValues);
        %     max_valuesInPlot = max(max_plotValues);
        % else
        %     min_valuesInPlot = min_plotValues;
        %     max_valuesInPlot = max_plotValues;
        % end
        % 
        % % Stretch the axes
        % stretched_min_vertexValues = min_valuesInPlot - percent_larger.*axis_range;
        % stretched_max_vertexValues = max_valuesInPlot + percent_larger.*axis_range;
        % axesTogether = [stretched_min_vertexValues; stretched_max_vertexValues];
        % newAxis = reshape(axesTogether, 1, []);
        % axis(newAxis);

    end
    % goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end   

    % Turn on legend
    legend('Interpreter','none','Location','best');

    % Plot the windfield as an image
    cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    set(cellArrayOfPlotHandles{3},'Color',[0.6 0.6 0.6]);

    % Plot the start point
    plot(startPoint(:,1),startPoint(:,2),'.-','Color',[0 0 1],'MarkerSize',30,'DisplayName','Input: startPoint');

    % Plot the goal points
    plot(originalGoalPoints(:,1),originalGoalPoints(:,2),'.-','Color',[0 1 0],'MarkerSize',20,'LineWidth', 2, 'DisplayName','Input: goalPoints');

    % Plot the feasible goal points
    if ~isempty(goalPoints)
        plot(goalPoints(:,1),goalPoints(:,2),'.','Color',[1 0 1],'MarkerSize',40,'LineWidth', 2, 'DisplayName','Input: allGoalPointsList');
    end

    % Plot the expansion sets
    % allColors = parula(Nsteps+1);
    allColors = turbo;
    Ntotal = 25;
    for ith_expansion = 1:cellArrayOfExitInfo{1}
        thisExpansion = allExpansions{ith_expansion,1};
        percentageDone = min(ith_expansion/Ntotal,1);
        if 1==0
            % Plot in color
            colorNumber = min(256,max(round(255*percentageDone)+1,1));
            thisColor = allColors(colorNumber,:);
        else
            % Plot in white to black
            thisColor = (1-percentageDone)*[1 1 1];
        end

        plot(thisExpansion(:,1),thisExpansion(:,2),'-',...
            'Color',thisColor,'MarkerSize',30, 'LineWidth', 0.5, 'DisplayName',sprintf('Expansion: %.0f',ith_expansion),'HandleVisibility','off');
        pause(0.1);
    end

    % Plot the final output
    plot(finalReachableSet(:,1),finalReachableSet(:,2),'LineWidth',3,'Color',[1 0 0],'DisplayName','Output: reachableSet')

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



%% fcn_INTERNAL_sparsifyPoints
function sparsePoints = fcn_INTERNAL_sparsifyPoints(densePoints,deltaX)

% Make sure first and last point are repeated, e.g. that the plot is a
% closed circuit
if ~isequal(densePoints(1,:),densePoints(end,:))
    densePoints(end,:) = densePoints(1,:);
end

segmentVectors = densePoints(2:end,:) - densePoints(1:end-1,:);
segmentLengths = sum(segmentVectors.^2,2).^0.5;

Npoints = length(densePoints(:,1));
currentPoint = 1;
currentDistance = 0;
sparsePointIndices = false(Npoints,1);
while currentPoint<Npoints
    currentPoint = currentPoint+1;
    currentDistance  = currentDistance + segmentLengths(currentPoint-1);

    % Did we "travel" farther than expected deltaX? If so, mark this point
    % so that it is kept.
    if currentDistance>=deltaX
        sparsePointIndices(currentPoint,1) = true;
        currentDistance = 0;
    end
end
sparsePoints = densePoints(sparsePointIndices,:);

% Make sure to close off the points
if ~isequal(sparsePoints(end,:),sparsePoints(1,:))
    sparsePoints = [sparsePoints; sparsePoints(1,:)];
end

if 1==0
    figure(388383);
    clf;
    plot(densePoints(:,1),densePoints(:,2),'.-','MarkerSize',30,'LineWidth',3,'DisplayName','Input: densePoints');
    hold on;
    axis equal
    plot(sparsePoints(:,1),sparsePoints(:,2),'.-','MarkerSize',10,'LineWidth',1,'DisplayName','Output: sparsePoints');
end


end % Ends fcn_INTERNAL_sparsifyPoints

%% fcn_INTERNAL_findGoalPointsHit
function goalPointsHit = fcn_INTERNAL_findGoalPointsHit(newStartPoints,allGoalPointsList)
if ~isempty(allGoalPointsList)
    uniquePoints = flipud(newStartPoints); % for some silly reason, polyshape takes points "backwards" (?!)
    uniquePoints = unique(uniquePoints,'rows','stable');
    region = polyshape(uniquePoints(1:end-1,:),'KeepCollinearPoints', true,'Simplify', false);
    goalPointsHit = isinterior(region,allGoalPointsList);
end
end % Ends fcn_INTERNAL_findGoalPointsHit