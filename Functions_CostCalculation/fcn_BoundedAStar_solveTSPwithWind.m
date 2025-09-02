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
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * first write of function using fcn_BoundedAStar_solveTSPwithWind
%   %   % as a starter
%   % * got TSP solution intialization working with greedy method, which
%   %   % produces a quick upper bound estimate on sim time
%
% 2025_08_06 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * TSP working with Djkstra's method. Minor bug found
%
% 2025_08_07 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * TSP working with Djkstra's method. Prior bug fixed. 
%   % * Added plotting of results
%
% 2025_08_11 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * TSP working with Djkstra's method. Prior bug fixed. 
%   % * Added plotting of results
% 
% 2025_08_27 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * functionalized debug input plotting: fcn_INTERNAL_debugPlotInputs

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

%172652 iterations with 10 goalpoints
% Ideas:
% 1) Preallocate array
% 2) Batch sort?
% 3) Remove infeasible points based on minimum point-to-point path costs
% 4) 

% Save the original goal points, in case some are infeasible. This helps
% with debugging later
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
    for ith_fromPoint = 1:length(pointsMissing)
        thisPointIndex = pointsMissing(ith_fromPoint);
        fprintf(1,'\tPoint %.0d, with [X Y] of: %.2f %.2f\n',thisPointIndex-1,allPointsOriginal(thisPointIndex,1),allPointsOriginal(thisPointIndex,2));
    end
    warning('on','backtrace');
    warning('Some goal points were repeated. Repeats listed above have been deleted, but erroneous results may occur.');
end

%%%%
% Number all the points (now unique)
NUniqueGoals = length(allPoints(:,1));
pointNumbers = (1:NUniqueGoals)';
if flag_do_debug
    figure(debug_figNum);
    % Number the unique points
    for ith_fromPoint = 1:NUniqueGoals
        text(allPoints(ith_fromPoint,1)+0.2,allPoints(ith_fromPoint,2),sprintf('%.0f',pointNumbers(ith_fromPoint)));
    end
end

%%%%
% For each point, find costs to go from that point to the others
% Costs are saved as "from" as rows, and "to" as columns. Note: self costs
% are -1. Costs that are not possible (infeasible) are NaN.
[costsFromTo, pathsFromTo, reachableSet] = fcn_INTERNAL_findCostsFromTo(...
    NUniqueGoals, allPoints, pointNumbers, Nsteps,...
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
    feasibleAllPoints = allPoints(pointsToKeep,:);
    feasiblePointNumbers = pointNumbers(pointsToKeep,:);
    notFeasiblePointNumbers = pointNumbers(~pointsToKeep,:);

    if ~any(1==feasiblePointNumbers,'all')
        warning('on','backtrace');
        warning('Catastrophic error detected where start point is not reachable from goal points');
        error('The start/end point is not reachable!');
    end

    NbadGoals = length(notFeasiblePointNumbers);
    if NbadGoals>0
        fprintf(1,'The following goal points were found as infeasible:\n');
        for ith_fromPoint = 1:NbadGoals
            thisPointIndex = notFeasiblePointNumbers(ith_fromPoint);
            fprintf(1,'\tPoint %.0d, with [X Y] of: %.2f %.2f\n',thisPointIndex,allPoints(thisPointIndex,1), allPoints(thisPointIndex,2));
        end
    end
else
    % All points are feasible
    feasibleAllPoints = allPoints;
end

NFeasibleGoals = size(feasibleCostsFromTo,1);

% Save the paths to the feasible goal points
pathsFromToFeasible = cell(NFeasibleGoals,NFeasibleGoals);
for ith_row = 1:NFeasibleGoals
    for jth_col = 1:NFeasibleGoals
        sourceRow = previousRows(ith_row,jth_col);
        sourceCol = previousCols(ith_row,jth_col);
        pathsFromToFeasible{ith_row, jth_col} = pathsFromTo{sourceRow, sourceCol};
    end
end

%%%%
% Find the minimum "to" costs. This is the absolute minimum possible, and
% usually infeasible, cost to get to a goal regardless of the "from"
% origin. For example, if goal 3 and 7 remain, and the minimum cost to get
% to each respectively is 15 and 6, then there is a minimum of 21 more
% "cost" to fill these missing cities in. This minimum cost value is VERY
% useful in later steps to eliminate bad options. To continue this example,
% if the minimum all-city circuit cost discovered so far is 100, and an
% option being examined has looked at all cities except 3 and 7, and
% currently has a cost of 85, there's no possible way that the addition of
% 3 and 7 as paths will better than the best cost found so far. So this
% option can be deleted without even checking.

% To find minimum costs, set all -1 costs in costsFromTo to large value
% (inf). Then find minimum along each column.
resetCosts = feasibleCostsFromTo;
negativeOneIndices = feasibleCostsFromTo==-1;
resetCosts(negativeOneIndices) = inf;
minimumToCosts = min(resetCosts,[],1);
minimumToCostsTranspose = minimumToCosts';
minimumPossibleTrajectoryCost = sum(minimumToCosts);

fprintf(1,'Absolute minimum possible cost: %.2f\n',minimumPossibleTrajectoryCost)

%%%%%
% Run a "greedy" algorithm to estimate number of steps to simulate
accumulatedCosts = nan(NFeasibleGoals+1,1);
flagsCityWasVisited = zeros(NFeasibleGoals,1);
visitSequence = nan(NFeasibleGoals+1,1);

% Initialize values
visitSequence(1,1) = 1;
flagsCityWasVisited(1,1) = 1;
accumulatedCosts(1,1) = 0;


for ith_visit = 2:NFeasibleGoals
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

% TO DO: use this to crop solutions
greedySearchSimLength = accumulatedCosts(end,1);

fprintf(1,'Greedy search result: %.2f\n',greedySearchSimLength);

% Plot the greedy result
if flag_do_debug
    figure(debug_figNum);

    % pointSequence = feasibleAllPoints(visitSequence,:);
    % Plot the ordered visit sequence
    colorOrder = get(gca, 'ColorOrder');
    Ncolors = length(colorOrder(:,1));

    for ith_fromPoint = 1:length(visitSequence)-1
        thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        from_index = visitSequence(ith_fromPoint,1);
        goal_index = visitSequence(ith_fromPoint+1,1);
        try
            thisPath =  pathsFromToFeasible{from_index,goal_index};
        catch
            disp('Stop here');
        end
        plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth',5);
        % arrowMagnitude = pointSequence(ith_point+1,:)-pointSequence(ith_point,:);
        % quiver(pointSequence(ith_point,1),pointSequence(ith_point,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
        %     'LineWidth',3);

    end
end

%%%%
% What is the average cost per destination? This is useful to know how
% selective the path is relative to greedy.
resetCosts = feasibleCostsFromTo;
negativeOneIndices = feasibleCostsFromTo==-1;
resetCosts(negativeOneIndices) = nan;
meanCostsEachDestination = mean(resetCosts,1,'omitmissing');
meanTrajectoryCost = sum(meanCostsEachDestination);
fprintf(1,'Mean (randomized) search result: %.2f\n',meanTrajectoryCost);


%%%%
% Run the TSP solver

% BATCH ideas:
% Batch first by number of cities visited, then by current best city.
% Total batches? Ngoals (visited) * Ngoals (best cities at this stage)
% Pros? 
% * the operations expand by "best city", so this calculation doesn't need
%   to be repeated and solution steps "done" are easy to prune
% * the depth of search is clear in the process, e.g. one step deeper each
%   time
% * the remaining cost prune might be easier, e.g. prune all branches at
%   once?
% Cons? each best city option expansion will be different

% The maximum problem size is the max number of sim steps times the number
% of cities, e.g. a sim starting for every city, at every time step
maxSims = ceil(greedySearchSimLength)*NFeasibleGoals;
fprintf(1,'Maximum number of simulations needed: %.d\n',maxSims);
solutionBatchSize = 100000;

% Solutions have the form:
% 1x1 (accumulated cost) 1x1 (minimum possible total cost) Ngx1 (flags city was visited)  Ngx1 (visit sequence) 1x1 (flagHeadingHome) 
accumulatedCosts = nan(solutionBatchSize,1);
minPossibleTotalCosts = ones(solutionBatchSize,1)*minimumPossibleTrajectoryCost;
flagsCityWasVisited = nan(solutionBatchSize,NFeasibleGoals);
visitSequence = nan(solutionBatchSize,NFeasibleGoals);
flagHeadingHome = nan(solutionBatchSize,1);

stagedTests = [accumulatedCosts, minPossibleTotalCosts, flagsCityWasVisited, visitSequence, flagHeadingHome];

% Initialize values
currentBestExpansionSolution = 1;
currentFromCity = 1;
stagedTests(currentBestExpansionSolution,1) = 0;
stagedTests(currentBestExpansionSolution,2+NFeasibleGoals+1) = currentFromCity;

flagKeepGoing = 1;
% Set an upper bound on allowable searches. Once a viable solution is
% found, it sets an upper cost limit. There's no reason to keep branches in
% the search that are higher than this limit. This costCropLimit gets
% updated as new solutions found. The first limit is the one found by the
% greedy search.
costCropLimit = greedySearchSimLength; 

iteration_number = 0;
while 1==flagKeepGoing
    iteration_number = iteration_number+1;

    % if iteration_number==11
    %     disp('Stop here');
    % end

    % Pull out all the details from this solutions row
    previousCost                = stagedTests(currentBestExpansionSolution,1);
    previousMinPossibleTotalCosts = stagedTests(currentBestExpansionSolution,2);
    previousFlagsCityWasVisited = stagedTests(currentBestExpansionSolution,3:2+NFeasibleGoals);
    previousVisitSequence       = stagedTests(currentBestExpansionSolution,3+NFeasibleGoals:(2+2*NFeasibleGoals));
    flagHeadingHome             = stagedTests(currentBestExpansionSolution,end);

    fprintf(1, '%.0f: Previous cost: %.2f of min possible total: %.2f, best so far: %.2f\n', iteration_number, previousCost, previousMinPossibleTotalCosts, costCropLimit);

    % Pull out the visit sequence. This is the ordering of cities that were
    % previously visited on this cycle. The last city in the sequence was
    % the one that is the "currentBestCity", e.g. the city that needs to be
    % expanded as the "from" city. The remaining unvisited cities are the
    % ones which will be staged as the subsequent branches.
    visitSequence = previousVisitSequence(~isnan(previousVisitSequence));
    currentFromCity = visitSequence(end);
    
    % Update the previous city's flag to indicate that city was visited
    previousFlagsCityWasVisited(currentFromCity) = 1; % Flag update

    % Find which cities were not visited
    unvisitedCities = find(isnan(previousFlagsCityWasVisited));
    NumUnvisited = length(unvisitedCities);
    NumVisited   = NFeasibleGoals-NumUnvisited;

    % Find the visit list. This is the list of cities that have been
    % visited thus far
    if length(visitSequence)~=NumVisited
        error('Discrepancy found in TSP solver');
    end

    if NumUnvisited==0 && 1==flagHeadingHome
        flagKeepGoing = 0;
        % This solution is the best one as it has minimum time, and
        % completes all the circuit
    else
        % Find costs for unvisited cities. This is done by building staged
        % "solution rows", where each row represents a branch that needs to
        % be tested in subsequent iterations.

        % Find out how many branches are needed. Usually, this is the
        % number of unvisited cities. But in the case that all the cities
        % have been visited, need to add one more branch to go back to the
        % home city. Check to see if all the cities have been visited
        % except the "home" city (city 1). If they have, then indicate that
        % we need to head home
        if NumUnvisited==0
            flagHeadingHome = 1;
            % Force the costs to be calculated to go back to "home"
            unvisitedCities = 1;
            NumBranches = 1; % Set to 1 so that the repmat operation that follows works.
        else
            flagHeadingHome = 0;
            NumBranches = NumUnvisited;
        end
        branchesFlagHeadingHome = ones(NumBranches,1)*flagHeadingHome;

        % Pull costs to go from previous city to all the next ones
        costsPreviousCityToTheseCities = feasibleCostsFromTo(currentFromCity,:);
        branchesAccumulatedCosts = previousCost + costsPreviousCityToTheseCities(:,unvisitedCities);

        % Check to see if this is a return-to-home calculation. If so, the
        % TSP has been solved. This may now be the best TSP cost, so compare
        % this current cost to the best one so far to see if it is better.
        if flagHeadingHome==1
            % Update the cost crop limit
            costCropLimit = min(costCropLimit,branchesAccumulatedCosts);
        end

        
        % Update the flags on cities that were visited. This is simply a
        % copy into the rows of the previousFlagsCityWasVisited, which was
        % updated earlier
        branchesFlagsCityWasVisited = repmat(previousFlagsCityWasVisited, NumBranches,1);


        % Update the branchesPreviousVisitSequence
        % Need to set the visit sequence to indicate which cities were
        % added. NOTE: for the last "home" city, this will put 1 into the
        % last column
        branchesVisitSequence = repmat(previousVisitSequence, NumBranches,1);
        if 0==flagHeadingHome
            branchesVisitSequence(:,NumVisited+1) = unvisitedCities';
        end   
        
        % Update the minimum remaining costs
        Nvisits = find(~isnan(branchesVisitSequence(1,:)),1,'last');
        visitedCityList = branchesVisitSequence(:,2:Nvisits);
        minimumCostsVisitedSoFar = minimumToCostsTranspose(visitedCityList);
        if 1==NumBranches
            sumCostsVisitedSoFar = sum(minimumCostsVisitedSoFar,'all');
        else
            sumCostsVisitedSoFar = sum(minimumCostsVisitedSoFar,2);
        end
        branchesMinPossibleTotalCosts = branchesAccumulatedCosts' + minimumPossibleTrajectoryCost*ones(NumBranches,1) - sumCostsVisitedSoFar;

        % Convert the cost search options into rows to add to solutionRows
        % queue. 
        % stagedTests = [accumulatedCosts, minPossibleTotalCosts, flagsCityWasVisited, visitSequence, flagHeadingHome];
        solutionRows = [...
            branchesAccumulatedCosts', ...
            branchesMinPossibleTotalCosts, ...
            branchesFlagsCityWasVisited, ...
            branchesVisitSequence, ...
            branchesFlagHeadingHome];
        
        % Remove any solution rows that are not feasible for later
        % exploration. These will have NaN costs
        solutionRows(isnan(solutionRows(:,1)),:) = [];

        % Push results into solutions for searching
        NsolutionRows = length(solutionRows(:,1));
        rowStartToFill = find(~isnan(stagedTests(:,1)),1,'last')+1;
        rowEndToFill   = rowStartToFill+NsolutionRows-1;

        % Copy rows into stagedTests
        stagedTests(rowStartToFill:rowEndToFill,:) = solutionRows;

        
        % Remove the row that was just searched
        stagedTests(currentBestExpansionSolution,:) = [];

        % Remove any queued solutions higher than the current completed
        % cost
        if ~isinf(costCropLimit)
            solutionRowsToRemove = stagedTests(:,1)>costCropLimit;
            stagedTests(solutionRowsToRemove,:) = [];
        end

        
        % Find next best city
        [~,currentBestExpansionSolution] = min(stagedTests(:,1));
    end
end

orderedVisitSequence = [visitSequence'; 1];

% Plot the TSP result
if flag_do_debug
    figure(debug_figNum);

    % pointSequence = feasibleAllPoints(orderedVisitSequence,:);
    % Plot the TSP result, the ordered visit sequence

    for ith_fromPoint = 1:length(orderedVisitSequence)-1
        thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        from_index = orderedVisitSequence(ith_fromPoint,1);
        goal_index = orderedVisitSequence(ith_fromPoint+1,1);
        thisPath =  pathsFromToFeasible{from_index,goal_index};
        plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth',10);

        % for ith_point = 1:NgoodGoals
        %     arrowMagnitude = pointSequence(ith_point+1,:)-pointSequence(ith_point,:);
        %     quiver(pointSequence(ith_point,1),pointSequence(ith_point,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
        %         'LineWidth',5,'Color',[0 1 0]);
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
    legend('Interpreter','none','Location','best');

    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    % Get meshgrid for streamline plotting
    [meshX,meshY] = meshgrid(windFieldX,windFieldY);
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6],'HandleVisibility','off')

    colorOrder = get(gca, 'ColorOrder');
    Ncolors = length(colorOrder(:,1));

    % Plot the start point
    plot(startPoint(:,1),startPoint(:,2),'.','Color',colorOrder(1,:),'MarkerSize',30,'DisplayName','Input: startPoint');

    % Plot the goal points in different colors
    for ith_fromPoint = 1:length(originalGoalPoints(:,1))
        thisColorRow = mod(ith_fromPoint,Ncolors)+1;
        h_plot = plot(originalGoalPoints(ith_fromPoint,1),originalGoalPoints(ith_fromPoint,2),'*',...
            'Color',colorOrder(thisColorRow,:),'MarkerSize',30,'LineWidth', 2);
        if ith_fromPoint ==1
            set(h_plot, 'DisplayName','Input: goalPoints');
        else
            set(h_plot,'HandleVisibility','off');
        end
    end

    % Circle the feasible points
    plot(feasibleAllPoints(:,1),feasibleAllPoints(:,2),'o','Color',[0 1 0],'LineWidth',3,'MarkerSize',15,'DisplayName','feasibleAllPoints');

    % Plot the TSP result, the ordered visit sequence
    % pointSequence = feasibleAllPoints(orderedVisitSequence,:);

    for ith_fromPoint = 1:length(orderedVisitSequence)-1
        thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        from_index = orderedVisitSequence(ith_fromPoint,1);
        goal_index = orderedVisitSequence(ith_fromPoint+1,1);
        thisPath =  pathsFromToFeasible{from_index,goal_index};
        h_plot = plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth',5);

        % Plot the base in green and head in red so we know directions
        plot(thisPath(1:2,1),thisPath(1:2,2),'g-','LineWidth',5,'HandleVisibility','off');
        plot(thisPath(end-1:end,1),thisPath(end-1:end,2),'r-','LineWidth',5,'HandleVisibility','off');
        % h_quiver = quiver(thisPath(end-1,1),thisPath(end-1,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
        %     'LineWidth',5,'Color',thisColor,'HandleVisibility','off','ShowArrowHead','on',...
        %     'MaxHeadSize',4,'AutoScale','off','AutoScaleFactor',20);

        if ith_fromPoint==1
            set(h_plot,'DisplayName','Output: TSP solution');
        else
            set(h_plot,'HandleVisibility','off');
        end
    end

    % Number the unique points
    for ith_fromPoint = 1:NUniqueGoals
        text(allPoints(ith_fromPoint,1)+nudge,allPoints(ith_fromPoint,2),sprintf('%.0f',pointNumbers(ith_fromPoint)));
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
        save('BUG_90002_fcn_BoundedAStar_expandReachabilityWithWind.mat',...
            'radius', 'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY', ...
            'startPoint', 'flagWindRoundingType','cellArrayOfWindExitConditions');
    end

    % Call function to expand outward into wind field
    % figure(222222);
    % clf;
    [reachableSet, exitCondition, cellArrayOfExitInfo, ...
        reachableSetExactCosts, cellArrayOfReachableSetPaths] = fcn_BoundedAStar_expandReachabilityWithWind(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY,...
        (startPoint), (flagWindRoundingType), (cellArrayOfWindExitConditions), [], (-1));

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