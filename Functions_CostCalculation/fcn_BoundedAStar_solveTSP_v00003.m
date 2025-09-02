function [orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, varargin)
% fcn_BoundedAStar_solveTSPwithWind solves the Traveling Salesman Problem
%
% FORMAT:
% [orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
%     startAndGoalPoints, costsFromTo, pathsFromTo, (figNum));
%
% INPUTS:
%
%     startAndGoalPoints: an Mx2 array of the "city" points, with no
%     repeats. Each must be reachable from at least one other point. First
%     point in the list is startPoint, points in rows 2+ are goalPoints.
% 
%     costsFromTo: an MxM matrix specifying the costs to traverse
%     from a point (row) to another point (column). 
% 
%     pathsFromTo: an MxM cell array of XYUV points that lead from
%     a row point to a column point.
%
%     (optional inputs)
%
%     cellArrayOfFunctionOptions: allows the user to specify "deep" search
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
% This function was written on 2025_08_29 by S. Brennan
% Questions or comments? contact S. Brennan sbrennan@psu.edu 
% or K. Hayes, kxh1031@psu.edu

% REVISION HISTORY:
% 2025_08_29 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * first write of function using fcn_BoundedAStar_solveTSPwithWind
%   %   % as a starter

% TO-DO
% (none)


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 5; % The largest Number of argument inputs to the function
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
        narginchk(3,MAX_NARGIN);

        % Check the startAndGoalPoints input, make sure it is Mx2
        fcn_DebugTools_checkInputsToFunctions(...
            startAndGoalPoints, '2column_of_numbers',[2 3]);

        % Check the costsFromTo input, make sure it is MxM
        M = size(startAndGoalPoints,1);
        assert(isnumeric(costsFromTo));
        assert(isequal(size(costsFromTo),[M M]));

        % Check the pathsFromTo input, make sure it is MxM
        assert(iscell(pathsFromTo));
        assert(isequal(size(pathsFromTo),[M M]));

    end
end

% Does user want to specify cellArrayOfFunctionOptions input?
cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = []; % windFieldU
cellArrayOfFunctionOptions{2} = []; % windFieldV
cellArrayOfFunctionOptions{3} = []; % windFieldX
cellArrayOfFunctionOptions{4} = []; % windFieldY
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        cellArrayOfFunctionOptions = temp;
    end
end
windFieldU = cellArrayOfFunctionOptions{1};
windFieldV = cellArrayOfFunctionOptions{2};
windFieldX = cellArrayOfFunctionOptions{3};
windFieldY = cellArrayOfFunctionOptions{4};

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

NumGoals   = size(startAndGoalPoints,1);
startPoint = startAndGoalPoints(1,:);
goalPoints = startAndGoalPoints(2:end,:);

% DEBUGGING: Plot the input situation: wind field, startPoint, and goalPoints
if flag_do_debug && ~isempty(windFieldU)
    fcn_INTERNAL_debugPlotInputs(windFieldU,windFieldV,windFieldX,windFieldY, ...
        startPoint, goalPoints, debug_figNum)
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
resetCosts = costsFromTo;
negativeOneIndices = costsFromTo==-1;
resetCosts(negativeOneIndices) = inf;
minimumToCosts = min(resetCosts,[],1);
minimumToCostsTranspose = minimumToCosts';
minimumPossibleTrajectoryCost = sum(minimumToCosts);

if flag_do_debug
    fprintf(1,'Absolute minimum possible cost: %.2f\n',minimumPossibleTrajectoryCost)
end

%%%%%
% Run a "greedy" algorithm to estimate number of steps to simulate
flagsCityWasVisited = zeros(NumGoals,1);
visitSequence = nan(NumGoals+1,1);

% Initialize values
visitSequence(1,1) = 1;
flagsCityWasVisited(1,1) = 1;
priorCost = 0;

%%%%%
% Perform Greedy search
% Used to find a starting value of best possible cost
[visitSequence, greedySearchCost] = fcn_INTERNAL_greedySearch(...
    costsFromTo, visitSequence, flagsCityWasVisited, priorCost, pathsFromTo, flag_do_debug, debug_figNum); %#ok<ASGLU>

%%%%
% What is the average cost per destination? This is useful to know how
% selective the path is relative to greedy.
resetCosts = costsFromTo;
negativeOneIndices = costsFromTo==-1;
resetCosts(negativeOneIndices) = nan;
meanCostsEachDestination = mean(resetCosts,1,'omitmissing');
meanTrajectoryCost = sum(meanCostsEachDestination);

if flag_do_debug
    fprintf(1,'Mean (randomized) search result: %.2f\n',meanTrajectoryCost);
end

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
maxSims = ceil(greedySearchCost)*NumGoals;

if flag_do_debug
    fprintf(1,'Maximum number of simulations needed: %.d\n',maxSims);
end

solutionBatchSize = 10000000;

% Solutions have the form:
% 1x1 (accumulated cost) 1x1 (minimum possible total cost) Ngx1 (flags city was visited)  Ngx1 (visit sequence) 1x1 (flagHeadingHome) 
accumulatedCosts = nan(solutionBatchSize,1);
minPossibleTotalCosts = ones(solutionBatchSize,1)*minimumPossibleTrajectoryCost;
flagsCityWasVisited = nan(solutionBatchSize,NumGoals);
visitSequence = nan(solutionBatchSize,NumGoals);
flagHeadingHome = nan(solutionBatchSize,1);
flagAlreadyTested = nan(solutionBatchSize,1);

stagedTests = [accumulatedCosts, minPossibleTotalCosts, flagsCityWasVisited, visitSequence, flagHeadingHome, flagAlreadyTested];

% Initialize values
rowsFilledSoFar = 1;
currentBestExpansionSolution = 1;
currentFromCity = 1;
stagedTests(currentBestExpansionSolution,1) = 0;
stagedTests(currentBestExpansionSolution,2+NumGoals+1) = currentFromCity;
stagedTests(currentBestExpansionSolution,end) = 1; % Removes it from future tests
NcolumnsStagedTests = size(stagedTests,2);

flagKeepGoing = 1;
% Set an upper bound on allowable searches. Once a viable solution is
% found, it sets an upper cost limit. There's no reason to keep branches in
% the search that are higher than this limit. This costCropLimit gets
% updated as new solutions found. The first limit is the one found by the
% greedy search.
costCropLimit = greedySearchCost; 

iteration_number = 0;
flagRestageByCosts = 0;
nextTriggeredPrintInterval = 5000;
nextTriggeredPrint = 1;
startTime = tic;
totalSolutionsTested = 0; 
while 1==flagKeepGoing
    iteration_number = iteration_number+1;
    if currentBestExpansionSolution>=nextTriggeredPrint
        fprintf(1,'Current expansion: %.0f\n',currentBestExpansionSolution);
        nextTriggeredPrint = nextTriggeredPrint + nextTriggeredPrintInterval; 
    end

    % if iteration_number==775
    %     disp('Stop here');
    % end

    % Pull out all the details from this solutions row
    previousCost                = stagedTests(currentBestExpansionSolution,1);
    previousMinPossibleTotalCosts = stagedTests(currentBestExpansionSolution,2);
    previousFlagsCityWasVisited = stagedTests(currentBestExpansionSolution,3:2+NumGoals);
    previousVisitSequence       = stagedTests(currentBestExpansionSolution,3+NumGoals:(2+2*NumGoals));
    flagHeadingHome             = stagedTests(currentBestExpansionSolution,end-1);

    if flag_do_debug
        fprintf(1, '%.0f: Previous cost: %.2f of min possible total: %.2f, best so far: %.2f\n', iteration_number, previousCost, previousMinPossibleTotalCosts, costCropLimit);
    end

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
    NumVisited   = NumGoals-NumUnvisited;

    % Find the visit list. This is the list of cities that have been
    % visited thus far
    if length(visitSequence)~=NumVisited
        error('Discrepancy found in TSP solver');
    end

    % Do we exit the loop? In other words, have all the cities been
    % visited, and we flagged that the cost to return home is also added?
    if NumUnvisited==0 && 1==flagHeadingHome
        flagKeepGoing = 0;
        bestCost = previousCost;
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
        branchesAlreadyTested   = zeros(NumBranches,1);

        % Pull costs to go from previous city to all the next ones
        costsPreviousCityToTheseCities = costsFromTo(currentFromCity,:);
        branchesAccumulatedCosts = previousCost + costsPreviousCityToTheseCities(:,unvisitedCities);

        % Check to see if this is a return-to-home calculation. If so, the
        % TSP has been solved. This may now be the best TSP cost, so compare
        % this current cost to the best one so far to see if it is better.
        if flagHeadingHome==1
            % This solution just solved the TSP. Is it better than before?
            if branchesAccumulatedCosts<costCropLimit
                oldCost = costCropLimit;
                costCropLimit = branchesAccumulatedCosts;
                flagRestageByCosts = 1;
            end
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
        if flagHeadingHome==0
            branchesMinPossibleTotalCosts = branchesAccumulatedCosts' + ...
                minimumPossibleTrajectoryCost*ones(NumBranches,1) - sumCostsVisitedSoFar;
        else
            branchesMinPossibleTotalCosts = branchesAccumulatedCosts;
        end

        % Convert the cost search options into rows to add to solutionRows
        % queue. 
        % stagedTests = [accumulatedCosts, minPossibleTotalCosts, flagsCityWasVisited, visitSequence, flagHeadingHome, flagAlreadyTested];
        solutionRows = [...
            branchesAccumulatedCosts', ...
            branchesMinPossibleTotalCosts, ...
            branchesFlagsCityWasVisited, ...
            branchesVisitSequence, ...
            branchesFlagHeadingHome, ...
            branchesAlreadyTested];
        
        % Remove any solution rows that are not feasible for later
        % exploration - these will have NaN costs. They are "removed" by
        % setting last column to 1. AS well, remove any rows whose current
        % or best possible cost is worse than the current best.
        % IDEA: do this first, and fill queued rows directly
        solutionRows(isnan(solutionRows(:,1)),end) = 1;
        solutionRows(solutionRows(:,1)>costCropLimit,end) = 1;
        solutionRows(solutionRows(:,2)>costCropLimit,end) = 1;

        % Push viable future tests into queue for future searching
        % indicesViable = find(solutionRows(:,end)==0);
        % NsolutionRows = length(indicesViable);

        NsolutionRows = length(solutionRows(:,1));
        
        rowStartToFill = rowsFilledSoFar+1;
        rowEndToFill   = rowStartToFill+NsolutionRows-1;
        rowsFilledSoFar = rowEndToFill;

        % Copy rows into stagedTests
        stagedTests(rowStartToFill:rowEndToFill,:) = solutionRows;

        
        % Remove the row that was just searched. Remove by setting last
        % column to 1.
        stagedTests(currentBestExpansionSolution,end) = 1;

        % If 1==flagRestageByCosts, this means that a new best possible
        % cost was found. In that case, there may be solutions in the queue
        % that are no longer viable. 
        % Remove any queued solutions higher than the current completed
        % cost. Remove by setting last column to 1.
        if 1==flagRestageByCosts || currentBestExpansionSolution>5000
            if flagRestageByCosts
                fprintf(1,'\tResetting costs. Old Cost: %.2f New cost: %.2f\n', oldCost, costCropLimit);
            else
                fprintf(1,'\tResorting tests. Old Cost: %.2f New cost: %.2f\n', costCropLimit, costCropLimit);
            end

            % A new cost just arrived. Shut off all stagedTests that exceed
            % this new cost.
            flagRestageByCosts = 0;
            solutionRowsToRemove = stagedTests(1:rowsFilledSoFar,1)>costCropLimit | stagedTests(1:rowsFilledSoFar,2)>costCropLimit;
            stagedTests(solutionRowsToRemove,end) = 1;

            % Sort the remaining?
            if 1==1
                viableIndices = stagedTests(:,NcolumnsStagedTests)==0;
                viableStagedTests = stagedTests(viableIndices,:);
                Nviable = size(viableStagedTests,1);

                % Sort viable ones by cost
                viableTestsSorted = sortrows(viableStagedTests,1);

                % Copy good values of remaining into stagedTests, after shutting off
                % all prior stagedTests

                % Shut off prior stagedTests
                stagedTests(1:rowsFilledSoFar,end) = 1;

                % Copy sorted viable into stagedTests
                stagedTests(1:Nviable,:) = viableTestsSorted;

                totalSolutionsTested = totalSolutionsTested + currentBestExpansionSolution;
                currentBestExpansionSolution = 1;
            end
        end

        
        % Find next best city
        if 1==1
            flagGoToNextCity = true;
            while flagGoToNextCity
                currentBestExpansionSolution = currentBestExpansionSolution+1;            
                if stagedTests(currentBestExpansionSolution,end)==0
                    flagGoToNextCity = false;
                elseif isnan(stagedTests(currentBestExpansionSolution,end))
                    error('Ran out of solutions without solving');
                elseif solutionBatchSize==currentBestExpansionSolution
                    error('Hit end without solving');
                end
            end
        else
            indicesToCheck = find(stagedTests(1:rowsFilledSoFar,end)==0);
            [~,minCostIndex] = min(stagedTests(indicesToCheck,1));
            currentBestExpansionSolution = indicesToCheck(minCostIndex);
        end
    end
end
duration = toc(startTime);
totalSolutionsTested = totalSolutionsTested + currentBestExpansionSolution;


orderedVisitSequence = [visitSequence'; 1];

% Confirm the cost
costCalculated = 0;
for ith_sequence = 2:length(orderedVisitSequence)
    fromIndex = orderedVisitSequence(ith_sequence-1);
    toIndex = orderedVisitSequence(ith_sequence);
    costCalculated = costCalculated + costsFromTo(fromIndex,toIndex);
end

if ~isequal(costCalculated, bestCost)
    warning('Discrepancy found between expected and actual traversal cost');
end

fprintf(1,'\nFINAL RESULTS:\n');
fprintf(1,'Number of cities (goals, no start): %.0f\n',length(orderedVisitSequence)-2);
fprintf(1,'Wall time: %.3f seconds\n',duration);
fprintf(1,'Solution sequence: \t')
for ith_sequence = 1:length(orderedVisitSequence)
    fprintf(1, '%.0f ', orderedVisitSequence(ith_sequence,1));
end
fprintf(1,'\n');
fprintf(1,'Number of main loop iterations: %.0f\n',iteration_number);
fprintf(1,'Number of staged tests checked: %.0f\n',totalSolutionsTested);
fprintf(1,'Number of staged tests: %.0f\n',rowsFilledSoFar);



% Plot the TSP result
if flag_do_debug
    figure(debug_figNum);

    colorOrder = get(gca, 'ColorOrder');
    Ncolors = length(colorOrder(:,1));

    % pointSequence = feasibleAllPoints(orderedVisitSequence,:);
    % Plot the TSP result, the ordered visit sequence

    for ith_fromPoint = 1:length(orderedVisitSequence)-1
        thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        from_index = orderedVisitSequence(ith_fromPoint,1);
        goal_index = orderedVisitSequence(ith_fromPoint+1,1);
        thisPath =  pathsFromTo{from_index,goal_index};
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

    % Find size of "nudge"
    % allPointsBeingPlotted = [reachableSet; nan nan];
    % 
    % max_plotValues = max(allPointsBeingPlotted);
    % min_plotValues = min(allPointsBeingPlotted);
    % sizePlot = max(max_plotValues) - min(min_plotValues);
    nudge = 0.15; %sizePlot*0.006; 

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
    for ith_fromPoint = 1:length(goalPoints(:,1))
        thisColorRow = mod(ith_fromPoint,Ncolors)+1;
        h_plot = plot(goalPoints(ith_fromPoint,1),goalPoints(ith_fromPoint,2),'.',...
            'Color',colorOrder(thisColorRow,:),'MarkerSize',30,'LineWidth', 2);
        if ith_fromPoint ==1
            set(h_plot, 'DisplayName','Input: goalPoints');
        else
            set(h_plot,'HandleVisibility','off');
        end
    end

    % Plot the TSP result, the ordered visit sequence
    % pointSequence = feasibleAllPoints(orderedVisitSequence,:);

    for ith_fromPoint = 1:length(orderedVisitSequence)-1
        thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
        thisColor = colorOrder(thisColorRow,:);

        from_index = orderedVisitSequence(ith_fromPoint,1);
        goal_index = orderedVisitSequence(ith_fromPoint+1,1);
        thisPath =  pathsFromTo{from_index,goal_index};
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
    for ith_fromPoint = 1:length(startAndGoalPoints)
        text(startAndGoalPoints(ith_fromPoint,1)+nudge,startAndGoalPoints(ith_fromPoint,2), ...
            sprintf('%.0f',ith_fromPoint));
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
function fcn_INTERNAL_debugPlotInputs(windFieldU,windFieldV,windFieldX,windFieldY, startPoint, goalPoints, debug_figNum)
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
for ith_fromPoint = 1:length(goalPoints(:,1))
    thisColorRow = mod(ith_fromPoint,Ncolors)+1;
    plot(goalPoints(ith_fromPoint,1),goalPoints(ith_fromPoint,2),'.',...
        'Color',colorOrder(thisColorRow,:),'MarkerSize',20,'LineWidth', 2, 'DisplayName','Input: goalPoints');
end

end % Ends fcn_INTERNAL_debugPlotInputs


%% fcn_INTERNAL_greedySearch
function [visitSequence, accumulatedCosts] = fcn_INTERNAL_greedySearch(...
    costsFromTo, visitSequence, flagsCityWasVisited, priorCost, pathsFromTo, flag_do_debug, debug_figNum)

accumulatedCosts = priorCost;
startGoal = find(~isnan(visitSequence),1,'last');
NumGoals = size(costsFromTo,1);

% For debugging
allCosts = ones(size(visitSequence))*priorCost;

for ith_visit = (startGoal+1):NumGoals
    % What was the last city visited?
    previousCity = visitSequence(ith_visit-1,1);

    % Which cities haven't been visited?
    unvisitedCities = find(flagsCityWasVisited==0);

    % What are the costs to travel from the last city?
    previousCosts = costsFromTo(previousCity,:);

    % What are the costs to travel from the last city to one of the
    % unvisited cities?
    unvisitedCosts = previousCosts(:,unvisitedCities);

    % Which one is the smallest cost? (be "greedy")
    [minCost,indexMin] = min(unvisitedCosts);

    % Go to that city
    thisCity = unvisitedCities(indexMin);
    visitSequence(ith_visit,1) = thisCity;
    flagsCityWasVisited(thisCity,1) = 1;

    % Add the cost to go to that city
    accumulatedCosts = accumulatedCosts + minCost;

    allCosts(ith_visit,1) = accumulatedCosts;

end

% Connect back to start
visitSequence(end,:) = 1;
accumulatedCosts = accumulatedCosts + costsFromTo(thisCity,1);

allCosts(end,1) = accumulatedCosts;

if 1==0
    figure(33333);
    plot(allCosts);
end

% Plot the greedy result
if flag_do_debug
    fprintf(1,'Greedy search cost: %.2f\n',allCosts(end,1));

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
        thisPath =  pathsFromTo{from_index,goal_index};
        plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth',5);
        % arrowMagnitude = pointSequence(ith_point+1,:)-pointSequence(ith_point,:);
        % quiver(pointSequence(ith_point,1),pointSequence(ith_point,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
        %     'LineWidth',3);

    end
end


end % Ends fcn_INTERNAL_greedySearch