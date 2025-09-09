function [predictedOrderedVisitSequence, probabilitiesSequence, finalCost] = ...
    fcn_BoundedAStar_approximateTSPviaMerging(...
    costsFromTo, varargin)
% fcn_BoundedAStar_approximateTSPviaMerging solves the Traveling Salesman Problem
%
% FORMAT:
% [predictedOrderedVisitSequence, probabilitiesSequence, finalCost] = fcn_BoundedAStar_solveTSP(...
%     costsFromTo, (cellArrayOfFunctionOptions), (figNum));
%
% INPUTS:
% 
%     costsFromTo: an MxM matrix specifying the costs to traverse
%     from a point (row) to another point (column). 
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
%     predictedOrderedVisitSequence: the set of points, starting and ending
%     with the first point, containing the visit sequence index list.
%
%     probabilitiesSequence: for each transition in the
%     predictedOrderedVisitSequence, the probabilities of that transition
%     being used (estimated). For example, the probabilitiesSequence(1,1)
%     contains the probability that the transition from (1,1) to (1,2) is
%     used.
%
%     finalCost: the final traversal cost
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_approximateTSPviaMerging
% for a full test suite.
%
% This function was written on 2025_09_06 by S. Brennan
% Questions or comments? contact S. Brennan sbrennan@psu.edu 
% or K. Hayes, kxh1031@psu.edu

% REVISION HISTORY:
% 2025_09_06 by S. Brennan
% - in fcn_BoundedAStar_approximateTSPviaMerging
%   % * first write of function using fcn_BoundedAStar_solveTSP
%   %   % as a starter
%
% 2025_09_09 by S. Brennan
% - In fcn_BoundedAStar_approximateTSPviaMerging:
%   % * Added final cost as an output
%   % * Fixed bug producing edge-closing path sequences
%   % * Fixed bug where last plotted line color was wrong

% TO-DO
% (none)


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 3; % The largest Number of argument inputs to the function
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
    debug_figNum = 23434; 
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
        narginchk(1,MAX_NARGIN);

        % Check the costsFromTo input, make sure it is MxM
        assert(isnumeric(costsFromTo));
        M = size(costsFromTo,1);
        assert(isequal(size(costsFromTo),[M M]));

    end
end

% Does user want to specify cellArrayOfFunctionOptions input?
cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = []; % windFieldU
cellArrayOfFunctionOptions{2} = []; % windFieldV
cellArrayOfFunctionOptions{3} = []; % windFieldX
cellArrayOfFunctionOptions{4} = []; %#ok<NASGU> % windFieldY
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        cellArrayOfFunctionOptions = temp; %#ok<NASGU> 
    end
end
% windFieldU = cellArrayOfFunctionOptions{1};
% windFieldV = cellArrayOfFunctionOptions{2};
% windFieldX = cellArrayOfFunctionOptions{3};
% windFieldY = cellArrayOfFunctionOptions{4};

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

% Update changedIndices
NoriginalCosts = size(costsFromTo,1);

costsFromToMerged = costsFromTo;
mergeProbability = fcn_INTERNAL_evalAllSharedEdges(costsFromToMerged);

if 1==flag_do_debug
    fcn_INTERNAL_plotTransitionsThroughCost(mergeProbability, [], [], debug_figNum)
end

%%%%
% Try ranking probabilities
Ninmatrix = numel(mergeProbability);
rankedIndices = nan(Ninmatrix,3);
tempProbability = mergeProbability;
for ith_rank = 1:Ninmatrix
    [maxProbability,indexMax] = max(tempProbability,[],'all');
    [A,N] = ind2sub(size(mergeProbability), indexMax);
    rankedIndices(ith_rank,:) = [maxProbability, A, N];
    tempProbability(A,N) = -2;
end

%%%%
% Convert rankings into visit order
edgeLinkingAndRanking = nan(NoriginalCosts,3);
Nfilled = 0;

% Create a cell array to store linkages
linkedEdges = nan(NoriginalCosts+1,NoriginalCosts+1);
NlinkedEdges = 0;
linkedEdgesStartEnd = [0 0]; % Initial value

% tempRankedIndices has the following format for each row:
Nranked = length(rankedIndices(:,1));
tempRankedIndices = rankedIndices;
for ith_probability = 1:Nranked

    %     if ith_probability==71
    %         disp('Stop here');
    %     end

    % Grab info from this ranking item
    thisRow = tempRankedIndices(ith_probability,:);
    thisProbability = thisRow(1,1);
    thisFrom = thisRow(1,2);
    thisTo   = thisRow(1,3);

    flagInList = 0;

    % Is this row already invalid?
    if isnan(thisProbability)
        flagInList = 1;
    end

    %     % * Has this "from" been used previously as "from"?
    %     if any(edgeLinkingAndRanking(:,2)==thisFrom)
    %         flagInList = 1;
    %     end
    %     % * Has this "to" link been used previously as "to"?
    %     if any(edgeLinkingAndRanking(:,3)==thisTo)
    %         flagInList = 1;
    %     end
    %     % * Is this link a reflection of a prior link?
    %     flippedToFromMatches = sum(fliplr(thisRow(1,2:3))==edgeLinkingAndRanking(:,2:3),2);
    %     if any(flippedToFromMatches==2)
    %         flagInList = 1;
    %     end

    % * Does this link close a cycle early? in other words, is this link a
    % reflection of a start/end pair?
    flippedToFromMatches = sum(fliplr(thisRow(1,2:3))==linkedEdgesStartEnd,2);
    if any(flippedToFromMatches==2) && Nfilled<(NoriginalCosts-1)
        flagInList = 1;
    end

    % If not in the list, then add to edgeLinkingAndRanking
    if 0==flagInList
        Nfilled = Nfilled+1;

        % Add to the edge linking list
        edgeLinkingAndRanking(Nfilled,:) = [thisProbability thisFrom thisTo];

        % Set all the rows that have the same "from" value to invalid
        rowsSameFrom = tempRankedIndices(:,2) == thisFrom;
        tempRankedIndices(rowsSameFrom,1) = nan;

        % Set all the rows that have the same "to" value to invalid
        rowsSameTo = tempRankedIndices(:,3) == thisTo;
        tempRankedIndices(rowsSameTo,1) = nan;

        % Check to see if this addition links into existing edge linking?
        % Does this addition join existing edges?
        fromLink = find(thisFrom==linkedEdgesStartEnd(:,2));
        toLink   = find(thisTo==linkedEdgesStartEnd(:,1));

        if fromLink==toLink
            if Nfilled==NoriginalCosts
                fromLink = [];
                toLink = [];
            else
                error('Unexpected match of from and to links');
            end
        end
        fromIndices = linkedEdges(fromLink,~isnan(linkedEdges(fromLink,:)));
        toIndices = linkedEdges(toLink,~isnan(linkedEdges(toLink,:)));        
        if ~isempty(fromLink) && ~isempty(toLink)
            % Merge edges
            allIndices = [fromIndices toIndices];
            Nindices = length(allIndices);
            fullRow = [allIndices nan(1,NoriginalCosts-Nindices+1)];
            linkedEdges(fromLink,:) = fullRow;
            linkedEdges(toLink,:) = [];
            NlinkedEdges = NlinkedEdges-1;
        elseif ~isempty(fromLink)
            % Add this edge onto end
            allIndices = [fromIndices thisTo];
            Nindices = length(allIndices);
            fullRow = [allIndices nan(1,NoriginalCosts-Nindices+1)];
            linkedEdges(fromLink,:) = fullRow;
        elseif ~isempty(toLink)
            % Add this edge onto start
            allIndices = [thisFrom toIndices];
            Nindices = length(allIndices);
            fullRow = [allIndices nan(1,NoriginalCosts-Nindices+1)];
            linkedEdges(toLink,:) = fullRow;
        else
            % This is a new addition
            allIndices = [thisFrom thisTo];
            Nindices = length(allIndices);
            fullRow = [allIndices nan(1,NoriginalCosts-Nindices+1)];
            NlinkedEdges = NlinkedEdges+1;
            linkedEdges(NlinkedEdges,:) = fullRow;
        end
        
        linkedEdgesStartEnd = nan(NlinkedEdges,2);
        for ith_link = 1:NlinkedEdges
            thisRow = linkedEdges(ith_link,~isnan(linkedEdges(ith_link,:)));
            linkedEdgesStartEnd(ith_link,:) = [thisRow(1,1) thisRow(1,end)];
        end
    end

end

% Rearrange, and add up visit costs
predictedOrderedVisitSequence = nan(1,NoriginalCosts+1);
probabilitiesSequence = nan(1,NoriginalCosts+1);
fromCity = 1;
totalCost = 0;
for ith_probability = 1:NoriginalCosts
    rowIndexToUse = edgeLinkingAndRanking(:,2)==fromCity;
    thisRow = edgeLinkingAndRanking(rowIndexToUse,:);
    toCity = thisRow(1,3);

    % Fill in results
    predictedOrderedVisitSequence(1,ith_probability) = fromCity;
    totalCost = totalCost + costsFromTo(fromCity,toCity);
    probabilitiesSequence(1,ith_probability) = thisRow(1,1);

    % Grab next value
    fromCity = toCity;
end
predictedOrderedVisitSequence(1,end) = 1;


finalCost = totalCost;

if 1==flag_do_debug
    disp([predictedOrderedVisitSequence; probabilitiesSequence]);
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
    nudge = 0.15; %#ok<NASGU> %sizePlot*0.006; 

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
    legend('Interpreter','none','Location','northwest');

    fcn_INTERNAL_plotTransitionsThroughCost(mergeProbability, predictedOrderedVisitSequence, probabilitiesSequence, figNum)

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
%% fcn_INTERNAL_evalSharedEdges
function [largestBaseCostDifference, largestBaseCostDifference_percent,...
    ANXmatch, ANXratios, ...
    XANmatch, XANratios] = ...
    fcn_INTERNAL_evalSharedEdges(adjustedCostsFromTo,A,N)

Ngoals = size(adjustedCostsFromTo,1);
% Test to see if A-->X is lower than A-->N-->X 
fromA = adjustedCostsFromTo(A,:);
fromN = adjustedCostsFromTo(N,:);
toN   = adjustedCostsFromTo(:,N);
fromAtoN = adjustedCostsFromTo(A,N);
fromNplusFromAtoN = fromN+fromAtoN;
if 1==0
    disp([fromA; fromNplusFromAtoN])
end
differencesBaseCost = fromNplusFromAtoN - fromA;
indicesToCheck = find(~isinf(differencesBaseCost));
[largestBaseCostDifference, indexMax] = max(differencesBaseCost(indicesToCheck));
largestBaseCostDifference_percent = largestBaseCostDifference/fromA(indicesToCheck(indexMax));

% Test to see if A-->N-->X is always minimum of 2-edge jump with A-->N
ArowPlusMatrix = repmat(fromA',1,Ngoals) + adjustedCostsFromTo;
% Perform column-wise minimization to see if N is solution
allIndices = (1:Ngoals);
colsToTest = allIndices(allIndices~=N);

bestRows = zeros(Ngoals-1,1);
bestRatios = zeros(Ngoals-1,1);
for ith_col = 1:length(colsToTest)
    thisCol = colsToTest(ith_col);
    [minVal, lowRow] = min(ArowPlusMatrix(:,thisCol));
    bestRows(ith_col,1) = lowRow;
    bestRatios(ith_col,1) = ArowPlusMatrix(N,thisCol)/minVal;
end
ANXmatch = bestRows==N;
ANXratios = bestRatios;

% Test to see if M-->A-->N is always minimum of 2-edge jump with A-->N
NcolPlusMatrix = repmat(toN',Ngoals,1) + adjustedCostsFromTo;

bestCols = zeros(Ngoals-1,1);
bestRatios = zeros(Ngoals-1,1);
rowsToTest = allIndices(allIndices~=A);
for ith_row = 1:length(rowsToTest)
    thisRow = rowsToTest(ith_row);
    [minVal, lowCol] = min(NcolPlusMatrix(thisRow,:));
    bestCols(ith_row,1) = lowCol;
    bestRatios(ith_row,1) = NcolPlusMatrix(thisRow,A)/minVal;
end
XANmatch = bestCols==A;
XANratios = bestRatios;
 
end % ends fcn_INTERNAL_findSharedEdges

%% fcn_INTERNAL_evalAllSharedEdges
function mergeProbability = fcn_INTERNAL_evalAllSharedEdges(costsFromTo)
% Finds the merging probability for all from/to permutations

adjustedCostsFromTo = costsFromTo;
indicesToChange = costsFromTo==-1;
adjustedCostsFromTo(indicesToChange) = inf;

% [~,indexMin] = min(adjustedCostsFromTo,[],'all');
% [A,N] = ind2sub(size(adjustedCostsFromTo), indexMin);
% % Gives
% % A = 7;
% % N = 4;
% A = 7; N = 4;
% [largestBaseCostDifference, largestBaseCostDifference_percent,...
%     ANXmatch, ANXratios, ...
%     XANmatch, XANratios] = fcn_INTERNAL_evalSharedEdges(adjustedCostsFromTo,A,N);
% percentage = 2*(mean(1./([ANXratios; XANratios]))-0.5);


Nmatrix = size(adjustedCostsFromTo,1);
mergeProbability = nan(size(adjustedCostsFromTo));
for ith_row = 1:Nmatrix
    for jth_col = 1:Nmatrix
        [largestBaseCostDifference, largestBaseCostDifference_percent,...
            ANXmatch, ANXratios, ...
            XANmatch, XANratios] = fcn_INTERNAL_evalSharedEdges(adjustedCostsFromTo,ith_row,jth_col); %#ok<ASGLU>
        percentage = 2*(mean(1./([ANXratios; XANratios]))-0.5);
        mergeProbability(ith_row,jth_col) = percentage;
    end
end

end % Ends fcn_INTERNAL_evalAllSharedEdges




%% fcn_INTERNAL_plotTransitionsThroughCost
function fcn_INTERNAL_plotTransitionsThroughCost(costsFromTo, knownOrderedVisitSequence, probabilitiesSequence, figNum)
Ngoals = size(costsFromTo,1);
cell_range = [1 1 Ngoals Ngoals];
costX = linspace(cell_range(1), cell_range(3), Ngoals);
costY = linspace(cell_range(2), cell_range(4), Ngoals);
% [X, Y] = meshgrid(costX, costY);
plotCosts = costsFromTo;
plotCosts(plotCosts==-1) = nan;

figure(figNum);
clf;
hold on;
% mesh(X,Y,plotCosts)
contourf(costX, costY, plotCosts, 20, 'EdgeColor', 'none','HandleVisibility','off');
set(gca,'YDir','reverse');
xlabel('Columns');
ylabel('Rows');

% Plot the sequence?
if ~isempty(knownOrderedVisitSequence)

    % Get the colors to use
    colorsTurbo = colormap('turbo');
    % Cut from middle to top (green to red)
    % colorsGreenToRed = colorsTurbo(128:end,:);
    colorsGreenToRed = colorsTurbo; %(1:end,:);
    colorsRedToGreen = flipud(colorsGreenToRed);
    colormap(colorsRedToGreen);    
    Ncolors = length(colorsRedToGreen(:,1));

    for ith_sequence = 1:length(knownOrderedVisitSequence)-2
        fromRow = knownOrderedVisitSequence(ith_sequence);
        fromCol = knownOrderedVisitSequence(ith_sequence+1);
        % if ith_sequence==1
        h_plot = plot(fromCol,fromRow,'.','MarkerSize',40); %#ok<NASGU>
        %         else
        %             h_plot = plot(fromCol,fromRow,'.','MarkerSize',40,'Color',toColor);
        %         end

        thisProbability = probabilitiesSequence(1,ith_sequence);
        thisColor = max(min(ceil(thisProbability*Ncolors),Ncolors),1);
        fromColor = colorsRedToGreen(thisColor,:);

        toRow   = knownOrderedVisitSequence(ith_sequence+1);
        toCol   = knownOrderedVisitSequence(ith_sequence+2);
        h_plot = plot(toCol,toRow,'.','MarkerSize',40); %#ok<NASGU> 
        %         toColor = get(h_plot,'Color');

        quiver(fromCol,fromRow,toCol-fromCol,toRow-fromRow,0,'filled','Color',fromColor,'LineWidth',3);
    end

    % Plot last point
    thisProbability = probabilitiesSequence(1,end-1);
    thisColor = max(min(ceil(thisProbability*Ncolors),Ncolors),1);
    fromColor = colorsRedToGreen(thisColor,:);

    fromRow = knownOrderedVisitSequence(end-1);
    fromCol = knownOrderedVisitSequence(end);
    toRow   = knownOrderedVisitSequence(1);
    toCol   = knownOrderedVisitSequence(2);
    plot(toCol,toRow,'.','MarkerSize',40);
    quiver(fromCol,fromRow,toCol-fromCol,toRow-fromRow,0,'filled','Color',fromColor,'LineWidth',3);

    % Turn on colorbar
    c = colorbar;
    c.Label.String = 'Probability of Corridor Use';

end
end % Ends fcn_INTERNAL_plotTransitionsThroughCost
