% script_test_fcn_BoundedAStar_solveTSP
% Tests: fcn_BoundedAStar_solveTSP

% Revision history
% 2025_08_29 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_solveTSP:
%   % * first write of script 
%   % * using script_test_fcn_BoundedAStar_solveTSP as starter

% TO DO:
% (none)


%% Set up the workspace
close all

%% Code demos start here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____                              ____   __    _____          _
%  |  __ \                            / __ \ / _|  / ____|        | |
%  | |  | | ___ _ __ ___   ___  ___  | |  | | |_  | |     ___   __| | ___
%  | |  | |/ _ \ '_ ` _ \ / _ \/ __| | |  | |  _| | |    / _ \ / _` |/ _ \
%  | |__| |  __/ | | | | | (_) \__ \ | |__| | |   | |___| (_) | (_| |  __/
%  |_____/ \___|_| |_| |_|\___/|___/  \____/|_|    \_____\___/ \__,_|\___|
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Demos%20Of%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 1

close all;
fprintf(1,'Figure: 1XXXXXX: DEMO cases\n');

%% DEMO case: matched wind field, 4 goal points specified
figNum = 10001;
titleString = sprintf('DEMO case: matched wind field, goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 4; 
Ngoals = 4;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: circulating wind field, 6 goal points specified
figNum = 10002;
titleString = sprintf('DEMO case: circulating wind field, goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 12;
Ngoals = 6;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: negative circulating wind field, 6 goal points specified
figNum = 10003;
titleString = sprintf('DEMO case: negative circulating wind field, goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 13;
Ngoals = 6;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% DEMO case: negative circulating wind field, 7 goal points specified
figNum = 10004;
titleString = sprintf('DEMO case: negative circulating wind field, 7 goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 14;
Ngoals = 7;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));




%% DEMO case: negative circulating wind field, 8 goal points specified
figNum = 10005;
titleString = sprintf('DEMO case: negative circulating wind field, 8 goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 14;
Ngoals = 8;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: negative circulating wind field, 9 goal points specified
figNum = 10006;
titleString = sprintf('DEMO case: negative circulating wind field, 9 goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 14;
Ngoals = 9;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: negative circulating wind field, 10 goal points specified
figNum = 10007;
titleString = sprintf('DEMO case: negative circulating wind field, 10 goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 14;
Ngoals = 10;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% Test cases start here. These are very simple, usually trivial
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  _______ ______  _____ _______ _____
% |__   __|  ____|/ ____|__   __/ ____|
%    | |  | |__  | (___    | | | (___
%    | |  |  __|  \___ \   | |  \___ \
%    | |  | |____ ____) |  | |  ____) |
%    |_|  |______|_____/   |_| |_____/
%
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 2

close all;
fprintf(1,'Figure: 2XXXXXX: TEST mode cases\n');

%% TEST case: circulating wind field, 6 goal points specified
figNum = 20001;
titleString = sprintf('TEST case: circulating wind field, goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

RNG = 12;
Ngoals = 6;
testCase = 1;
knownOrderedVisitSequence = [ 1     7     4     3     5     6     2     1];

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

% Update changedIndices
NoriginalCosts = size(costsFromTo,1);
allIndices = (1:NoriginalCosts)';
startingIndices = num2cell(allIndices);
changedIndices = startingIndices;


costsFromToMerged = costsFromTo;
mergeProbability = fcn_INTERNAL_evalAllSharedEdges(costsFromToMerged);
fcn_INTERNAL_plotTransitionsThroughCost(mergeProbability, knownOrderedVisitSequence,4000)

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


% %%%
% % Itereatively recompute cost matrix?
% for ith_merge = 1:NoriginalCosts-1
%     figNum = 4000+ith_merge;
%     mergeProbability = fcn_INTERNAL_evalAllSharedEdges(costsFromToMerged);
%     % fcn_INTERNAL_plotTransitionsThroughCost(mergeProbability, [], figNum)
%     [maxProbability,indexMax] = max(mergeProbability,[],'all');
%     [A,N] = ind2sub(size(mergeProbability), indexMax);
%     fprintf(1,'Best merge found:\n')
%     fprintf(1,'\tProbability: %.2f',maxProbability);
%     fprintf(1,'\tFrom: %s\n',num2str(changedIndices{A}));
%     fprintf(1,'\tTo: %s\n', num2str(changedIndices{N}));
%     [costsFromToMerged, changedIndices] = fcn_INTERNAL_mergeGoals(A,N,costsFromToMerged,changedIndices);
% end

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% Fast Mode Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ______        _     __  __           _        _______        _
% |  ____|      | |   |  \/  |         | |      |__   __|      | |
% | |__ __ _ ___| |_  | \  / | ___   __| | ___     | | ___  ___| |_ ___
% |  __/ _` / __| __| | |\/| |/ _ \ / _` |/ _ \    | |/ _ \/ __| __/ __|
% | | | (_| \__ \ |_  | |  | | (_) | (_| |  __/    | |  __/\__ \ |_\__ \
% |_|  \__,_|___/\__| |_|  |_|\___/ \__,_|\___|    |_|\___||___/\__|___/
%
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Fast%20Mode%20Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 8

close all;
fprintf(1,'Figure: 8XXXXXX: FAST mode cases\n');

%% Basic example - NO FIGURE
figNum = 80001;
fprintf(1,'Figure: %.0f: FAST mode, empty figNum\n',figNum);
figure(figNum); close(figNum);

RNG = 4;
Ngoals = 4;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), ([]));

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

RNG = 4;
Ngoals = 4;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

% Call TSP function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
    startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (-1));

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

RNG = 4;
Ngoals = 4;
testCase = 0;

% Load starting data
[startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(RNG, Ngoals, testCase);

cellArrayOfFunctionOptions = cell(4,1);
cellArrayOfFunctionOptions{1} = windFieldU;
cellArrayOfFunctionOptions{2} = windFieldV;
cellArrayOfFunctionOptions{3} = windFieldX;
cellArrayOfFunctionOptions{4} = windFieldY;

Niterations = 1;

% Slow mode
tic;
for ith_test = 1:Niterations
    % Call TSP function
    [orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
        startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), ([]));
end
slow_method = toc;

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call TSP function
    [orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
        startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (-1));
end
fast_method = toc;

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));

% Plot results as bar chart
figure(373737);
clf;
hold on;

X = categorical({'Normal mode','Fast mode'});
X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
Y = [slow_method fast_method ]*1000/Niterations;
bar(X,Y)
ylabel('Execution time (Milliseconds)')


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));



%% BUG cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ____  _    _  _____
% |  _ \| |  | |/ ____|
% | |_) | |  | | |  __    ___ __ _ ___  ___  ___
% |  _ <| |  | | | |_ |  / __/ _` / __|/ _ \/ __|
% | |_) | |__| | |__| | | (_| (_| \__ \  __/\__ \
% |____/ \____/ \_____|  \___\__,_|___/\___||___/
%
% See: http://patorjk.com/software/taag/#p=display&v=0&f=Big&t=BUG%20cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All bug case figures start with the number 9

% close all;

%% BUG

%% Fail conditions
if 1==0

       
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


%% fcn_INTERNAL_loadExampleData
function [startAndGoalPoints, costsFromTo, pathsFromTo, ...
    windFieldU, windFieldV, windFieldX, windFieldY] =fcn_INTERNAL_loadExampleData(RNGseed, Ngoals, testCase)

saveFileName = sprintf('TEST_fcn_BoundedAStar_solveTSP_haltonSet.mat');
saveLocation = fullfile(pwd,'Test_Fixtures',saveFileName);
if 1==0
    haltonPoints = haltonset(2);
    haltonPointsScrambled = scramble(haltonPoints,'RR2');
    haltonPoints10000 = net(haltonPointsScrambled,10000);
    save(saveLocation, 'haltonPoints10000','-v7.3');
else
    load(saveLocation, 'haltonPoints10000');
end

if 0==testCase
    saveFileName = sprintf('TEST_fcn_BoundedAStar_solveTSP_RNG%03d_Goals%02d.mat', RNGseed,Ngoals);
else
    saveFileName = sprintf('TEST_fcn_BoundedAStar_solveTSP_RNG%03d_Test%02d.mat', RNGseed,testCase);
end

saveLocation = fullfile(pwd,'Test_Fixtures',saveFileName);

if 1==1 && exist(saveLocation, 'file')
    load(saveLocation, ...
        'startAndGoalPoints', 'costsFromTo', 'pathsFromTo', ...
        'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY');
else
    % Fill inputs
    % 1 produces left to right, with edge patches that have right to left
    % 2 produces left to right on top half, and top to bottom on bottom half
    % 3 produces left to right on left half, bottom to top on right half
    % 4 produces a saddle point near the middle
    % 4822262 produces wind field that is mostly left to right

    if ~exist('seed','var') || isempty(RNGseed)
        randomSeed = 4;
    else
        randomSeed = RNGseed;
    end

    % Call random occupancy map function - code taken from
    % script_demo_generateRandomOccupancyAnimated
    rng(randomSeed)
    nRows = 50;
    mColumns = 50;
    mapSize = [nRows mColumns];

    occupancyRatio = 0.2;
    dilationLevel = 400;
    seedMap = rand(nRows,mColumns);
    leftDilationMultiplier = [];
    rightDilationMultiplier = [];
    optimizedThreshold = [];

    % Call the function once to initialize settings for upcoming calls
    % [occupancyMatrix, randomMatrixDilated, forcedThreshold, leftDilationMultiplier, rightDilationMultiplier] = ...
    [~, randomMatrixDilated, ~, ~, ~] = ...
        fcn_GridMapGen_generateRandomOccupancyMap(...
        'mapSize', (mapSize),... % [nRows mCols])
        'occupancyRatio',(occupancyRatio),... % [1x1] value between 0 and 1
        'dilationLevel',(dilationLevel),.... % [1x1] strictly positive int
        'seedMap', (seedMap),... % [1x1] integer to be a random seed or NxM matrix of random numbers
        'leftDilationMultiplier', (leftDilationMultiplier),... %  [nRows nRows], ...
        'rightDilationMultiplier', (rightDilationMultiplier),... % [mCols mCols], ...
        'thresholdForced', (optimizedThreshold), ... % [1x1] scalar
        'flagSkipThresholdOptimization',(0),...% [1x1] scalar
        'figNum',(-1));


    %%%% WIND POST-PROCESSING SHOULD BE MADE INTO AN ALTERNATE FUNCTION (?). FOR
    %%%% NOW THE CODE IS JUST COPIED.
    % Use the gradient to estimate wind direction
    [px,py] = gradient(randomMatrixDilated);
    eastWind  = py;
    northWind = -px;

    % Solve for the wind magnitude
    windMagnitude = (eastWind.^2+northWind.^2).^0.5;
    maxWind = max(windMagnitude,[],'all');
    normalizedEastWind = eastWind./maxWind;
    normalizedNorthWind = northWind./maxWind;

    XY_range = [-10 -10 10 10];
    windFieldX = linspace(XY_range(1), XY_range(3), nRows);
    windFieldY = linspace(XY_range(2), XY_range(4), mColumns);


    % Call graph generation function
    radius = 0.5;
    maxWindSpeed = 1;

    windFieldU = normalizedEastWind*maxWindSpeed;
    windFieldV = normalizedNorthWind*maxWindSpeed;
    xRange = windFieldX(end)-windFieldX(1);
    yRange = windFieldY(end)-windFieldY(1);

    if Ngoals==4
        startPoint = [0 0];
        goalPoints = [4 6; -6 4; 8 -4; -6 -2];
    elseif Ngoals==6
        startPoint = [0 -8];
        goalPoints = [0 -2; 0 4; -3 8; -8 8; -8 0; -8 -8];
    elseif Ngoals>=7
        startPoint = [0 0];
        percentCut = 0.1;
        % randDeviation = (rand(Ngoals,2)-0.5)*(1-percentCut);
        offset = 300;
        startingPoints = haltonPoints10000(offset:offset+Ngoals-1,:);
        randDeviation = (startingPoints-0.5)*(1-percentCut);
        midX = mean(windFieldX);
        midY = mean(windFieldY);
        goalPoints = randDeviation.*[xRange yRange] + ones(Ngoals,1)*[midX midY];
        if 1==1
            figure(38383);
            clf;
            plot(goalPoints(:,1),goalPoints(:,2),'k.','MarkerSize',30);
        end
    else
        error('Not filled yet')
    end
    
    if 0==testCase
        % Do nothing
    elseif 1==testCase
        goalPoints = [0 -2; 0 4; -7 -7; -8 8; -8 0; -8 -8];
    end

    cellArrayOfSearchOptions = cell(5,1);
    cellArrayOfSearchOptions{1} = 150; % Nsteps
    cellArrayOfSearchOptions{2} = 1;   % flagStopIfEntireFieldCovered
    cellArrayOfSearchOptions{3} = 0.2; % toleranceToStopIfSameResult
    cellArrayOfSearchOptions{4} = goalPoints;  % allGoalPointsList
    cellArrayOfSearchOptions{5} = 0;   % flagStopIfHitOneGoalPoint

    % Fill in costs
    [startAndGoalPoints, feasibleGoalIndices, ...
        costsFromTo, pathsFromTo] = ...
        fcn_BoundedAStar_findTSPCostsAndPaths(...
        radius, ...
        windFieldU, ...
        windFieldV, ...
        windFieldX, ...
        windFieldY, ...
        startPoint, ...
        goalPoints, ...
        (cellArrayOfSearchOptions),...
        (9999)); %#ok<ASGLU>

    %%%%%
    % Save results to file?
    if 1==1
        disp('Saving test file to avoid recalculation. This is done only once.')
        save(saveLocation, ...
            'startAndGoalPoints', 'costsFromTo', 'pathsFromTo', ...
            'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY','-v7.3');

    end
end
end % Ends fcn_INTERNAL_loadExampleData


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
 
end fcn_INTERNAL_findSharedEdges

%% fcn_INTERNAL_evalAllSharedEdges
function mergeProbability = fcn_INTERNAL_evalAllSharedEdges(costsFromTo)

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

end fcn_INTERNAL_evalAllSharedEdges

%% fcn_INTERNAL_plotTransitionsThroughCost
function fcn_INTERNAL_plotTransitionsThroughCost(costsFromTo, knownOrderedVisitSequence, figNum)
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
    for ith_sequence = 1:length(knownOrderedVisitSequence)-2
        fromRow = knownOrderedVisitSequence(ith_sequence);
        fromCol = knownOrderedVisitSequence(ith_sequence+1);
        if ith_sequence==1
            h_plot = plot(fromCol,fromRow,'.','MarkerSize',40);
        else
            h_plot = plot(fromCol,fromRow,'.','MarkerSize',40,'Color',toColor);
        end
        fromColor = get(h_plot,'Color');

        toRow   = knownOrderedVisitSequence(ith_sequence+1);
        toCol   = knownOrderedVisitSequence(ith_sequence+2);
        h_plot = plot(toCol,toRow,'.','MarkerSize',40);
        toColor = get(h_plot,'Color');

        quiver(fromCol,fromRow,toCol-fromCol,toRow-fromRow,0,'filled','Color',fromColor,'LineWidth',3);
    end

    % Plot last point
    fromRow = knownOrderedVisitSequence(end-1);
    fromCol = knownOrderedVisitSequence(end);
    toRow   = knownOrderedVisitSequence(1);
    toCol   = knownOrderedVisitSequence(2);
    plot(toCol,toRow,'.','MarkerSize',40);
    quiver(fromCol,fromRow,toCol-fromCol,toRow-fromRow,0,'filled','Color',fromColor,'LineWidth',3);
end
end % Ends fcn_INTERNAL_plotTransitionsThroughCost

%% fcn_INTERNAL_mergeGoals
function [costsFromToMerged, changedIndices] = fcn_INTERNAL_mergeGoals(A,N,costsFromTo, inputIndices)

% Fill a starter matrix
costsFromToMerged = costsFromTo;

% Grab the cost from row A to column N
costAtoN = costsFromTo(A,N);

% Add this cost to the "from N" (the row)
costsFromToMerged(N,:) = costsFromToMerged(N,:) + costAtoN;

% Change the "to N" cost (the column) to be the previous "to A" cost
toACosts = costsFromTo(:,A);
costsFromToMerged(:,N) = toACosts;

costsFromToMerged(N,N) = -1;

% Delete the A entry (rows and columns)
costsFromToMerged(:,A) = [];
costsFromToMerged(A,:) = [];

inputIndices{N,1} = [inputIndices{A,1} inputIndices{N,1}];

nextIndex = 1;
changedIndices = cell(length(inputIndices)-1,1);
for ith_index = 1:length(inputIndices)
    if A~=ith_index
        changedIndices{nextIndex} = inputIndices{ith_index};
        nextIndex = nextIndex+1;
    end
end

end % Ends fcn_INTERNAL_mergeGoals