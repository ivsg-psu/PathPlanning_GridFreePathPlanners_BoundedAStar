%% Figure 1 - Wind field example
figNum = 1;
%titleString = sprintf('DEMO case: get reachable envelope in wind field (starting point at origin, strong wind)');
%fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(12);

% Call graph generation function
radius = 1;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 100; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

figure(1)
hold on
fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, [], 1);
s = streamslice(windFieldX, windFieldY, windFieldU, windFieldV);
set(s, 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
xlabel('X - East'), ylabel('Y - North')
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')

%% Figure 2 - Matched wind field boundary approximation example
figNum = 2;
%titleString = sprintf('DEMO case: get reachable envelope in wind field (starting point at origin, strong wind)');
%fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(12);

% Call graph generation function
radius = 1;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 100; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(exitCondition));
assert(iscell(cellArrayOfExitInfo));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(exitCondition,1)==1); 
assert(size(exitCondition,2)==1);
assert(isequal(size(cellArrayOfExitInfo),[2 1]));

% Check variable values
% (too difficult - randomly generated)

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

% Figure setup
figure(2)
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')
xlabel('X - East'), ylabel('Y - North')
c = gca().Children;
set(c(end), 'DisplayName', 'Start Point', 'LineStyle','none')
set(c(1), 'DisplayName', 'Approximated Reachable Boundary')

%% Figure 3 - Strong wind field boundary approximation example
figNum = 3;
%titleString = sprintf('DEMO case: get reachable envelope in wind field (starting point at origin, strong wind)');
%fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(12);

% Call graph generation function
radius = 0.25;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 100; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(exitCondition));
assert(iscell(cellArrayOfExitInfo));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(exitCondition,1)==1); 
assert(size(exitCondition,2)==1);
assert(isequal(size(cellArrayOfExitInfo),[2 1]));

% Check variable values
% (too difficult - randomly generated)

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

% Figure setup
figure(3)
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')
xlabel('X - East'), ylabel('Y - North')
c = gca().Children;
set(c(end), 'DisplayName', 'Start Point', 'LineStyle','none')
set(c(1), 'DisplayName', 'Approximated Reachable Boundary')

%% Figure 4 - Forward simulation verification for trajectory 
figNum = [];
titleString = sprintf('DEMO case: basic example of calculation back to start');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
% figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(12);

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
flagWindRoundingType = 0;

Nsteps = 8;
cellArrayOfExpansions = cell(Nsteps,1);
cellArrayOfIntermediateCalculations = cell(Nsteps,5);
cellArrayOfExpansions{1,1} = startPoints;
% figure(234); clf;

for ith_step = 2:Nsteps
    % Call function to find reachable set on this time step
    % FORMAT:
    % [reachableSet, thisCellArrayOfIntermediateCalculations] =  fcn_BoundedAStar_reachabilityWithInputs(...
    %     radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));
    [reachableSet, thisCellArrayOfIntermediateCalculations] = ...
        fcn_BoundedAStar_reachabilityWithInputs(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, ...
        (cellArrayOfExpansions{ith_step-1,1}), (flagWindRoundingType), (2223));
    title(sprintf('Step: %.0f of %.0f',ith_step,Nsteps))

    cellArrayOfExpansions{ith_step,1} = reachableSet;
    for ith_cell = 1:5
        cellArrayOfIntermediateCalculations{ith_step,ith_cell} = thisCellArrayOfIntermediateCalculations{ith_cell,1};
    end

end

endPoint = [-3 6];
% endPoint = [0.5 .98];
% endPoint = [-2 0];

% Call function to find "backward" path
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Do forward simlation (to test)
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, pathXYAndControlUV(:,3:4), ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    (figNum));

% figure(figNum);
% hold on;
% % Plot the final XY path in blue
% plot(pathXYAndControlUV(:,1),pathXYAndControlUV(:,2),'LineWidth',3,...
%     'MarkerSize',30,...
%     'Color',[0 0 1],'DisplayName','Expected: XY path')
% 
% % Plot the endPoint
% plot(endPoint(:,1),endPoint(:,2),'.','Color',[1 0 0],'MarkerSize',30,'LineWidth', 2, 'DisplayName','Expected: endPoint');


sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

% Make sure plot opened up
% assert(isequal(get(gcf,'Number'),figNum));

figure(4)
hold on
legend off
fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, [], 4)
s = streamslice(windFieldX, windFieldY, windFieldU, windFieldV);
set(s, 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
plot(startPoints(1), startPoints(2), '.g', 'MarkerSize', 30, 'DisplayName', 'Start Point');
plot(endPoint(1), endPoint(2), '.r', 'MarkerSize', 30, 'DisplayName', 'End Point');
plot(pathXYAndControlUV(:,1), pathXYAndControlUV(:,2), '-b', 'LineWidth', 3, 'DisplayName', 'Expected XY Path');
plot(pathXY(:,1), pathXY(:,2), '--c', 'LineWidth', 2, 'DisplayName', 'Simulated XY Path')
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')
xlabel('X - East'), ylabel('Y - North')
legend
axis([-6 6 -2 9])

%% Figure 5 - Circulating wind field TSP solution with N = 6
% NOTE: vehicle speed set to 0.45 for now while I try to find a bug. For
% some reason, radius = 0.5 has started throwing NaN startPoints errors,
% but 0.45 looks close to the figure in the paper for now.
figNum = 5;
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


% Load starting data
% 6 is OK, 8 is fairly good,
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(12);

% Call graph generation function
radius = 0.45;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoint = [0 -8];
goalPoints = [0 -2; 0 4; -3 8; -8 8; -8 0; -8 -8];

cellArrayOfSearchOptions = cell(5,1);
cellArrayOfSearchOptions{1} = 50; % Nsteps
cellArrayOfSearchOptions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfSearchOptions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfSearchOptions{4} = goalPoints;  % allGoalPointsList
cellArrayOfSearchOptions{5} = 0;   % flagStopIfHitOneGoalPoint

% Call function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSPwithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, startPoint, goalPoints, (cellArrayOfSearchOptions), (figNum));

% sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

% Figure setup
figure(5)
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')
xlabel('X - East'), ylabel('Y - North')
c = gca().Children;
set(c(end), 'DisplayName', 'Start Point', 'LineStyle','none', 'Color', 'g')
set(c(end-1), 'DisplayName', 'Goal Points')
set(c(end-2), 'DisplayName', 'Feasible Points')
set(c(end-3), 'DisplayName', 'Solution Path')
legend('location', 'northeast')

%% Figure 6 - Environment setup for TSP simulation example
figNum = 6;
%titleString = sprintf('DEMO case: negative circulating wind field, 8 goal points specified');
%fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
% Map 6 is OK, 8 is fairly good,
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(14);

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoint = [0 -8];

xRange = windFieldX(end)-windFieldX(1);
yRange = windFieldY(end)-windFieldY(1);

Ngoals = 8;
rng(5);
percentCut = 0.1;
randDeviation = (rand(Ngoals,2)-0.5)*(1-percentCut);
midX = mean(windFieldX);
midY = mean(windFieldY);
goalPoints = randDeviation.*[xRange yRange] + ones(Ngoals,1)*[midX midY];

figure(6)
hold on
legend off
fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, [], 6)
s = streamslice(windFieldX, windFieldY, windFieldU, windFieldV);
set(s, 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
plot(startPoint(1), startPoint(2), '.g', 'MarkerSize', 30, 'DisplayName', 'Start Point');
plot(goalPoints(:,1), goalPoints(:,2), '.k', 'MarkerSize', 30, 'DisplayName', 'Goal Points');
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')
xlabel('X - East'), ylabel('Y - North')
legend('location', 'northwest')

%% Figure 7 - Boundary expansion solution to TSP example
figNum = 7;
%titleString = sprintf('DEMO case: negative circulating wind field, 8 goal points specified');
%fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
% Map 6 is OK, 8 is fairly good,
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(14);

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoint = [0 -8];

xRange = windFieldX(end)-windFieldX(1);
yRange = windFieldY(end)-windFieldY(1);

Ngoals = 8;
rng(5);
percentCut = 0.1;
randDeviation = (rand(Ngoals,2)-0.5)*(1-percentCut);
midX = mean(windFieldX);
midY = mean(windFieldY);
goalPoints = randDeviation.*[xRange yRange] + ones(Ngoals,1)*[midX midY];

%goalPoints = [0 -2; 0 4; -3 8; -8 8; -8 0; -8 -8];

cellArrayOfSearchOptions = cell(5,1);
cellArrayOfSearchOptions{1} = 50; % Nsteps
cellArrayOfSearchOptions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfSearchOptions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfSearchOptions{4} = goalPoints;  % allGoalPointsList
cellArrayOfSearchOptions{5} = 0;   % flagStopIfHitOneGoalPoint

% Call function
[orderedVisitSequence] = fcn_BoundedAStar_solveTSPwithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, startPoint, goalPoints, (cellArrayOfSearchOptions), (figNum));

%sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(orderedVisitSequence));

% Check variable sizes
assert(size(orderedVisitSequence,1)>=3); 
assert(size(orderedVisitSequence,2)==1);

% Check variable values
% (too difficult - randomly generated)
% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

figure(7)
f = gcf();
fontname(f, 'Times New Roman')
fontsize(f, 14, 'points')
xlabel('X - East'), ylabel('Y - North')
c = gca().Children;
set(c(end), 'DisplayName', 'Start Point', 'LineStyle','none', 'Color', 'g')
set(c(end-1), 'DisplayName', 'Goal Points')
set(c(end-2), 'DisplayName', 'Feasible Points')
set(c(end-3), 'DisplayName', 'Solution Path')
legend('location', 'northwest')


%% Figure 8 - RRT* solution to TSP example



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

%% fcn_INTERNAL_loadZeroWindData
function [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadZeroWindData(seed)
% Fill inputs
% 1 produces left to right, with edge patches that have right to left
% 2 produces left to right on top half, and top to bottom on bottom half
% 3 produces left to right on left half, bottom to top on right half
% 4 produces a saddle point near the middle
% 4822262 produces wind field that is mostly left to right

if ~exist('seed','var') || isempty(seed)
    randomSeed = 4;
else
    randomSeed = seed;
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
normalizedEastWind = 0*eastWind./maxWind;
normalizedNorthWind = 0*northWind./maxWind;

XY_range = [-10 -10 10 10];
windFieldX = linspace(XY_range(1), XY_range(3), nRows);
windFieldY = linspace(XY_range(2), XY_range(4), mColumns);

end % Ends fcn_INTERNAL_loadZeroWindData

%% fcn_INTERNAL_loadSideWindData
function [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadSideWindData(seed)
% Fill inputs
% 1 produces left to right, with edge patches that have right to left
% 2 produces left to right on top half, and top to bottom on bottom half
% 3 produces left to right on left half, bottom to top on right half
% 4 produces a saddle point near the middle
% 4822262 produces wind field that is mostly left to right

if ~exist('seed','var') || isempty(seed)
    randomSeed = 4;
else
    randomSeed = seed;
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

normalizedEastWind = ones(size(eastWind));
normalizedNorthWind = 0*northWind./maxWind;

XY_range = [-10 -10 10 10];
windFieldX = linspace(XY_range(1), XY_range(3), nRows);
windFieldY = linspace(XY_range(2), XY_range(4), mColumns);

end % Ends fcn_INTERNAL_loadSideWindData


%% fcn_INTERNAL_loadExampleData
function [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(seed)
% Fill inputs
% 1 produces left to right, with edge patches that have right to left
% 2 produces left to right on top half, and top to bottom on bottom half
% 3 produces left to right on left half, bottom to top on right half
% 4 produces a saddle point near the middle
% 4822262 produces wind field that is mostly left to right

if ~exist('seed','var') || isempty(seed)
    randomSeed = 4;
else
    randomSeed = seed;
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

end % Ends fcn_INTERNAL_loadExampleData
