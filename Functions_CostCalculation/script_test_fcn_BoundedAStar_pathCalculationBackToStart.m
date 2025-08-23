% script_test_fcn_BoundedAStar_pathCalculationBackToStart
% Tests: fcn_BoundedAStar_pathCalculationBackToStart

% Revision history
% 2025_08_17 by S. Brennan, sbrennan@psu.edu
% - first write of script 
%   % * using script_test_fcn_BoundedAStar_reachabilityWithInputs as 
%   %   % starter
%
% 2025_08_19 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_pathCalculationBackToStart
%   % * Added working example of backward path calculation

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

%% DEMO case: basic example of calculation back to start
figNum = 10001;
titleString = sprintf('DEMO case: basic example of calculation back to start');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 1;
maxWindSpeed = 0.01;

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

figure(figNum);
hold on;
% Plot the final XY path in blue
plot(pathXYAndControlUV(:,1),pathXYAndControlUV(:,2),'LineWidth',3,...
    'MarkerSize',30,...
    'Color',[0 0 1],'DisplayName','Expected: XY path')

% Plot the endPoint
plot(endPoint(:,1),endPoint(:,2),'.','Color',[1 0 0],'MarkerSize',30,'LineWidth', 2, 'DisplayName','Expected: endPoint');


sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

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

% %% TEST case: Drop a point on a streamline
% figNum = 20001;
% titleString = sprintf('TEST case: Drop a point on a streamline');
% fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
% figure(figNum); clf;


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

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 1;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
flagWindRoundingType = 0;

Nsteps = 4;
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
        (cellArrayOfExpansions{ith_step-1,1}), (flagWindRoundingType), (-1));
    cellArrayOfExpansions{ith_step,1} = reachableSet;
    for ith_cell = 1:5
        cellArrayOfIntermediateCalculations{ith_step,ith_cell} = thisCellArrayOfIntermediateCalculations{ith_cell,1};
    end

end

endPoint = [-3 6];
% endPoint = [-1 1];

% Call function
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, ([]));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 1;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
flagWindRoundingType = 0;

Nsteps = 4;
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
        (cellArrayOfExpansions{ith_step-1,1}), (flagWindRoundingType), (-1));
    cellArrayOfExpansions{ith_step,1} = reachableSet;
    for ith_cell = 1:5
        cellArrayOfIntermediateCalculations{ith_step,ith_cell} = thisCellArrayOfIntermediateCalculations{ith_cell,1};
    end

end

endPoint = [-3 6];
% endPoint = [-1 1];

% Call function
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (-1));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 1;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
flagWindRoundingType = 0;

Nsteps = 4;
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
        (cellArrayOfExpansions{ith_step-1,1}), (flagWindRoundingType), (-1));
    cellArrayOfExpansions{ith_step,1} = reachableSet;
    for ith_cell = 1:5
        cellArrayOfIntermediateCalculations{ith_step,ith_cell} = thisCellArrayOfIntermediateCalculations{ith_cell,1};
    end

end

endPoint = [-3 6];
% endPoint = [-1 1];

Niterations = 10;

% Slow mode
tic;
for ith_test = 1:Niterations
    % Call function
    pathXYAndControlUV =  ...
        fcn_BoundedAStar_pathCalculationBackToStart(...
        endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, ([]));
end
slow_method = toc;

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call function
    pathXYAndControlUV =  ...
        fcn_BoundedAStar_pathCalculationBackToStart(...
        endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (-1));
end
fast_method = toc;

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

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

%% BUG case: found error in expandReachabilityWithWind case 10007 (fixed)

figNum = 90001;
titleString = sprintf('BUG case: found error in expandReachabilityWithWind case 10007 (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

load('BUG_90001_fcn_BoundedAStar_pathCalculationBackToStart.mat',...
    'thisGoal','cellArrayOfExpansions','cellArrayOfIntermediateCalculations');

%thisGoal = [2 2];
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    thisGoal, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);


%% BUG case: found error in expandReachabilityWithWind case 10007 #2 (fixed)

figNum = 90002;
titleString = sprintf('BUG case: found error in expandReachabilityWithWind case 10007 #2 (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

load('BUG_90002_fcn_BoundedAStar_pathCalculationBackToStart.mat',...
    'thisGoal','cellArrayOfExpansions','cellArrayOfIntermediateCalculations');

%thisGoal = [2 2];
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    thisGoal, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

%% BUG case: found error in expandReachabilityWithWind case 10007 #3 (fixed)

figNum = 90003;
titleString = sprintf('BUG case: found error in expandReachabilityWithWind case 10007 #3 (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

load('BUG_90003_fcn_BoundedAStar_pathCalculationBackToStart.mat',...
    'thisGoal','cellArrayOfExpansions','cellArrayOfIntermediateCalculations');

%thisGoal = [2 2];
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    thisGoal, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

%% BUG case: found error in expandReachabilityWithWind case 10007 #4 (fixed)

figNum = 90004;
titleString = sprintf('BUG case: found error in expandReachabilityWithWind case 10007 #4 (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

load('BUG_90004_fcn_BoundedAStar_pathCalculationBackToStart.mat',...
    'thisGoal','cellArrayOfExpansions','cellArrayOfIntermediateCalculations');

%thisGoal = [2 2];
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    thisGoal, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

%% BUG case: found error in expandReachabilityWithWind case 10007 #5 (fixed)

figNum = 90005;
titleString = sprintf('BUG case: found error in expandReachabilityWithWind case 10007 #5 (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

load('BUG_90005_fcn_BoundedAStar_pathCalculationBackToStart.mat',...
    'thisGoal','cellArrayOfExpansions','cellArrayOfIntermediateCalculations');

%thisGoal = [2 2];
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    thisGoal, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);

%% BUG case: found error in expandReachabilityWithWind case 10007 #6 (fixed)

figNum = 90006;
titleString = sprintf('BUG case: found error in expandReachabilityWithWind case 10007 #6 (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

load('BUG_90006_fcn_BoundedAStar_pathCalculationBackToStart.mat',...
    'thisGoal','cellArrayOfExpansions','cellArrayOfIntermediateCalculations');

%thisGoal = [2 2];
pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    thisGoal, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));

% Check variable types
assert(isnumeric(pathXYAndControlUV));

% Check variable sizes
assert(size(pathXYAndControlUV,1)>=2); 
assert(size(pathXYAndControlUV,2)==4);


%% Fail conditions
if 1==0
    %
       
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
function [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData
% Fill inputs
% 1 produces left to right, with edge patches that have right to left
% 2 produces left to right on top half, and top to bottom on bottom half
% 3 produces left to right on left half, bottom to top on right half
% 4 produces a saddle point near the middle
% 4822262 produces wind field that is mostly left to right

randomSeed = 4;

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
