% script_test_fcn_BoundedAStar_pathCalculation
% Tests: fcn_BoundedAStar_pathCalculation

% Revision history
% 2025_08_18 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_pathCalculation
%   % * first write of script 
%   % * using script_test_fcn_BoundedAStar_reachabilityWithInputs as 
%   %   % starter

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

%% DEMO case: calculate unit path, starting from origin, 5 steps
figNum = 10001;
titleString = sprintf('DEMO case: calculate unit path, starting from origin, 5 steps');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
Nsteps = 5;
controlInputs = ones(Nsteps,2);

% Call function
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, controlInputs, ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pathXY));

% Check variable sizes
assert(size(pathXY,1)==Nsteps+1); 
assert(size(pathXY,2)==2);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: calculate unit path, starting from origin, 2 control inputs
figNum = 10002;
titleString = sprintf('DEMO case: calculate unit path, starting from origin, 2 control inputs');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0; 0 0];
Nsteps = 5;
controlInputs = cell(2,1);
controlInputs{1,1} =  ones(Nsteps,2);
controlInputs{2,1} = -ones(Nsteps-2,2);

% Call function
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, controlInputs, ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(pathXY));

% Check variable sizes
assert(size(pathXY{1},1)==Nsteps+1); 
assert(size(pathXY{1},2)==2);
assert(size(pathXY{2},1)==Nsteps-1); 
assert(size(pathXY{2},2)==2);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: calculate unit path, starting from origin, 2 control inputs, 2 starting points
figNum = 10003;
titleString = sprintf('DEMO case: calculate unit path, starting from origin, 2 control inputs, 2 starting points');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0; -1 -2];
Nsteps = 5;
controlInputs = cell(2,1);
controlInputs{1,1} =  ones(Nsteps,2);
controlInputs{2,1} = -ones(Nsteps-2,2);

% Call function
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, controlInputs, ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(pathXY));

% Check variable sizes
assert(size(pathXY{1},1)==Nsteps+1); 
assert(size(pathXY{1},2)==2);
assert(size(pathXY{2},1)==Nsteps-1); 
assert(size(pathXY{2},2)==2);

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

%% TEST case: Drop a point on a streamline
figNum = 20001;
titleString = sprintf('TEST case: Drop a point on a streamline');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
Nsteps = 5;
controlInputs = zeros(Nsteps,2);

% Call function
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, controlInputs, ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pathXY));

% Check variable sizes
assert(size(pathXY,1)==Nsteps+1); 
assert(size(pathXY,2)==2);

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
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
Nsteps = 5;
controlInputs = ones(Nsteps,2);

% Call function
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, controlInputs, ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    ([]));

% Check variable types
assert(isnumeric(pathXY));

% Check variable sizes
assert(size(pathXY,1)==Nsteps+1); 
assert(size(pathXY,2)==2);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
Nsteps = 5;
controlInputs = ones(Nsteps,2);

% Call function
pathXY = ...
    fcn_BoundedAStar_pathCalculation(...
    startPoints, controlInputs, ...
    windFieldU,  ...
    windFieldV,  ...
    windFieldX,  ...
    windFieldY,  ...
    (-1));

% Check variable types
assert(isnumeric(pathXY));

% Check variable sizes
assert(size(pathXY,1)==Nsteps+1); 
assert(size(pathXY,2)==2);

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
maxWindSpeed = 4;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;

startPoints = [0 0];
Nsteps = 5;
controlInputs = ones(Nsteps,2);

Niterations = 10;

% Slow mode
tic;
for ith_test = 1:Niterations
    % Call function
    pathXY = ...
        fcn_BoundedAStar_pathCalculation(...
        startPoints, controlInputs, ...
        windFieldU,  ...
        windFieldV,  ...
        windFieldX,  ...
        windFieldY,  ...
        ([]));
end
slow_method = toc;

% Check variable types
assert(isnumeric(pathXY));

% Check variable sizes
assert(size(pathXY,1)==Nsteps+1); 
assert(size(pathXY,2)==2);

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call function
    pathXY = ...
        fcn_BoundedAStar_pathCalculation(...
        startPoints, controlInputs, ...
        windFieldU,  ...
        windFieldV,  ...
        windFieldX,  ...
        windFieldY,  ...
        (-1));
end
fast_method = toc;

% Check variable types
assert(isnumeric(pathXY));

% Check variable sizes
assert(size(pathXY,1)==Nsteps+1); 
assert(size(pathXY,2)==2);

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

%% BUG case: expansion glitching observed due to bug in Path library (fixed)
% 
% figNum = 90001;
% titleString = sprintf('BUG case: expansion glitching observed due to bug in Path library (fixed)');
% fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
% figure(figNum); clf;


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

