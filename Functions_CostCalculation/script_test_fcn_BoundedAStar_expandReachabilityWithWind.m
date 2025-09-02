% script_test_fcn_BoundedAStar_expandReachabilityWithWind
% Tests: fcn_BoundedAStar_expandReachabilityWithWind

% Revision history
% 2025_07_29 by S. Brennan, sbrennan@psu.edu
% - first write of script 
%   % * using script_test_fcn_BoundedAStar_matrixEnvelopeExpansion as 
%   %   % starter
%
% 2025_08_04 by S. Brennan, sbrennan@psu.edu
% - in script_test_fcn_BoundedAStar_expandReachabilityWithWind
%   % * added exit condition outputs and tests
%   % * added cellArrayOfExitInfo output info and tests
%   % * added cellArrayOfWindExitConditions inputs and tests in TEST section
%
% 2025_08_11 by S. Brennan, sbrennan@psu.edu
% - in script_test_fcn_BoundedAStar_expandReachabilityWithWind
%   % * added demo cases for outputting exact paths. See demo 10007
%
% 2025_08_17 by S. Brennan, sbrennan@psu.edu
% - in script_test_fcn_BoundedAStar_expandReachabilityWithWind
%   % * added demo/test cases for outputting exact paths. See test 90001
%
% 2025_08_25 by S. Brennan, sbrennan@psu.edu
% - in script_test_fcn_BoundedAStar_expandReachabilityWithWind
%   % * added demo/test cases for outputting exact paths. See test 90002


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


%% DEMO case: get reachable envelope in wind field (starting point at origin, light wind)
figNum = 10001;
titleString = sprintf('DEMO case: get reachable envelope in wind field (starting point at origin, light wind)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 1;
maxWindSpeed = 0.5;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, ...
    (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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

%% DEMO case: get reachable envelope in wind field (starting point at origin, strong wind)
figNum = 10002;
titleString = sprintf('DEMO case: get reachable envelope in wind field (starting point at origin, strong wind)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 0.3;
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

sgtitle(titleString, 'Interpreter','none');

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

%% DEMO case: get reachable envelope in strong wind field (starting point at origin, strong wind, seed = 3)
figNum = 10003;
titleString = sprintf('DEMO case: get reachable envelope in strong wind field (starting point at origin, strong wind, seed = 3)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(3);

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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


%% DEMO case: get reachable envelope in matched wind field (starting point at origin, seed = 3)
figNum = 10004;
titleString = sprintf('DEMO case: get reachable envelope in matched wind field (starting point at origin, seed = 3)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(3);

% Call graph generation function
radius = 0.7;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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

%% DEMO case: matched wind field, goal points specified
figNum = 10005;
titleString = sprintf('DEMO case: matched wind field, goal points specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(3);

% Call graph generation function
radius = 0.7;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

xRange = windFieldX(end)-windFieldX(1);
yRange = windFieldY(end)-windFieldY(1);

Ngoals = 30;
rng(1);
allGoalPointsList = rand(Ngoals,2).*[xRange yRange] + ones(Ngoals,1)*[windFieldX(1) windFieldY(1)];

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 50; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = allGoalPointsList;  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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

%% DEMO case: TSP test case
figNum = 10006;
titleString = sprintf('DEMO case:  TSP test case');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(4);

% Call graph generation function
radius = 0.4;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [-6 6];
flagWindRoundingType = 1;

% xRange = windFieldX(end)-windFieldX(1);
% yRange = windFieldY(end)-windFieldY(1);

% Ngoals = 30;
% rng(1);
% allGoalPointsList = rand(Ngoals,2).*[xRange yRange] + ones(Ngoals,1)*[windFieldX(1) windFieldY(1)];
allGoalPointsList = [0 0; 6 6; 8 -4; -6 -6];

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 100; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = []; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = allGoalPointsList;  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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


%% DEMO case: matched wind field, goal points specified
figNum = 10007;
titleString = sprintf('DEMO case: output of exact paths');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(3);

% Call graph generation function
radius = 0.7;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

xRange = windFieldX(end)-windFieldX(1);
yRange = windFieldY(end)-windFieldY(1);

Ngoals = 30;
rng(1);
allGoalPointsList = rand(Ngoals,2).*[xRange yRange] + ones(Ngoals,1)*[windFieldX(1) windFieldY(1)];

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 50; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = allGoalPointsList;  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo, ...
    reachableSetExactCosts, cellArrayOfReachableSetPaths] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), ...
    (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));
    
% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(exitCondition));
assert(iscell(cellArrayOfExitInfo));
assert(isnumeric(reachableSetExactCosts));
assert(iscell(cellArrayOfReachableSetPaths));


% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(exitCondition,1)==1); 
assert(size(exitCondition,2)==1);
assert(isequal(size(cellArrayOfExitInfo),[2 1]));
assert(size(reachableSetExactCosts,1)>=3); 
assert(size(reachableSetExactCosts,2)==1);
assert(size(cellArrayOfReachableSetPaths,1)>=3);
assert(size(cellArrayOfReachableSetPaths,2)==1);

% Check variable values
% (too difficult - randomly generated)

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% DEMO case: time varying wind field capabilities
figNum = 10008;
titleString = sprintf('DEMO case:  time varying wind field capabilities');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load time varying wind field 
load('TVwindField.mat');

% Call graph generation function
radius = 0.5;

windFieldU = windFieldUk{1};
windFieldV = windFieldVk{1};

% FIX
windFieldX = linspace(-10, 10, length(windFieldU));
windFieldY = linspace(-10, 10, length(windFieldV));

startPoints = [0 0];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, ...
    (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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

%% TEST case: testing cellArrayOfWindExitConditions, Nsteps = 5
figNum = 20001;
titleString = sprintf('TEST case: testing cellArrayOfWindExitConditions, Nsteps = 5');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadZeroWindData;

% Call graph generation function
radius = 1;
maxWindSpeed = 0.5;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 5; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(exitCondition,1));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: testing cellArrayOfWindExitConditions, exits due to flagStopIfEntireFieldCovered
figNum = 20002;
titleString = sprintf('TEST case: testing cellArrayOfWindExitConditions, exits due to flagStopIfEntireFieldCovered');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadZeroWindData;

% Call graph generation function
radius = 1;
maxWindSpeed = 0.5;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 20; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(exitCondition,2));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: testing cellArrayOfWindExitConditions, exits due to toleranceToStopIfSameResult
figNum = 20003;
titleString = sprintf('TEST case: testing cellArrayOfWindExitConditions, exits due to toleranceToStopIfSameResult');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadSideWindData;
delta = windFieldX(2)-windFieldX(1);

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 20; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 4*delta; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(exitCondition,1));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: testing cellArrayOfWindExitConditions, exits due to allGoalPointsList
figNum = 20004;
titleString = sprintf('TEST case: testing cellArrayOfWindExitConditions, exits due to allGoalPointsList');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadZeroWindData;

% Call graph generation function
radius = 1;
maxWindSpeed = 0.5;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 20; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [1 1; 2 2; 3 3];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(exitCondition,4));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: testing cellArrayOfWindExitConditions, exits due to flagStopIfHitOneGoalPoint
figNum = 20005;
titleString = sprintf('TEST case: testing cellArrayOfWindExitConditions, exits due to flagStopIfHitOneGoalPoint');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadZeroWindData;

% Call graph generation function
radius = 1;
maxWindSpeed = 0.5;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 20; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 0.2; % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [1 1; 2 2; 3 3];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 1;   % flagStopIfHitOneGoalPoint
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(exitCondition,5));

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

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

% % Call graph generation function
% radius = 1;
% maxWindSpeed = 0.5;
% 
% windFieldU = normalizedEastWind*maxWindSpeed;
% windFieldV = normalizedNorthWind*maxWindSpeed;
% startPoints = [0 0];
% flagWindRoundingType = 1;
% cellArrayOfWindExitConditions = [];
% flagTimeVarying = [];
% 
% % Call function
% [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
%     radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), ...
    (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), ([]));

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
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (-1));

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
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];
flagTimeVarying = [];

Niterations = 5;

% Slow mode
tic;
for ith_test = 1:Niterations
    % Call function
    [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), ([]));
end
slow_method = toc;

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

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call function
    [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (-1));
end
fast_method = toc;

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

%% BUG case: Demo case 10001 from fcn_BoundedAStar_solveTSPwithWind 
figNum = 90001;
titleString = sprintf('BUG case: Demo case 10001 from fcn_BoundedAStar_solveTSPwithWind');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
load('BUG_90001_fcn_BoundedAStar_expandReachabilityWithWind.mat',...
    'radius', 'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY', ...
    'startPoint', 'flagWindRoundingType','cellArrayOfWindExitConditions');

flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoint), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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

%% BUG case: Demo case 10001 from fcn_BoundedAStar_solveTSPwithWind #2
figNum = 90002;
titleString = sprintf('BUG case: Demo case 10001 from fcn_BoundedAStar_solveTSPwithWind #2');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
load('BUG_90002_fcn_BoundedAStar_expandReachabilityWithWind.mat',...
    'radius', 'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY', ...
    'startPoint', 'flagWindRoundingType','cellArrayOfWindExitConditions');

flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoint), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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

%% BUG case: Demo case 10008 from fcn_BoundedAStar_findTSPCostsAndPaths #1
figNum = 90003;
titleString = sprintf('BUG case: Demo case 10008 from fcn_BoundedAStar_findTSPCostsAndPaths #1');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
load('BUG_90003_fcn_BoundedAStar_expandReachabilityWithWind.mat',...
    'radius', 'windFieldU', 'windFieldV', 'windFieldX', 'windFieldY', ...
    'startPoint', 'flagWindRoundingType','cellArrayOfWindExitConditions');

flagTimeVarying = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo, ...
            reachableSetExactCosts, cellArrayOfReachableSetPaths] =  fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, ...
    (startPoint), (flagWindRoundingType), (cellArrayOfWindExitConditions), (flagTimeVarying), (figNum));

sgtitle(titleString, 'Interpreter','none');

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