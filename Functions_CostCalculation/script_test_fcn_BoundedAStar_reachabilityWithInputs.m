% script_test_fcn_BoundedAStar_reachabilityWithInputs
% Tests: fcn_BoundedAStar_reachabilityWithInputs

% Revision history
% 2025_07_29 by S. Brennan, sbrennan@psu.edu
% - first write of script 
%   % * using script_test_fcn_BoundedAStar_matrixEnvelopeExpansion as 
%   %   % starter

% TO DO:
% -- take wind post-processing out of this script and put it into another
%    separate function


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

%% DEMO case: get reachable envelope in wind field
figNum = 10001;
titleString = sprintf('DEMO case: get reachable envelope in wind field');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 5;
maxWindSpeed = 10;
windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
windFieldX = linspace(XY_range(1), XY_range(3), nRows);
windFieldY = linspace(XY_range(2), XY_range(4), mColumns);
startPoints = [0 0; 1 2];

% Call function
reachableSet = fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(expandedSets));

% Check variable sizes
Npoints = 629;
assert(size(expandedSets,1)==Npoints); 
assert(size(expandedSets,2)==2);
assert(size(expandedSets,3)==11);

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

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), ([]));

% Check variable types
assert(isnumeric(windFieldU));
assert(isnumeric(windFieldV));
assert(isnumeric(x));
assert(isnumeric(y));

% Check variable sizes
Npoints = 200;
assert(size(windFieldU,1)==Npoints); 
assert(size(windFieldU,2)==Npoints); 

assert(size(windFieldV,1)==Npoints); 
assert(size(windFieldV,2)==Npoints); 

assert(size(x,1)==1);
assert(size(x,2)==Npoints);

assert(size(y,1)==1);
assert(size(y,2)==Npoints);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

% Check variable types
assert(isnumeric(windFieldU));
assert(isnumeric(windFieldV));
assert(isnumeric(x));
assert(isnumeric(y));

% Check variable sizes
Npoints = 200;
assert(size(windFieldU,1)==Npoints); 
assert(size(windFieldU,2)==Npoints); 

assert(size(windFieldV,1)==Npoints); 
assert(size(windFieldV,2)==Npoints); 

assert(size(x,1)==1);
assert(size(x,2)==Npoints);

assert(size(y,1)==1);
assert(size(y,2)==Npoints);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

Niterations = 10;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));
end
fast_method = toc;

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
function [normalizedEastWind, normalizedNorthWind] = fcn_INTERNAL_loadExampleData
% Fill inputs
randomSeed = 4822262;

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

end % Ends fcn_INTERNAL_loadExampleData