%% DEMO case: use A* planner to find a path between start and finish
fig_num = 10004;
titleString = sprintf('DEMO case: use A* planner to find a path between two nodes');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = 4822262;
windMagnitude = [];
NpointsInSide = [];
XY_range = [-10 -10 10 10];
peaksMode = [];
n_nodes = 10;
rngSeed = [];

% Call random occupancy map function - code taken from
% script_demo_generateRandomOccupancyAnimated
rng(1996)
nRows = 40;
mColumns = 40;
mapSize = [nRows mColumns];

Nsteps = 50;
Ncontours = 30;
movementSideways = 1; % 0.6; %.5; %2.3;

occupancyRatio = 0.2;
dilationLevel = 400;
seedMap = rand(nRows,mColumns);
initialSeedMap = seedMap;
Nseeds = numel(seedMap);
leftDilationMultiplier = [];
rightDilationMultiplier = [];
optimizedThreshold = [];
flag_firstDraw = 1;

% Call the function once to initialize settings for upcoming calls
[occupancyMatrix, randomMatrixDilated, forcedThreshold, leftDilationMultiplier, rightDilationMultiplier] = ...
    fcn_GridMapGen_generateRandomOccupancyMap(...
    'mapSize', (mapSize),... % [nRows mCols])
    'occupancyRatio',(occupancyRatio),... % [1x1] value between 0 and 1
    'dilationLevel',(dilationLevel),.... % [1x1] strictly positive int
    'seedMap', (seedMap),... % [1x1] integer to be a random seed or NxM matrix of random numbers
    'leftDilationMultiplier', (leftDilationMultiplier),... %  [nRows nRows], ...
    'rightDilationMultiplier', (rightDilationMultiplier),... % [mCols mCols], ...
    'thresholdForced', (optimizedThreshold), ... % [1x1] scalar
    'flagSkipThresholdOptimization',(0),...% [1x1] scalar
    'figNum',(15456));


%%%% WIND POST-PROCESSING SHOULD BE MADE INTO AN ALTERNATE FUNCTION (?). FOR
%%%% NOW THE CODE IS JUST COPIED.
% Use the gradient to estimate wind direction
[px,py] = gradient(randomMatrixDilated);
eastWind  = py;
northWind = -px;

% Solve for the wind magnitude
windMagnitude = (eastWind.^2+northWind.^2).^0.5;
maxWind = max(windMagnitude,[],'all');
normalizedWindMagnitude = windMagnitude./maxWind;
normalizedEastWind = 1*eastWind./maxWind;
normalizedNorthWind = 1*northWind./maxWind;

%%%%

% Call graph generation function
x = linspace(XY_range(1), XY_range(3), nRows);
y = linspace(XY_range(2), XY_range(4), mColumns);
start = [0.2, 0.9 , n_nodes+1, -1, 0];
finish = [0.6, 0.1, n_nodes+2, -1, 0];

% Call set expansion methods
radius = 7.5;
centerPoint = [0 0];
expandedSets = fcn_BoundedAStar_setExpansion(radius,normalizedEastWind,normalizedNorthWind,x,y, (centerPoint), (-1));

% Plot expanded sets
figure(fig_num)
%hold on
axis([XY_range(1), XY_range(3), XY_range(2), XY_range(4)])
for j = 1:size(expandedSets,3)
    axis equal
    hold on
    grid on
    streamslice(x,y,normalizedEastWind,normalizedNorthWind)
    plot(expandedSets(:,1,j),expandedSets(:,2,j),'LineWidth',2,'Color','black')
    drawnow
    if j == size(expandedSets,3)
        break
    end
    clf
end
