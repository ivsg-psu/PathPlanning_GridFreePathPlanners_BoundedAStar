% script_test_fcn_BoundedAStar_generateWindGraph
% Tests: fcn_BoundedAStar_generateWindGraph

% Revision history
% 2025_07_16 by K. Hayes, kxh1031@psu.edu
% -- first write of script
% 2025_07_22 by K. Hayes
% -- changed wind field plotting to use automatic wind field plotting
%    function
% -- added fcn_GridMapGen_generateRandomOccupancyMap for more realistic wind maps

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

%% DEMO case: create a graph within a randomly generated wind field
fig_num = 10001;
titleString = sprintf('DEMO case: create a graph within a randomly generated wind field');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [0 0 1 1];
peaksMode = [];
n_nodes = 5;

% Call wind field generation function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode),(-1));

% Call graph generation function
start = [0, 0.3 , n_nodes+1, -1, 0];
finish = [0.8, 0.8, n_nodes+2, -1, 0];
[vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, n_nodes, start, finish, (randomSeed), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vertices));
assert(isnumeric(edges));
assert(isnumeric(costgraph));

% Check variable sizes
Npoints = n_nodes+2;
assert(size(vertices,1)==Npoints); 
assert(size(vertices,2)==2); 

assert(size(edges,1)==Npoints); 
assert(size(edges,2)==Npoints); 

assert(size(costgraph,1)==Npoints);
assert(size(costgraph,2)==Npoints);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% DEMO case: create a graph within a realistic wind field
fig_num = 10002;
%titleString = sprintf('DEMO case: create a graph within a realistic wind field');
fprintf(1,'Figure %.0f: %s\n',fig_num) %,titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [0 0 1 1];
peaksMode = [];
n_nodes = 5;

% Call random occupancy map function - code taken from
% script_demo_generateRandomOccupancyAnimated
nRows = 200;
mColumns = 200;
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

colorMin = min(randomMatrixDilated,[],"all");
colorMax = max(randomMatrixDilated,[],"all");

figure(99); clf;
numColors = 256;
cmap = turbo(numColors);
colormap(cmap);

h_fig = figure(99);
%set(h_fig,'Name','animatedRandom','NumberTitle','off'); %, 'Position',[684 85 592 317]);

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
normalizedEastWind = eastWind./maxWind;
normalizedNorthWind = northWind./maxWind;

%%%%

% Call graph generation function
x = linspace(XY_range(1), XY_range(3), nRows);
y = linspace(XY_range(2), XY_range(4), mColumns);
start = [0, 0.3 , n_nodes+1, -1, 0];
finish = [0.8, 0.8, n_nodes+2, -1, 0];
[vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(eastWind, northWind, x, y, n_nodes, start, finish, (randomSeed), (fig_num));

% sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vertices));
assert(isnumeric(edges));
assert(isnumeric(costgraph));

% Check variable sizes
Npoints = n_nodes+2;
assert(size(vertices,1)==Npoints); 
assert(size(vertices,2)==2); 

assert(size(edges,1)==Npoints); 
assert(size(edges,2)==Npoints); 

assert(size(costgraph,1)==Npoints);
assert(size(costgraph,2)==Npoints);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% DEMO case: use A* planner to find a path between start and finish
fig_num = 10003;
titleString = sprintf('DEMO case: use A* planner to find a path between two nodes');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [0 0 1 1];
peaksMode = 1;
n_nodes = 25;
rngSeed = [];

% Call wind field generation function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode),(-1));

% Call graph generation function
start = [0, 0.9 , n_nodes+1, -1, 0];
finish = [0.9, 0, n_nodes+2, -1, 0];

[vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, n_nodes, start, finish, (rngSeed), (fig_num));

% Call A*
%hvec = sum((vertices - finish(1:2)).^2,2).^0.5';

% Use calculated costs to go from any node to end node as heuristic
% function
hvec = costgraph(:,end)';
all_pts = [vertices(1:n_nodes,:), [1:n_nodes]',-1*ones(n_nodes,1), zeros(n_nodes,1)];

[cost, route] = fcn_algorithm_Astar(edges, costgraph, hvec, all_pts, start, finish);

    
    % Plot wind field automatically 
    fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, x, y, 'default', (99));

    figure(99)
    hold on
    plot(vertices(:,1),vertices(:,2),'o','MarkerSize',5,'MarkerFaceColor','white','MarkerEdgeColor','black','DisplayName','Nodes')
    plot(route(:,1),route(:,2),'Color','black','Linewidth',3,'DisplayName','Route')
    plot(start(1),start(2),'x','Color','red','MarkerSize',10,'LineWidth',3,'DisplayName','Start')
    plot(finish(1),finish(2),'x','Color','green','MarkerSize',10,'LineWidth',3,'DisplayName','Goal')

    xlabel('X-East');
    ylabel('Y-North');

    axis equal;
    legend('Location','best');

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vertices));
assert(isnumeric(edges));
assert(isnumeric(costgraph));

% Check variable sizes
Npoints = n_nodes;
assert(size(vertices,1)==Npoints+2); 
assert(size(vertices,2)==2); 

assert(size(edges,1)==Npoints+2); 
assert(size(edges,2)==Npoints+2); 

assert(size(costgraph,1)==Npoints+2);
assert(size(costgraph,2)==Npoints+2);

% Make sure plot opened up
% assert(isequal(get(gcf,'Number'),fig_num));

%% DEMO case: use A* planner to find a path between start and finish
fig_num = 10004;
titleString = sprintf('DEMO case: use A* planner to find a path between two nodes');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = 4822262;
windMagnitude = [];
NpointsInSide = [];
XY_range = [0 0 1 1];
peaksMode = [];
n_nodes = 10;
rngSeed = [];

% Call random occupancy map function - code taken from
% script_demo_generateRandomOccupancyAnimated
rng(1996)
nRows = 16;
mColumns = 16;
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

colorMin = min(randomMatrixDilated,[],"all");
colorMax = max(randomMatrixDilated,[],"all");

figure(99); clf;
numColors = 256;
cmap = turbo(numColors);
colormap(cmap);

h_fig = figure(99);
%set(h_fig,'Name','animatedRandom','NumberTitle','off'); %, 'Position',[684 85 592 317]);

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

[vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(normalizedEastWind, normalizedNorthWind, x, y, n_nodes, start, finish, (randomSeed), (fig_num));

% Call A*
%hvec = sum((vertices - finish(1:2)).^2,2).^0.5';

% Use calculated costs to go from any node to end node as heuristic
% function
hvec = costgraph(:,end)';
all_pts = [vertices(1:n_nodes,:), [1:n_nodes]',-1*ones(n_nodes,1), zeros(n_nodes,1)];

[cost, route] = fcn_algorithm_Astar(edges, costgraph, hvec, all_pts, start, finish);

    
    % Plot wind field automatically 
    fcn_BoundedAStar_plotWindField(normalizedEastWind, normalizedNorthWind, x, y, 'default', (99));

    figure(99)
    hold on
    plot(vertices(:,1),vertices(:,2),'o','MarkerSize',5,'MarkerFaceColor','white','MarkerEdgeColor','black','DisplayName','Nodes')
    plot(route(:,1),route(:,2),'Color','black','Linewidth',3,'DisplayName','Route')
    plot(start(1),start(2),'x','Color','red','MarkerSize',10,'LineWidth',3,'DisplayName','Start')
    plot(finish(1),finish(2),'x','Color','green','MarkerSize',10,'LineWidth',3,'DisplayName','Goal')

    xlabel('X-East');
    ylabel('Y-North');

    axis equal;
    legend('Location','best');

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vertices));
assert(isnumeric(edges));
assert(isnumeric(costgraph));

% Check variable sizes
Npoints = n_nodes;
assert(size(vertices,1)==Npoints+2); 
assert(size(vertices,2)==2); 

assert(size(edges,1)==Npoints+2); 
assert(size(edges,2)==Npoints+2); 

assert(size(costgraph,1)==Npoints+2);
assert(size(costgraph,2)==Npoints+2);

% Make sure plot opened up
% assert(isequal(get(gcf,'Number'),fig_num));
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

%% TEST case: Non-default, non-uniform XY range for random map generation
fig_num = 20001;
titleString = sprintf('TEST case: Non-default, non-uniform XY range for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;



%% TEST case: Non-default NpointsInSide for random map generation
fig_num = 20002;
titleString = sprintf('TEST case: Non-default NpointsInSide for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;



%% TEST case: Non-default windMagnitude for random map generation
fig_num = 20003;
titleString = sprintf('TEST case: Non-default windMagnitude for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;



%% TEST case: Non-default randomSeed for random map generation
fig_num = 20004;
titleString = sprintf('TEST case: Non-default randomSeed for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;



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
fig_num = 80001;
fprintf(1,'Figure: %.0f: FAST mode, empty fig_num\n',fig_num);
figure(fig_num); close(fig_num);

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
assert(~any(figHandles==fig_num));


%% Basic fast mode - NO FIGURE, FAST MODE
fig_num = 80002;
fprintf(1,'Figure: %.0f: FAST mode, fig_num=-1\n',fig_num);
figure(fig_num); close(fig_num);

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
assert(~any(figHandles==fig_num));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
fig_num = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',fig_num);
figure(fig_num);
close(fig_num);

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
assert(~any(figHandles==fig_num));

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
assert(~any(figHandles==fig_num));


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
