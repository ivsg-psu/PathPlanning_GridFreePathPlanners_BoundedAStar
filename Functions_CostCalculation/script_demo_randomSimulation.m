%% particles drifting in wind without control input

figNum = 10001;
fig_num = figNum;
% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData(3);

% Call graph generation function
radius = 1;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];

% Call function
% [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
%     radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (-1));

NTrials =100;
% startPoints = (-1) + (2).*rand(100,2);
trajectory = cell(NTrials,1);

for i = 1:NTrials
    startPoint = startPoints(i,:);
    timeLength = 20;
    trajectory{i} = fcn_BoundedAStar_streamlineFollowing(startPoint, timeLength, windFieldX, windFieldY, normalizedEastWind, normalizedNorthWind, (-1));
end


figure(fig_num)
fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, 'default', fig_num)
hold on
box on
axis([-10 10 -10 10])

for i = 1:NTrials
    plot(trajectory{i}(:,1), trajectory{i}(:,2), 'b', 'LineWidth', 2)
end


%% with randomly chosen input trajectories
close all
radius = 0.5;
NTrials = 1000;
randDir = (2*pi)*rand(NTrials,1);
% trajInput = radius*[cos(randDir) sin(randDir)];
finishPoint = [5, -5];

% startPointsx = (-1) + (2).*rand(100,1);
% startPointsy = (-1) + (2).*rand(100,1);
% startPoints = [startPointsx startPointsy];
startPoints = zeros(NTrials,2);
trajectory = cell(NTrials,1);

trajectory2 = cell(NTrials, 1);

for i = 1:NTrials
    startPoint = startPoints(i,:);
    timeLength = 13;
    dir = (2*pi)*rand(timeLength,1);
    trajInputr = radius*[cos(dir) sin(dir)];
    trajectory2{i} = fcn_BoundedAStar_simulateIndividualTrajectory(startPoint,finishPoint, trajInputr, windFieldX, windFieldY, windFieldU, windFieldV, -1);
end

figure(fig_num)
fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, 'default', fig_num)
hold on
box on
axis([-10 10 -10 10])

for i = 1:NTrials
    plot(trajectory2{i}(:,1), trajectory2{i}(:,2), 'b', 'LineWidth', 1)
end


% find final point of each trajectory
endPt = nan*ones(NTrials,2);
for n = 1:NTrials 
    endPt(n,:) = trajectory2{n}(end,:);
    % if endPt(n,1) > 10
    %     endPt(n,1) = 10;
    % elseif endPt(n,1) < -10
    %     endPt(n,1) = -10;
    % end
    % if endPt(n,2) > 10
    %     endPt(n,2) = 10;
    % elseif endPt(n,2) < -10
    %     endPt(n,2) = -10;
    % end
end

plot(endPt(:,1), endPt(:,2),'.r','MarkerSize',25)


%% gifmaking

load('setExpansion.mat');

ntSteps = length(reachableSetData);

figure(fig_num)
hold on;
box on;
axis([-10 10 -10 10]);
    

for n = 2:ntSteps
    for i = 1:NTrials
        startPoint = startPoints(i,:);
        timeLength = n;
        dir = (2*pi)*rand(timeLength,1);
        trajInputr = radius*[cos(dir) sin(dir)];
        trajectory2{i,n} = fcn_BoundedAStar_simulateIndividualTrajectory(startPoint,finishPoint, trajInputr, windFieldX, windFieldY, windFieldU, windFieldV, -1);
        endPt(i,:) = trajectory2{i,n}(end,:);
    end
    figure(fig_num)
    hold on;
    box on;
    axis([-10 10 -10 10]);
    fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, 'default',fig_num);
    plot(reachableSetData{n}(:,1), reachableSetData{n}(:,2),'-w','Linewidth',2,'DisplayName','Estimate of reachable set');
    plot(endPt(:,1), endPt(:,2), 'xg', 'MarkerSize', 5, 'DisplayName', 'Simulated trajectory location at time step');
    legend('Location','southeast')
    exportgraphics(gcf,'randomsim.gif','Append',true);
    drawnow
    if n ~= ntSteps
        clf
    end
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