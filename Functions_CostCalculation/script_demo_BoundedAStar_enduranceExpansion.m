%% Overview

% Most mission parameters are taken from Wikipedia:
% https://en.wikipedia.org/wiki/General_Atomics_MQ-9_Reaper#Specifications

% The RAMP platform model contains two state variables:
% 1) Mass State (fuel)
% 2) Temperature State (fuel temp)

% It is assumed that if the aircraft is asked to go from one point to the
% next, it can do it. The input to the dynamic model is effectively then an
% altitude, velocity, and time to represent those flight conditions.
% Additionally, an operation mode is defined (which affects the heat load
% applied)

% Inputs
route.alt_ft = 25000; % notional altitude (ft) https://en.wikipedia.org/wiki/General_Atomics_MQ-9_Reaper#Specifications
route.v_ftps = 284; % notional velocity (ft/s) https://en.wikipedia.org/wiki/General_Atomics_MQ-9_Reaper#Specifications
t = 5280/route.v_ftps; % notional time (s) (notional distance to next waypoint, 5280ft)
route.mode = 'High'; % 'High' or 'Low'

% The platform is given some initial conditions at the start of the route
% segment (initial conditions here for no flight, but should be updated
% between segments)
Platform.MassState = 4000; % fuel weight lb https://en.wikipedia.org/wiki/General_Atomics_MQ-9_Reaper#Specifications
Platform.TempState = 15; % fuel temp C (fuel assumed to be at steady state for standard day initially)

% The aerodynamic model is a standard drag polar model
% CD = CDv + k(CL-CLv)^2
% The Platform object has an airframe subclass to store necessary
% constants.
Platform.Airframe.emptyWeight = 4900; % empty weight lb https://en.wikipedia.org/wiki/General_Atomics_MQ-9_Reaper#Specifications
Platform.Airframe.CDv = 0.0384; % dimensionless https://www.researchgate.net/publication/354100611_Estimation_of_Drag_Polar_for_ABT-18_Unmanned_Aerial_Vehicle#read
Platform.Airframe.CLv = 0.0384; % paper actually didn't have this so just repeated CDv here (not actually the same all the time)
Platform.Airframe.k = 0.10201; % dimensionless https://www.researchgate.net/publication/354100611_Estimation_of_Drag_Polar_for_ABT-18_Unmanned_Aerial_Vehicle#read
Platform.Airframe.Sref = 200; % reference area ft^2 wingspan ~66 ft (wikepedia), eyeball guess chord ~3 ft, assume Sref = span*chord ~= 200 ft^2

% The platform also has an atmosphere model incorporated as a function of
% altitude. Initialize.
Platform.Ambient.T_C = []; % C
Platform.Ambient.rho_slgpft3 = []; % slug/ft^3

% The heat load is dependent on the route operation mode ("low" and "high")
Platform.HeatLoad.Low = 5; % kW (ARBITRARILY CHOSEN VALUE)
Platform.HeatLoad.High = 20; % kW (ARBITRARILY CHOSEN VALUE)

% Specify TMS properties
Platform.TMS.InletArea_ft2 = 0.0347; % ft^2 (Sized to make nominal operation cooling 10kW)
Platform.TMS.Treject_C = 30; % C (ARBITRARILY CHOSEN)
Platform.TMS.cp_air_kJpkgK = 1; % kJ/kg-K - known property of air
Platform.TMS.cp_fuel_kJpkgK = 2; % kJ/kg-K - based on Jet A

% Set up parameters for passing into function
costRad = 40;
modelParams.Platform = Platform;
modelParams.route = route;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 1;

% [reachableSet, cellArrayOfIntermediateCalculations, newStates] = fcn_BoundedAStar_reachabilityWithInputsNEW(costRad, windFieldU, windFieldV, windFieldX, windFieldY, modelParams, [], [], 10001);

[finalReachableSet, exitCondition, cellArrayOfExitInfo, newStates] = ...
    fcn_BoundedAStar_expandReachabilityWithModel(costRad, ...
    windFieldU, windFieldV, windFieldX, windFieldY, modelParams, [], [], [], 0, 10001)



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



