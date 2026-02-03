% script_test_fcn_BoundedAStar_expandReachabilityWithWind
% Tests: fcn_BoundedAStar_expandReachabilityWithWind

% Revision history
% 2026_02_03 by Kaelea Hayes
% -- first write of test script, using
%    script_test_fcn_BoundedAStar_expandReachabilityWithWind as starter

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


%% DEMO case: create realistic wind field 
figNum = 10001;
titleString = sprintf('DEMO case: create realistic wind field ');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Wind field parameters
XY_range = [];
NpointsInSide =[];
windMagnitude = [];
randomSeed = [];

% Call function
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_BoundedAStar_realisticWindField((XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (figNum));

% Check variable types
assert(isnumeric(normalizedEastWind));
assert(isnumeric(normalizedNorthWind));
assert(isnumeric(windFieldX));
assert(isnumeric(windFieldY));

% Check variable sizes
assert(size(normalizedEastWind,1)==200); 
assert(size(normalizedEastWind,2)==200); 
assert(size(normalizedNorthWind,1)==200); 
assert(size(normalizedNorthWind,2)==200); 
assert(length(windFieldX) == 200);
assert(length(windFieldY) == 200);

% Check variable values
% (too difficult - randomly generated)

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TV
Nsteps=3;
for ith_step = 1:Nsteps

    % % Resample top Nrand values
    % % Increasing this number causes objects to "disappear" more as they
    % % progress in time
    % Nrand = 20;
    % seedVector = reshape(seedMap,[],1);
    % [~,sortedRandInd] = sort(seedVector,'descend');
    % seedVector(sortedRandInd(1:Nrand),1) = rand(Nrand,1);
    % seedMap = reshape(seedVector,nRows,mColumns);

    %%%%%%%%%%%
    % Change the map slightly to "evolve" the seeds
    % Resample Nrand values
    Nrand = 20;
    randomThreshold = Nrand/Nseeds;
    randomChange = rand(nRows,mColumns);
    indicesToChange = find(randomChange<randomThreshold);
    seedMap(indicesToChange) = rand(length(indicesToChange),1);

    %%%%%%%%%%%
    % Randomly walk sideways
    percentageSideways = mod(movementSideways,1); % A value between 0 and 1
    columnsSideways = floor(movementSideways);

    % Move the percentage
    if percentageSideways>0
        % Do not walk last columns, and refill first columns
        randomChange = rand(nRows,mColumns);
        indicesChange = find(randomChange<percentageSideways);
        indicesChange = indicesChange(indicesChange<(nRows*(mColumns-1)));
        seedMap(indicesChange+nRows) = seedMap(indicesChange);
        firstColumnChanged = find(indicesChange<=nRows);
        seedMap(firstColumnChanged) = rand(length(firstColumnChanged),1);
    end

    % Move the columns
    if columnsSideways>0
        randomChange = rand(nRows,columnsSideways);
        seedMap = [randomChange seedMap(:,1:(mColumns-columnsSideways))];
    end


    %%%%%%%%%%%
    % Update the map based on changed seed
    % Call the function once to initialize settings for upcoming calls
    [~, randomMatrixDilated, updatedThreshold, leftDilationMultiplier, rightDilationMultiplier] = ...
        fcn_GridMapGen_generateRandomOccupancyMap(...
        'seedMap', (seedMap),... % [1x1] integer to be a random seed or NxM matrix of random numbers
        'leftDilationMultiplier', (leftDilationMultiplier),... %  [nRows nRows], ...
        'rightDilationMultiplier', (rightDilationMultiplier),... % [mCols mCols], ...
        'thresholdForced', (forcedThreshold), ... % [1x1] scalar
        'flagSkipThresholdOptimization',(0),...% [1x1] scalar
        'figNum',(-1));
    % 'mapSize', (mapSize),... % [nRows mCols])
    % 'occupancyRatio',(occupancyRatio),... % [1x1] value between 0 and 1
    forcedThreshold = 0.9*forcedThreshold + 0.1*updatedThreshold;

    %%%%
    % Rescale the colors to integer values

    % Filter the color minimum and max values
    keep = 0.8;
    colorMin = (keep)*colorMin + (1-keep)*min(randomMatrixDilated,[],"all");
    colorMax = (keep)*colorMax + (1-keep)*max(randomMatrixDilated,[],"all");


    rescaledToColorIntegers = fcn_INTERNAL_rescaleToColors(randomMatrixDilated, colorMin, colorMax, numColors);
    clf;
    image(rescaledToColorIntegers);
    hold on;
    % contour(randomMatrixDilated,50,'k-','Linewidth',0.2);

    %%%%
    % Use the gradient to estimate wind direction
    [px,py] = gradient(randomMatrixDilated);

    if 1==0
        % Check results
        xAxisValues = (1:mColumns);
        yAxisValues = (1:nRows);
        figure(1234);
        clf;

        image(rescaledToColorIntegers);
        hold on;

        contour(xAxisValues,yAxisValues,randomMatrixDilated);
        hold on
        quiver(xAxisValues,yAxisValues,px,py)
        axis equal;
    end

    % Uncomment to show that there's a bug in the code below. Need to
    % figure out why derivative function does not match gradient!
    if 1==0
        [rowDerivative, colDerivative, leftDilationMultiplier, rightDilationMultiplier] = ...
            fcn_GridMapGen_dilateDerivativeByN(randomMatrixDilated, 1, ...
            ([]), ([]), (-1));
        figure;
        contour(xAxisValues,yAxisValues,randomMatrixDilated);
        hold on;
        quiver(xAxisValues,yAxisValues,rowDerivative, colDerivative);
        disp([px(1:10,1:10); nan(1,10); colDerivative(1:10,1:10)])
    end


    % Set the wind vectors so they align with the contours. NOTE: this
    % corresponds to a -90 degree rotation, e.g.
    % rotated = original*[0 -1; 1 0]. However, the axes are reversed in
    % image format, so have to add a minus sign.
    eastWind  = py;
    northWind = -px;

    if 1==0
        % Check results
        quiver(xAxisValues,yAxisValues,eastWind,northWind)
    end

    %%%%
    % Solve for the wind magnitude
    windMagnitude = (eastWind.^2+northWind.^2).^0.5;
    maxWind = max(windMagnitude,[],'all');
    normalizedWindMagnitude = windMagnitude./maxWind;
    normalizedEastWind = eastWind./maxWind;
    normalizedNorthWind = northWind./maxWind;

    if 1==0
        % Plot the result. This plot shows that the contour lines are tight in
        % areas where the wind is highest. This is typical of weather.
        figure(4575);
        clf;
        image(normalizedWindMagnitude*256);
        hold on;
        contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
        axis equal
        quiver(xAxisValues,yAxisValues,normalizedEastWind,normalizedNorthWind)
    end

    % For debugging. This shows that the wind magnitude and wind directions
    % follow the contour lines.
    if 1==0
        figure(5757);
        clf;

        subplot(2,2,1);
        image(rescaledToColorIntegers);
        title('Pressure regions');
        hold on;
        contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
        colormap(gca,cmap);

        subplot(2,2,2);
        EastWindColors = fcn_INTERNAL_rescaleToColors(eastWind, [], [], numColors);
        image(EastWindColors);
        title('EastWind','FontSize',10);
        hold on;
        contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
        colormap(gca,cmap);

        subplot(2,2,3);
        northWindColors = fcn_INTERNAL_rescaleToColors(northWind, [], [], numColors);
        image(northWindColors);
        title('NorthWind','FontSize',10);
        hold on;
        contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
        colormap(gca,cmap);

        subplot(2,2,4);
        windMagColors = fcn_INTERNAL_rescaleToColors(windMagnitude, [], [], numColors);
        image(windMagColors);
        title('windMagnitude','FontSize',10);
        hold on;
        contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
        colormap(gca,cmap);

    end

    %%%%%
    % Extract all the individual contour XY coordinates
    % [M,c] = contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
    M = contourc(randomMatrixDilated,Ncontours);

    levels = []; % The level number for each contour
    coordinates = cell(1,1); % The XY coordinates for each contour
    cellArrayOfWindMagnitudes = cell(1,1);
    cellArrayOfEastWind = cell(1,1);
    cellArrayOfNorthWind = cell(1,1);
    allPointsXY = []; % The XY coordinates for all coordinates, separated by NaN values
    allWindMagnitudes = []; % The wind magnitudes for all coordinates, separated by NaN values
    allEastWind = [];
    allNorthWind = [];

    indexM = 1;
    numContours = 0;
    longestIndex = 0;
    longestContourLength = 0;
    while indexM < size(M,2)
        numPointsThisContour = M(2,indexM); % Number of points in this contour segment
        xData = M(1,indexM+(1:numPointsThisContour)); % x coordinates
        yData = M(2,indexM+(1:numPointsThisContour)); % y coordinates
        pointsXY = [xData' yData'];
        thisContourIndices = max(1,floor(pointsXY));
        thisXind = thisContourIndices(:,1);
        thisYind = thisContourIndices(:,2);
        indices = sub2ind(mapSize,thisYind,thisXind);
        thisMagnitude = normalizedWindMagnitude(indices);
        thisEastWind  = normalizedEastWind(indices);
        thisNorthWind = normalizedNorthWind(indices);

        % Save results
        numContours = numContours+1;
        levels(numContours,1) = M(1,indexM); %#ok<SAGROW>
        coordinates{numContours,1} = pointsXY;
        cellArrayOfWindMagnitudes{numContours,1} = thisMagnitude;
        cellArrayOfEastWind{numContours,1} = thisEastWind;
        cellArrayOfNorthWind{numContours,1} = thisNorthWind;

        allPointsXY = [allPointsXY; nan nan; pointsXY]; %#ok<AGROW>
        allWindMagnitudes = [allWindMagnitudes; nan; thisMagnitude]; %#ok<AGROW>
        allEastWind = [allEastWind; nan; thisEastWind]; %#ok<AGROW>
        allNorthWind = [allNorthWind; nan; thisNorthWind]; %#ok<AGROW>

        % Save longest
        if numPointsThisContour>longestContourLength
            longestContourLength = numPointsThisContour;
            longestIndex = numContours;
        end

        % Move down the columns
        indexM = indexM + numPointsThisContour + 1; % Move to the next contour segment
    end

    % For debugging - plot longest contour to show plotting command works
    if 1==0
        figure(5858);
        clf;

        image(rescaledToColorIntegers);
        ylabel('Rows');
        xlabel('Columns');
        title('Individual contour test');
        hold on;
        % contour(randomMatrixDilated,50,'k-','Linewidth',0.2);
        axis equal
        colormap(gca,cmap);
        % quiver(xAxisValues,yAxisValues,normalizedEastWind,normalizedNorthWind)

        longestContourXY = coordinates{longestIndex};

        % Plot the longest contour
        % plot(longestContourXY(:,1),longestContourXY(:,2),'w-','LineWidth',1);

        % % Quiver on largest contour only
        % quiver(coordinates{longestIndex}(:,1),coordinates{longestIndex}(:,2), cellArrayOfEastWind{longestIndex,1}, cellArrayOfNorthWind{longestIndex,1},'w-','LineWidth',2);

        % Plot only large winds
        winds = cellArrayOfWindMagnitudes{longestIndex,1};
        eastW = cellArrayOfEastWind{longestIndex,1};
        northW = cellArrayOfNorthWind{longestIndex,1};
        smallWinds = winds<0.4;
        limitedContour = longestContourXY;
        limitedContour(smallWinds,1) = nan;
        limitedContour(smallWinds,2) = nan;
        plot(limitedContour(:,1),limitedContour(:,2),'w-','LineWidth',1);

        % Put arrows on one random point that's not nan valued
        valuesToPlot = fcn_INTERNAL_breakDataByNaNs([limitedContour eastW northW winds]);
        quiver(valuesToPlot(:,1),valuesToPlot(:,2),valuesToPlot(:,3)*5,valuesToPlot(:,4)*5,0,'LineWidth',1,'Color',[1 1 1],'MaxHeadSize',10);

    end

    % Plot only large winds
    smallWinds = allWindMagnitudes<0.2;
    limitedContours = allPointsXY;
    limitedContours(smallWinds,1) = nan;
    limitedContours(smallWinds,2) = nan;
    plot(limitedContours(:,1),limitedContours(:,2),'w-','LineWidth',1);

    % Put arrows on one random point that's not nan valued
    valuesToPlot = fcn_INTERNAL_breakDataByNaNs([limitedContours allEastWind allNorthWind allWindMagnitudes]);
    quiver(valuesToPlot(:,1),valuesToPlot(:,2),valuesToPlot(:,3)*5,valuesToPlot(:,4)*5,0,'LineWidth',1,'Color',[1 1 1],'MaxHeadSize',10);

    %%%%
    % Steer seedmap back to start?
    if 1==flag_blendEndToStart
        remainingSteps = (Nsteps-ith_step);
        if remainingSteps<=NblendingSteps
            seedMap =  remainingSteps/NblendingSteps*seedMap + (NblendingSteps-remainingSteps)/NblendingSteps*initialSeedMap;
        end
    end

    drawnow; % Ensure the plot is updated on the screen

    if 1==flag_saveAnimatedGif

        % Capture the frame and save it
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if flag_firstDraw == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
            flag_firstDraw = 0;
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end
    end
end

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
% 
% % Call function
% [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
%     radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (figNum));

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 1;
cellArrayOfWindExitConditions = [];

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), ...
    (flagWindRoundingType), (cellArrayOfWindExitConditions), ([]));

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

% Call function
[reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (-1));

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

Niterations = 5;

% Slow mode
tic;
for ith_test = 1:Niterations
    % Call function
    [reachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), ([]));
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
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (cellArrayOfWindExitConditions), (-1));
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