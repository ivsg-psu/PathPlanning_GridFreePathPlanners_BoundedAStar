function [allWindFieldU, allWindFieldV, allWindFieldX, allWindFieldY] = fcn_BoundedAStar_realisticTimeVarying(varargin)
% fcn_BoundedAStar_realisticTimeVarying
% fills in a windField grid with random [U V] components (updated from
% fillWindField) and steps them randomly 'sideways' on the map
%
% FORMAT:
% [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_BoundedAStar_realisticTimeVarying( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (figNum))
%
% INPUTS:
%
%     (optional inputs)
%
%     XY_range: a 1x4 vector in the "axis" format of [Xmin Ymin Xmax Ymax]
%     that defines the extent of the grid in X and Y. 
%
%     NpointsInSide: The number of partitions on each side of the grid. For
%     example, NpointsInSide=10 produces a 10x10 grid
%
%     windMagnitude: the largest value of wind to use, in knots
%  
%     randomSeed: the random number generation seed to use
%   
%     Nsteps: number of steps to generate in maps
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     normalizedEastWind: a nxn matrix containing the u-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     normalizedNorthWind: a nxn matrix containing the v-direction components of the
%     wind velocity at each grid point, where n is equal to NpointsInSide
%
%     windFieldX: a 1xn vector containing the x values assigned to each grid point
%     within the specfied XY_range
%
%     windFieldY: a 1xn vector containing the y values assigned to each grid point
%     within the specified XY_range
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%     fcn_GridMapGen_generateRandomOccupancyMap
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_realisticWindField
% for a full test suite.
%
% This function was written on 2026_02_03 by Kaelea Hayes
% Questions or comments? contact kaeleahayes@psu.edu

% REVISION HISTORY:
% 2026_02_03 by Kaelea Hayes
% -- first write of function using fcn_BoundedAStar_realisticWindField as a
%    starter

% TO-DO
% -- add hardcoded occupancy grid parameters as inputs

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 6; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS");
    MATLABFLAG_MAPGEN_FLAG_DO_DEBUG = getenv("MATLABFLAG_MAPGEN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
end

%% check input arguments?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if 0==flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(0,MAX_NARGIN);
        
        %%%%% No required inputs, delete these?
        % % Check the XY_range input, make sure it is '4column_of_numbers'
        % % type with exactly 1 row
        % fcn_DebugTools_checkInputsToFunctions(...
        %     XY_range, '4column_of_numbers',[1 1]);
        % 
        % % Check the radius input, make sure it is '1column_of_numbers'
        % % type, 1 row
        % fcn_DebugTools_checkInputsToFunctions(...
        %     radius, '1column_of_numbers',[1 1]);
    end
end

% Does user want to specify the XY_range input?
XY_range = [-10 -10 10 10]; % Default is 10 (units) away from origin in each direction
if 1 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        XY_range = temp;
    end
end

% Does user want to specify the NpointsInSide input?
NpointsInSide = 200; % Default is 200 on each side
if 2 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        NpointsInSide = temp;
    end
end

% Does user want to specify the NpointsInSide input?
windMagnitude = 1; % Default is nondimensional 1
if 3 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        windMagnitude = temp;
    end
end

% Does user want to specify the randomSeed input?
randomSeed = 4; % Default is 4
if 4 <= nargin
    temp = varargin{4};
    if ~isempty(temp)
        randomSeed = temp;
    end
end

% Does user want to specify the Nsteps input?
Nsteps = 10; % Default is 10
if 5 <= nargin
    temp = varargin{5};
    if ~isempty(temp)
        nSteps = temp;
    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        figure(figNum);
        flag_do_plots = 1;
    end
end

%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%See: http://patorjk.com/software/taag/#p=display&f=Big&t=Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

% Set random seed
rng(randomSeed);

% Set map size
mapSize = [NpointsInSide NpointsInSide];
nRows = mapSize(1);
mColumns = mapSize(2);

% Set other hardcoded parameters (to be updated to function inputs at a
% later date, but hardcoded for now)
occupancyRatio = 0.2;
dilationLevel = 6400;
seedMap = rand(nRows,mColumns);
initialSeedMap = seedMap;
Nseeds = numel(seedMap);
leftDilationMultiplier = [];
rightDilationMultiplier = [];
optimizedThreshold = [];
flag_firstDraw = 1;

% Other hardcoded parameters
movementSideways = 1;
Ncontours = 30;
flag_blendEndToStart = 1; % causes the animation to "loop"
NblendingSteps = 10;

flag_saveAnimatedGif = 1;
filename = 'script_demo_generateRandomOccupancyAnimated.gif';
delayTime = 0.1; % Delay between frames in seconds

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
    'figNum',(-1));

colorMin = min(randomMatrixDilated,[],"all");
colorMax = max(randomMatrixDilated,[],"all");

numColors = 256;
cmap = turbo(numColors);
colormap(cmap);

h_fig = figure(figNum);
set(h_fig,'Name','animatedRandom','NumberTitle','off'); %, 'Position',[684 85 592 317]);

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


%     % Set the wind vectors so they align with the contours. NOTE: this
%     % corresponds to a -90 degree rotation, e.g.
%     % rotated = original*[0 -1; 1 0]. However, the axes are reversed in
%     % image format, so have to add a minus sign.
%     eastWind  = py;
%     northWind = -px;

    if 1==0
        % Check results
        quiver(xAxisValues,yAxisValues,eastWind,northWind)
    end

    % Use the gradient to estimate wind direction
    [px,py] = gradient(randomMatrixDilated);
    eastWind  = py;
    northWind = -px;

    % Solve for the wind magnitude
    wMag = (eastWind.^2+northWind.^2).^0.5;
    maxWind = max(wMag,[],'all');
    normalizedWindMagnitude = wMag./maxWind;
    normalizedEastWind = eastWind./maxWind;
    normalizedNorthWind = northWind./maxWind;
    
    windFieldX = linspace(XY_range(1), XY_range(3), nRows);
    windFieldY = linspace(XY_range(2), XY_range(4), mColumns);
    
    windFieldU = normalizedEastWind*windMagnitude;
    windFieldV = normalizedNorthWind*windMagnitude;

    allWindFieldU{ith_step} = windFieldU;
    allWindFieldV{ith_step} = windFieldV;
    allWindFieldX{ith_step} = windFieldX;
    allWindFieldY{ith_step} = windFieldY;

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
        thisEastWind  = windFieldU(indices);
        thisNorthWind = windFieldV(indices);

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

%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if flag_do_plots
% 
%     figure(figNum);
%     clf;
% 
%     % Check to see if hold is already on. If it is not, set a flag to turn it
%     % off after this function is over so it doesn't affect future plotting
%     flag_shut_hold_off = 0;
%     if ~ishold
%         flag_shut_hold_off = 1;
%         hold on
%     end   
% 
%     % Turn on legend
%     legend('Interpreter','none','Location','best');
% 
%     % Plot wind field
%     cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU, windFieldV,windFieldX,windFieldY,'default',figNum);
%     set(cellArrayOfPlotHandles{3},'Color',[0.6 0.6 0.6]);

end % Ends the flag_do_plot if statement

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end


end % Ends the main function



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

%% fcn_INTERNAL_rescaleToColors
function rescaledToColorIntegers = fcn_INTERNAL_rescaleToColors(randomMatrixDilated, colorMin, colorMax, numColors)
flag_useMean = 1;

if 1==flag_useMean
    meanValue = mean(randomMatrixDilated,'all','omitnan');
    stdValue = std(randomMatrixDilated,0,'all','omitnan');
end
if isempty(colorMin)
    if 0==flag_useMean
        colorMin = min(randomMatrixDilated,[],'all');
    else
        colorMin = meanValue - 2*stdValue;
    end
end
if isempty(colorMax)
    if 0==flag_useMean
        colorMax = max(randomMatrixDilated,[],'all');
    else
        colorMax = meanValue + 2*stdValue;
    end
end

colorInterval = (colorMax-colorMin)/(numColors-1); % The interval represents the "jump" between different colors
offsetRemovedMatrix = randomMatrixDilated - colorMin; % Remove the offset
countMatrix = floor(offsetRemovedMatrix./colorInterval)+1;

% Make sure output is between 1 and numColors
rescaledToColorIntegers = min(max(countMatrix,1),numColors);

end % Ends fcn_INTERNAL_rescaleToColors

%% fcn_INTERNAL_breakDataByNaNs
function valuesToPlot = fcn_INTERNAL_breakDataByNaNs(inputData)
% Finds "chunks" of data separated by nan values. In each chunk, randomly
% picks one row, and saves it to plot. This code is used to select where to
% put "arrowheads" on the contours, since putting them everywhere makes the
% plot very messy

% [limitedContour winds eastW northW]
ith_value = 1;
keepGoing = 1;
Nfound = 0;
remainder = inputData;
while 1==keepGoing
    nextNan = find(isnan(remainder(:,1)),1,'first');
    if isempty(nextNan)
        keepGoing = 0;
        dataToProcess = remainder(ith_value:end,:);
    else
        dataToProcess = remainder(ith_value:(nextNan-1),:);
    end
    if nextNan == length(remainder(:,1))
        keepGoing = 0;
    else
        remainder = remainder(nextNan+1:end,:);
    end
    if ~isempty(dataToProcess)
        goodData = dataToProcess(~isnan(dataToProcess(:,1)),1);
        if length(goodData)>3
            Nfound = Nfound+1;
            randomIndex = round(length(goodData)*rand);
            randomIndex = max(1,min(length(goodData),randomIndex));
            valuesToPlot(Nfound,:) = dataToProcess(randomIndex,:); %#ok<AGROW>
        end
    end
end
end