function [windFieldU, windFieldV, windFieldX, windFieldY] = fcn_BoundedAStar_generateTimeVaryingWindField(nSteps, optStruct, varargin)
% fcn_BoundedAStar_generateWindGraph
% generates cell arrays containing wind field vector information at a
% specified number of time steps
%
% FORMAT:
% windGraph = fcn_BoundedAStar_generateTimeVaryingWindField(nSteps, optStruct, (flagDoLoop), (fig_num))
%
% INPUTS:
%     
%     nSteps: a scalar indicating the number of time steps that should be
%     generated for the wind field
%
%     optStruct: a structure containing options for wind field generation, 
%     where any fields not specified will be assigned a default value. has
%     fields:
%       mapSize: a 1x2 vector containing the desired number of rows and
%           columns in the wind field generation. This number decides the
%           complexity of the wind field and is not related to its actual size.
%           Lower numbers create more uniform wind fields. Defaults to [50
%           50].
%       movementSideways: a scalar indicating the speed at which the
%           'weather system' moves from the 'western' part of the map to
%           the 'eastern' part of the map. Defaults to 1.
%       nBlendingSteps: a scalar indicating the number of steps to use in
%           blending the wind field movement back to the start. Used only
%           for wind fields where the looping flag is enabled. Defaults to
%           10.
%       occupancyRatio: a scalar indicating the occupied ratio of the
%           grid. Defaults to 0.2.
%       dilationLevel: a scalar that directly changes the 'zoom' of the
%           wind field. Higher numbers 'zoom' into the wind field
%           pheonomena further than lower numbers. Defaults to 400.
%       randomSeed: a scalar used as the random seed for wind field
%           generation. If -1 is passed, the random seed is not specified.
%           Defaults to 1. 
%
%     (optional inputs)
%
%     flagDoLoop: a flag indicating whether or not the time varying wind
%     field should loop back to its initial set. Defaults to 1. 
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     windFieldU:  a cell array of matrices containing the u-direction components of the
%     wind velocity at each grid point
%
%     windFieldV:  a cell array of matrices containing the v-direction components of the
%     wind velocity at each grid point
%
%     windFieldX: a vector containing the x values assigned to each grid point
% 
%     windFieldY: a vector containing the y values assigned to each grid point
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%     fcn_DebugTools_generateRandomOccupancyMap 
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_generateTimeVaryingWindField
% for a full test suite. 
%
% This function was written on 2025_09_02 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_09_02 by K. Hayes
% -- first write of function using fcn_BoundedAStar_generateWindGraph as
%    starter. function main content based on existing script in MapGen
%    toolbox script_demo_generateRandomOccupancyAnimated.m
%    

%
% TO-DO
% -- add plotting/animation support
% -- save to .mat file automatically with description of parameters
%    required for generation

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 4; % The largest Number of argument inputs to the function
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
        narginchk(2,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        % fcn_DebugTools_checkInputsToFunctions(...
        %     radius, '1column_of_numbers',[1 1]);
    end
end

% Check fields in optStruct and unpack them for fcn use
if ~isfield(optStruct, 'mapSize')
    mapSize = [50 50];
else
    mapSize = optStruct.mapSize;
end

if ~isfield(optStruct, 'movementSideways')
    movementSideways = 1;
else
    movementSideways = optStruct.movementSideways;
end

if ~isfield(optStruct, 'nBlendingSteps')
    nBlendingSteps = 10;
else
    nBlendingSteps = optStruct.nBlendingSteps;
end

if ~isfield(optStruct, 'occupancyRatio')
    occupancyRatio = 0.2;
else
    occupancyRatio = optStruct.occupancyRatio;
end

if ~isfield(optStruct, 'dilationLevel')
    dilationLevel = 400;
else
    dilationLevel = optStruct.dilationLevel;
end

if ~isfield(optStruct, 'randomSeed')
    randomSeed = 1;
else
    randomSeed = optStruct.randomSeed;
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        fig_num = temp;
        figure(fig_num);
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

fig_num = 1111;
figure(fig_num); clf;
numColors = 256;
cmap = turbo(numColors);
colormap(cmap);

h_fig = figure(fig_num);
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
    % Prep the figure for plotting
    temp_h = figure(fig_num);
    flag_rescale_axis = 0;
    if isempty(get(temp_h,'Children'))
        flag_rescale_axis = 1;
    end      
    
    % Is this 2D or 3D?
    dimension_of_points = 2; 

    % % Find size of plotting domain
    % allPointsBeingPlotted = [vertices; nan nan];
    % 
    % max_plotValues = max(allPointsBeingPlotted);
    % min_plotValues = min(allPointsBeingPlotted);
    % sizePlot = max(max_plotValues) - min(min_plotValues);
    % nudge = sizePlot*0.006; %#ok<NASGU>
    % 
    % % Find size of plotting domain
    % if flag_rescale_axis
    %     percent_larger = 0.3;
    %     axis_range = max_plotValues - min_plotValues;
    %     if (0==axis_range(1,1))
    %         axis_range(1,1) = 2/percent_larger;
    %     end
    %     if (0==axis_range(1,2))
    %         axis_range(1,2) = 2/percent_larger;
    %     end
    %     if dimension_of_points==3 && (0==axis_range(1,3))
    %         axis_range(1,3) = 2/percent_larger;
    %     end
    % 
    %     % Force the axis to be equal?
    %     if 1==1
    %         min_valuesInPlot = min(min_plotValues);
    %         max_valuesInPlot = max(max_plotValues);
    %     else
    %         min_valuesInPlot = min_plotValues;
    %         max_valuesInPlot = max_plotValues;
    %     end
    % 
    %     % Stretch the axes
    %     stretched_min_vertexValues = min_valuesInPlot - percent_larger.*axis_range;
    %     stretched_max_vertexValues = max_valuesInPlot + percent_larger.*axis_range;
    %     axesTogether = [stretched_min_vertexValues; stretched_max_vertexValues];
    %     newAxis = reshape(axesTogether, 1, []);
    %     axis(newAxis);
    % 
    % end
    goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end

    hold on;
    grid on;

    % Plot the graph nodes
    plot(vertices(:,1),vertices(:,2),'.','MarkerSize',10,'DisplayName','Nodes')

    % Plot graph edges with colors according to their cost in the cost
    % graph

    % Figure out scale of cost graph
    min_cost  = min(min(costgraph));    % should be == 0 if previous scaling was correct
    max_cost = max(max(costgraph));

    % costScale = max_cost - min_cost;
    % percentVec = 0:0.1:1;
    % 
    % colorBreakpoints = costScale*percentVec;
    colorBreakpoints = linspace(min_cost,max_cost,11);
    colorMap = turbo(11);
    colormap(colorMap)

    hold on
        
    % Plot streamlines
    obj = streamslice(x,y,windFieldU,windFieldV);
    set(obj,'Color',[0.597 0.597 0.597], 'LineWidth', 0.5)

    % Determine which edges to plot
    beelineVector = [(finish(1)-start(1)), (finish(2)-start(2))];     % 'beeline' vector from start to finish
    unitVector = beelineVector/sum(beelineVector.^2).^0.5;            % unit vector matching 'beeline' vector

    % Plot edges
    for i = 1:n_nodes+2
        for j = 1:n_nodes+2
            if edges(i,j) == 1
                % Calculate dot product of edge vector and beeline unit
                % vector
                thisVector = [vertices(j,1) - vertices(i,1), vertices(j,2) - vertices(i,2)];
                dotProd = dot(unitVector,thisVector);
    
               if dotProd > 0 
                    coloridx = find(colorBreakpoints>=costgraph(i,j),1,'first');
                    thiscolor = colorMap(coloridx,:);
                    plot([vertices(i,1), vertices(j,1)], [vertices(i,2), vertices(j,2)],'-','Color',thiscolor,'LineWidth',3)
               end
            end
        end
    end

    cb = colorbar;
    cb.Label.String = 'wind-based cost';
    cb.TickLabels = colorBreakpoints;
        
        
    %legend('Interpreter','none');
    xlabel('X-East');
    ylabel('Y-North');
    title('Calculated Cost of Edges (moving towards goal)')

    axis(goodAxis);
    axis equal;

    % Shut the hold off?
    if flag_shut_hold_off
        hold off;
    end

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
    meanValue = mean(randomMatrixDilated,'all','omitmissing');
    stdValue = std(randomMatrixDilated,0,'all','omitmissing');
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

