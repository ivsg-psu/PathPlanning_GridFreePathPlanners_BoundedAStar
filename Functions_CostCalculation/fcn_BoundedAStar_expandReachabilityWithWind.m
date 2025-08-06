function [finalReachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(radius, windFieldU, windFieldV, windFieldX, windFieldY, varargin)
% fcn_BoundedAStar_expandReachabilityWithWind
% performs iterative reachability expansion from a startPoint until one of
% the following criteria are met:
%     1. Maximum number of iterations reached
%     2. Entire wind field is covered by finalReachableSet
%     3. The expansion stalls where each iteration is giving "same" result
%     4. All user-given goal points are hit
%     5. One user-given goal points is hit
%     Each of these, or combinations, can be specified by the user. See the
%     flagWindExitConditions input.
%
% FORMAT:
% [finalReachableSet, exitCondition, cellArrayOfExitInfo] = fcn_BoundedAStar_expandReachabilityWithWind(...
%     radius, ... 
%     windFieldU,  ...
%     windFieldV,  ...
%     windFieldX,  ...
%     windFieldY,  ...
%     (startPoints),  ...
%     (flagWindRoundingType),...
%     (cellArrayOfWindExitConditions),...
%     (figNum));
%
% INPUTS:
%
%     radius: a 1x1 scalar representing the radius of travel without wind.
%     This is usually the travel speed multiplied by the time step.
%
%     windFieldU:  a matrix containing the u-direction components of the
%     wind velocity at each grid point
%
%     windFieldV:  a matrix containing the v-direction components of the
%     wind velocity at each grid point
%
%     windFieldX: a vector containing the x values assigned to each grid
%     point
% 
%     windFieldY: a vector containing the y values assigned to each grid
%     point
%
%     (optional inputs)
%
%     startPoints: a Nx2 vector representing the [x,y] values of the start
%     point, or bounding envelope to start with. Defaults to [0,0] if no
%     value is entered
%
%     flagWindRoundingType: a flag to indicate how the final set is rounded
%     with respect to wind. Values include:
%
%          0: (default) indices into the wind matrix are removed so that
%          there are no repeats. This produces a bounding set whose
%          smoothness matches the wind field discretization. However, the
%          set values between discrete points in the wind field are not
%          returned.
%
%          1: all set outputs are returned. This produces step-wise wind
%          propogation, which represents how the set expansion truly
%          behaves with a discrete wind field. However, the set is usually
%          non-smooth.
%
%     cellArrayOfWindExitConditions: allows the user to specify the exit
%     conditions to monitor. These are specificed by a 5x1 cell array to
%     allow the user to set: 
%    
%         1. Nsteps: the maximum number of iterations (default is 100)
%    
%         2. flagStopIfEntireFieldCovered: stops expansion if entire wind field
%         is covered by finalReachableSet (default is 1) 
%    
%         3. toleranceToStopIfSameResult: the expansion is stopped  is giving
%         "same" result, where same means that the number of points in the
%         boundary is the same, and all points are within tolerance of prior
%         points. (default is 4*deltaX, where deltaX is windFieldX(2)-windFieldX(1))
%    
%         4. allGoalPointsList: an Nx2 array of user-given goal points that
%         must be hit before stopping the code. Default is empty.
%    
%         5. flagStopIfHitOneGoalPoint: if set to 1, stops if any
%         user-given goal points is hit. Default is 0.
%
%     If any condition is met, the expansion is stopped and the
%     exitCondition type is set accordingly.
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     finalReachableSet: a set of points defining the distance conversion of the
%     original travel locations to locations with wind
%
%     exitCondition: an integer listing which of the 5 exit conditions
%     activated.
%
%     cellArrayOfExitInfo: details on the exit condition, with following
%     cell contents:
%     1: number of iterations completed
%     2: goal points hit (as counts of sim steps to hit that point).
%     Returns empty if allGoalPointsList is empty.
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_expandReachabilityWithWind
% for a full test suite.
%
% This function was written on 2025_08_02 by S. Brennan
% Questions or comments? contact S. Brennan sbrennan@psu.edu 
% or K. Hayes, kxh1031@psu.edu

% REVISION HISTORY:
% 2025_08_02 by S. Brennan
% - first write of function using fcn_BoundedAStar_reachabilityWithInputs
%   % as a starter
% 2025_08_03 by S. Brennan;
% - In fcn_BoundedAStar_expandReachabilityWithWind
%   % * Added option to only update "jogs" when threshold is 120 degrees
%   % * Fixes bug seen in some map expansions
% 2025_08_04 by S. Brennan
% - In fcn_BoundedAStar_expandReachabilityWithWind
%   % * Added cellArrayOfWindExitConditions
%   % * Added exitCondition output information
%   % * Added cellArrayOfExitInfo output information
% 2025_08_05 by S. Brennan
%   % * Changed flag outputs to count sim steps to hit goal, not just
%   %   % binary

% TO-DO
% -- when wind is VERY high, higher than self speed, the set pushes away
% from the initial value, and thus the predicted set does not include
% previous sets. This can be fixed by the expansion step includeing the
% prior expansion. This is a bit tricky with the calculations, but would
% increase accuracy.


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 9; % The largest Number of argument inputs to the function
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
    debug_figNum = 999978; 
else
    debug_figNum = []; 
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
        narginchk(5,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        fcn_DebugTools_checkInputsToFunctions(...
            radius, '1column_of_numbers',[1 1]);
    end
end

% Does user want to specify startPoints input?
startPoints = [0 0]; % Default is origin
if 6 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        startPoints = temp;
    end
end

% Does user want to specify flagWindRoundingType input?
flagWindRoundingType = 0; % Default is 0
if 7 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        flagWindRoundingType = temp;
    end
end

% Does user want to specify flagWindRoundingType input?
% Defaults
cellArrayOfWindExitConditions = cell(5,1);
cellArrayOfWindExitConditions{1} = 100; % Nsteps
cellArrayOfWindExitConditions{2} = 1;   % flagStopIfEntireFieldCovered
cellArrayOfWindExitConditions{3} = 4*(windFieldX(2)-windFieldX(1)); % toleranceToStopIfSameResult
cellArrayOfWindExitConditions{4} = [];  % allGoalPointsList
cellArrayOfWindExitConditions{5} = 0;   % flagStopIfHitOneGoalPoint
if 8 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        cellArrayOfWindExitConditions = temp;
    end
end
Nsteps = cellArrayOfWindExitConditions{1};
flagStopIfEntireFieldCovered = cellArrayOfWindExitConditions{2};
toleranceToStopIfSameResult = cellArrayOfWindExitConditions{3};
allGoalPointsList = cellArrayOfWindExitConditions{4};
flagStopIfHitOneGoalPoint = cellArrayOfWindExitConditions{5};

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

originalStartPoints = startPoints;

% Get meshgrid for streamline plotting
[meshX,meshY] = meshgrid(windFieldX,windFieldY);

% Plot the wind field?
if flag_do_debug
    figure(debug_figNum);
    clf;
    hold on;
    
    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',debug_figNum)
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6])

    % Plot the start points
    plot(startPoints(:,1),startPoints(:,2),'.-','Color',[0 0 1],'MarkerSize',30,'DisplayName','Input: startPoints');

    if ~isempty(allGoalPointsList)
        plot(allGoalPointsList(:,1),allGoalPointsList(:,2),'.','Color',[1 0 1],'MarkerSize',40,'LineWidth', 2, 'DisplayName','Input: allGoalPointsList');
        
        % Label the points
        for ith_point = 1:length(allGoalPointsList(:,1))
            text(allGoalPointsList(ith_point,1),allGoalPointsList(ith_point,2),sprintf('%.0f',ith_point));
        end
    end
end

%%%%%
% Estimate number of steps to simulate
% XYvectors = [windFieldX' windFieldY'];
% maxXYs = max(XYvectors,[],1,'omitmissing');
% minXYs = min(XYvectors,[],1,'omitmissing');
% range = maxXYs - minXYs;
% longestPossibleDistance = sum(range.^2,2).^0.5;
% windMagnitude = (windFieldU.^2+windFieldV.^2).^0.5;
% maxWindSpeed = max(windMagnitude,[],'all','omitmissing');
% slowestPossibleSpeeds = radius - maxWindSpeed;
% if slowestPossibleSpeeds<0
%     warning(['The wind field is strong enough that some portions ' ...
%         'have wind faster than the fastest vehicle speed. This may ' ...
%         'produce unreachable areas and thus mission goals that are not ' ...
%         'feasible. For estimation, the flight speed will be used.']);
%     slowestPossibleSpeeds = radius; %#ok<NASGU>
% end
% maxNsteps = longestPossibleDistance/slowestPossibleSpeeds;

minX = windFieldX(1);
maxX = windFieldX(end);
minY = windFieldY(1);
maxY = windFieldY(end);
deltaX = windFieldX(2) - windFieldX(1);

allExpansions = cell(Nsteps,1);

if ~isempty(allGoalPointsList)
    stepsWhenGoalPointsHit = nan(length(allGoalPointsList(:,1)),1);
end

ith_step = 0;
flagContinueExpansion = 1;
while 1==flagContinueExpansion
    ith_step = ith_step+1;
    allExpansions{ith_step,1} = startPoints;

    % Call function to find reachable set on this time step
    reachableSet = fcn_BoundedAStar_reachabilityWithInputs(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (-1));

    if flag_do_debug && 1==1
        figure(debug_figNum);
        plot(reachableSet(:,1),reachableSet(:,2),'b.-','LineWidth',5, 'MarkerSize',30);
        title(sprintf('Expansion: %.0f',ith_step));
        axis([minX maxX minY maxY]);
        drawnow

    end

    % Remove "pinch" points?
    Nreachable = length(reachableSet(:,1));
    reachableSetNotPinched = fcn_Path_removePinchPointInPath(reachableSet(1:end-1,:),-1);
    % If more than half the points show up in the "pinch", then the pinch is
    % inverted. We want to keep the points NOT in the pinch. Use the "setdiff"
    % command to do this.
    if length(reachableSetNotPinched(:,1))< (Nreachable/2)
        reachableSetNotPinched = setdiff(reachableSet,reachableSetNotPinched,'rows','stable');
    end
    reachableSetNotPinched = [reachableSetNotPinched; reachableSetNotPinched(1,:)]; %#ok<AGROW>

    % Clean up set boundary from "jogs"?
    diff_angles = fcn_Path_calcDiffAnglesBetweenPathSegments(reachableSetNotPinched, -1);
    jogAngleThreshold = 120*pi/180;
    if any(abs(diff_angles)>jogAngleThreshold)
        noJogPoints = fcn_Path_cleanPathFromForwardBackwardJogs(reachableSetNotPinched, (jogAngleThreshold) , (-1));
    else
        noJogPoints = reachableSetNotPinched;
    end

    if flag_do_debug && 1==1
        figure(debug_figNum);
        plot(noJogPoints(:,1),noJogPoints(:,2),'c.-','LineWidth',3, 'MarkerSize',20);
        drawnow
    end

    % Downsample the set boundary
    startPointsSparse = fcn_INTERNAL_sparsifyPoints(noJogPoints,deltaX*0.5);

    if flag_do_debug && 1==1
        figure(debug_figNum);
        plot(startPointsSparse(:,1),startPointsSparse(:,2),'k.-','LineWidth',1, 'MarkerSize',10);
        drawnow
    end

    % Bound the XY values
    newStartPointsX =  max(min(startPointsSparse(:,1),maxX),minX);
    newStartPointsY =  max(min(startPointsSparse(:,2),maxY),minY);
    newStartPoints = [newStartPointsX newStartPointsY];

    if flag_do_debug && 1==1
        figure(debug_figNum);
        plot(newStartPoints(:,1),newStartPoints(:,2),'r.-','LineWidth',1, 'MarkerSize',5);
        drawnow
        pause(0.1);
    end

    % Save results for next loop
    startPoints = newStartPoints;

    % Check exit conditions
    if ith_step>=Nsteps
        flagContinueExpansion = 0;
        exitCondition = 1;
    end
    if 1==flagStopIfEntireFieldCovered
        valuesAtXEdges = startPointsSparse(:,1)>=maxX | startPointsSparse(:,1)<=minX;
        valuesAtYEdges = startPointsSparse(:,2)>=maxY | startPointsSparse(:,2)<=minY;
        valuesAtBothEdges = valuesAtXEdges | valuesAtYEdges;
        if all(valuesAtBothEdges,'all')
            flagContinueExpansion = 0;            
            exitCondition = 2;
        end
    end
    if ~isempty(toleranceToStopIfSameResult)
        if ith_step>2
            thisBoundary = allExpansions{ith_step,1};
            lastBoundary = allExpansions{ith_step-1,1};
            if length(thisBoundary(:,1))==length(lastBoundary(:,1))
                % Check distances
                differences = thisBoundary-lastBoundary;
                absDifferences = sum(differences.^2,2);

                if absDifferences<toleranceToStopIfSameResult^2
                    flagContinueExpansion = 0;
                    exitCondition = 3;
                end
            end
        end

    end
    if ~isempty(allGoalPointsList)
        goalPointsHit = fcn_INTERNAL_findGoalPointsHit(newStartPoints,allGoalPointsList);
        pointsFoundThisSimStep = find(isnan(stepsWhenGoalPointsHit) & goalPointsHit);
        if ~isempty(pointsFoundThisSimStep)
            stepsWhenGoalPointsHit(pointsFoundThisSimStep) = ith_step;
            if all(goalPointsHit==1,'all')
                flagContinueExpansion = 0;
                exitCondition = 4;
            end
            if 1==flagStopIfHitOneGoalPoint && any(goalPointsHit==1,'all')
                flagContinueExpansion = 0;
                exitCondition = 5;
            end
        end
    end

end

allExpansions{ith_step+1,1} = newStartPoints;

finalReachableSet = newStartPoints;

% Fill in cellArrayOfExitInfo
cellArrayOfExitInfo = cell(2,1);
cellArrayOfExitInfo{1,1} = ith_step+1;

if ~isempty(allGoalPointsList)
    goalPointsHit = stepsWhenGoalPointsHit;
else
    goalPointsHit = [];
end
cellArrayOfExitInfo{2,1} = goalPointsHit;


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
    temp_h = figure(figNum);
    flag_rescale_axis = 0;
    if isempty(get(temp_h,'Children'))
        flag_rescale_axis = 1;
    end      
    
    % Is this 2D or 3D?
    dimension_of_points = 2; %#ok<NASGU>

    % Find size of plotting domain
    allPointsBeingPlotted = [reachableSet; nan nan];

    max_plotValues = max(allPointsBeingPlotted);
    min_plotValues = min(allPointsBeingPlotted);
    sizePlot = max(max_plotValues) - min(min_plotValues);
    nudge = sizePlot*0.006; %#ok<NASGU>

    % Find size of plotting domain
    if flag_rescale_axis
        % NO NEED TO RESIZE THE AXIS FOR IMAGE PLOTTING
        % percent_larger = 0.3;
        % axis_range = max_plotValues - min_plotValues;
        % if (0==axis_range(1,1))
        %     axis_range(1,1) = 2/percent_larger;
        % end
        % if (0==axis_range(1,2))
        %     axis_range(1,2) = 2/percent_larger;
        % end
        % if dimension_of_points==3 && (0==axis_range(1,3))
        %     axis_range(1,3) = 2/percent_larger;
        % end
        % 
        % % Force the axis to be equal?
        % if 1==1
        %     min_valuesInPlot = min(min_plotValues);
        %     max_valuesInPlot = max(max_plotValues);
        % else
        %     min_valuesInPlot = min_plotValues;
        %     max_valuesInPlot = max_plotValues;
        % end
        % 
        % % Stretch the axes
        % stretched_min_vertexValues = min_valuesInPlot - percent_larger.*axis_range;
        % stretched_max_vertexValues = max_valuesInPlot + percent_larger.*axis_range;
        % axesTogether = [stretched_min_vertexValues; stretched_max_vertexValues];
        % newAxis = reshape(axesTogether, 1, []);
        % axis(newAxis);

    end
    % goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end   

    % Turn on legend
    legend('Interpreter','none','Location','best');

    % Plot the windfield as an image
    cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    set(cellArrayOfPlotHandles{3},'Color',[0.6 0.6 0.6]);

    % Plot the start points
    plot(originalStartPoints(:,1),originalStartPoints(:,2),'.-','Color',[0 1 0],'MarkerSize',20,'LineWidth', 2, 'DisplayName','Input: startPoints');

    % Plot the goal points
    if ~isempty(allGoalPointsList)
        plot(allGoalPointsList(:,1),allGoalPointsList(:,2),'.','Color',[1 0 1],'MarkerSize',40,'LineWidth', 2, 'DisplayName','Input: allGoalPointsList');
    end

    % Plot the expansion sets
    % allColors = parula(Nsteps+1);
    allColors = turbo;
    Ntotal = 25;
    for ith_expansion = 1:cellArrayOfExitInfo{1}
        thisExpansion = allExpansions{ith_expansion,1};
        percentageDone = min(ith_expansion/Ntotal,1);
        if 1==0
            % Plot in color
            colorNumber = min(256,max(round(255*percentageDone)+1,1));
            thisColor = allColors(colorNumber,:);
        else
            % Plot in white to black
            thisColor = (1-percentageDone)*[1 1 1];
        end

        plot(thisExpansion(:,1),thisExpansion(:,2),'-',...
            'Color',thisColor,'MarkerSize',30, 'LineWidth', 0.5, 'DisplayName',sprintf('Expansion: %.0f',ith_expansion),'HandleVisibility','off');
        pause(0.1);
    end

    % Plot the final output
    plot(finalReachableSet(:,1),finalReachableSet(:,2),'LineWidth',3,'Color',[1 0 0],'DisplayName','Output: reachableSet')

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



%% fcn_INTERNAL_sparsifyPoints
function sparsePoints = fcn_INTERNAL_sparsifyPoints(densePoints,deltaX)

% Make sure first and last point are repeated, e.g. that the plot is a
% closed circuit
if ~isequal(densePoints(1,:),densePoints(end,:))
    densePoints(end,:) = densePoints(1,:);
end

segmentVectors = densePoints(2:end,:) - densePoints(1:end-1,:);
segmentLengths = sum(segmentVectors.^2,2).^0.5;

Npoints = length(densePoints(:,1));
currentPoint = 1;
currentDistance = 0;
sparsePointIndices = false(Npoints,1);
while currentPoint<Npoints
    currentPoint = currentPoint+1;
    currentDistance  = currentDistance + segmentLengths(currentPoint-1);

    % Did we "travel" farther than expected deltaX? If so, mark this point
    % so that it is kept.
    if currentDistance>=deltaX
        sparsePointIndices(currentPoint,1) = true;
        currentDistance = 0;
    end
end
sparsePoints = densePoints(sparsePointIndices,:);

% Make sure to close off the points
if ~isequal(sparsePoints(end,:),sparsePoints(1,:))
    sparsePoints = [sparsePoints; sparsePoints(1,:)];
end

if 1==0
    figure(388383);
    clf;
    plot(densePoints(:,1),densePoints(:,2),'.-','MarkerSize',30,'LineWidth',3,'DisplayName','Input: densePoints');
    hold on;
    axis equal
    plot(sparsePoints(:,1),sparsePoints(:,2),'.-','MarkerSize',10,'LineWidth',1,'DisplayName','Output: sparsePoints');
end


end % Ends fcn_INTERNAL_sparsifyPoints

%% fcn_INTERNAL_findGoalPointsHit
function goalPointsHit = fcn_INTERNAL_findGoalPointsHit(newStartPoints,allGoalPointsList)
if ~isempty(allGoalPointsList)
    uniquePoints = flipud(newStartPoints); % for some silly reason, polyshape takes points "backwards" (?!)
    uniquePoints = unique(uniquePoints,'rows','stable');
    region = polyshape(uniquePoints(1:end-1,:),'KeepCollinearPoints', true,'Simplify', false);
    goalPointsHit = isinterior(region,allGoalPointsList);
end
end % Ends fcn_INTERNAL_findGoalPointsHit