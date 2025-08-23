function pathXYAndControlUV =  ...
    fcn_BoundedAStar_pathCalculationBackToStart(...
    endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, varargin)

% fcn_BoundedAStar_pathCalculationBackToStart
% calculates the path "backward" from an endPoint to the startPoint
%
% FORMAT:
% pathXYAndControlUV =  ...
%     fcn_BoundedAStar_pathCalculationBackToStart(...
%     endPoint, cellArrayOfExpansions, cellArrayOfIntermediateCalculations, (figNum));
%
% INPUTS:
%
%     endPoint: a 1x2 vector of the [X Y] position of the endPoint
%
%     cellArrayOfExpansions:  a cell array of each set expansion
%
%     cellArrayOfIntermediateCalculations: intermediate outputs that are
%     used to construct the reachable set, saved in a cell array that is
%     arranged as {Nsteps, 5}, where Nsteps are the simulation steps. The 5
%     columns include:
%
%       preExpansionPoints: the xK points, after simplification
%       xKPlusOne_AMatrixPoints: the A matrix calculation result
%       xKPlusOne_BMatrixPoints: the B matrix calculation result
%       xKPlusOne_WindDisturbance: the wind disturbances
%       flagPointsFilledArtificially: 1 if startPoint is 1 or 2
%           points, 0 otherwise
%
%     (optional inputs)
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     pathXYAndControlUV: a Nx4 vector containing the [X Y U V] values for
%     the pathXY and control inputs (UV)
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_pathCalculationBackToStart
% for a full test suite.
%
% This function was written on 2025_07_29 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_08_18 by S. Brennan
% - In fcn_BoundedAStar_pathCalculationBackToStart
%   % first write of function
%   % * using fcn_BoundedAStar_matrixEnvelopeExpansion as a starter
% 2025_08_19 by S. Brennan
% - In fcn_BoundedAStar_pathCalculationBackToStart
%   % * completed first end-to-end working version

% TO-DO
% (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
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
    debug_figNum = 3445467;
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
        narginchk(3,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        fcn_DebugTools_checkInputsToFunctions(...
            endPoint, '2column_of_numbers',[1 1]);

        assert(iscell(cellArrayOfExpansions));
        assert(iscell(cellArrayOfIntermediateCalculations));
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
startPoint = cellArrayOfExpansions{1,1};

% For debugging, plot the wind field. The remaining debug plots will be
% overlaid on top of this
if flag_do_debug
    figure(debug_figNum);
    clf;
    hold on;
    axis equal;
    grid on;

    % % Plot the windfield as an image
    % fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',debug_figNum);
    %
    % % Get meshgrid for streamline plotting
    % [meshX,meshY] = meshgrid(windFieldX,windFieldY);
    % s = streamslice(meshX,meshY,windFieldU,windFieldV);
    % set(s,'Color',[0.6 0.6 0.6])

    % Plot the endPoint
    plot(endPoint(:,1),endPoint(:,2),'.-',...
        'Color',[0 0 1],'MarkerSize',30,...
        'DisplayName','Input: endPoints');

    % Plot the startPoint
    plot(startPoint(:,1),startPoint(:,2),'.-',...
        'Color',[0 1 0],'MarkerSize',40,...
        'DisplayName','Input: startPoint');

    legend('Interpreter','none','Location','best');
end

%% Find set that the endPoint is contained by
Nexpansions = length(cellArrayOfExpansions);
flagEndPointContained = nan(Nexpansions,1);
for ith_expansion = 1:Nexpansions
    thisRegion = cellArrayOfExpansions{ith_expansion};
    flagEndPointContained(ith_expansion,1) = ...
        fcn_INTERNAL_findGoalPointsHit(thisRegion,endPoint);

    % Show expansion?
    if flag_do_debug && ~isempty(thisRegion)
        figure(debug_figNum);
        h_plot1 = plot(thisRegion(:,1),thisRegion(:,2),'-',...
            'Color',0.5*[0 1 0],'MarkerSize',20, 'LineWidth',4);
        if ith_expansion==1
            set(h_plot1,'DisplayName','cellArrayOfExpansions');
        else
            set(h_plot1,'HandleVisibility','off');
        end
    end

    if flagEndPointContained(ith_expansion,1)==1
        break;
    end
end
stepContainingEndPoint = find(flagEndPointContained,1,'first');
if isempty(stepContainingEndPoint)
    error('endPoint not containted within any of the given set sequences');
end

pathXYAndControlUV = nan(stepContainingEndPoint,4);
currentLocation = endPoint;
flagFirstTimeInLoop = 1;
for ith_step = stepContainingEndPoint:-1:2
    if 1==0 && 15==ith_step
        disp('Stop here');
        flag_do_deep_debug = 1;
        dbstop in fcn_BoundedAStar_pathCalculationBackToStart at 380
        % dbstop in fcn_BoundedAStar_pathCalculationBackToStart at 717
        % dbstop in fcn_BoundedAStar_pathCalculationBackToStart at 634
    else
        flag_do_deep_debug = 0;
    end

    currentExpansionPoints        = cellArrayOfExpansions{ith_step,1};
    preExpansionPoints            = cellArrayOfIntermediateCalculations{ith_step,1};
    xKPlusOne_AMatrixPoints       = cellArrayOfIntermediateCalculations{ith_step,2};
    xKPlusOne_BMatrixPoints       = cellArrayOfIntermediateCalculations{ith_step,3};
    xKPlusOne_WindDisturbance     = cellArrayOfIntermediateCalculations{ith_step,4};
    flagPointsFilledArtificially  = cellArrayOfIntermediateCalculations{ith_step,5};

    if flag_do_debug
        figure(debug_figNum);


        % Plot currentLocation
        h_plot0 = plot(currentLocation(:,1),currentLocation(:,2),'*',...
            'Color',[1 0 0],'MarkerSize',20);

        % Plot starting expansion
        h_plot1 = plot(preExpansionPoints(:,1),preExpansionPoints(:,2),'.-',...
            'Color',[1 0 0],'MarkerSize',20);

        % Plot starting points to A
        vectorLength = xKPlusOne_AMatrixPoints - preExpansionPoints;
        h_plot2 = quiver(preExpansionPoints(:,1), preExpansionPoints(:,2),...
            vectorLength(:,1),vectorLength(:,2),0,...
            'LineWidth',1,...
            'ShowArrowHead','on','MaxHeadSize',1,...
            'Color',0.8*[1 0 0],'MarkerSize',30);

        % Plot A to B
        shiftedPoints = xKPlusOne_AMatrixPoints;
        h_plot3 = quiver(shiftedPoints(:,1), shiftedPoints(:,2),...
            xKPlusOne_BMatrixPoints(:,1),xKPlusOne_BMatrixPoints(:,2),0,...
            'LineWidth',1,...
            'ShowArrowHead','on','MaxHeadSize',1,...
            'Color',0.6*[1 0 0],'MarkerSize',30);

        % Plot B to Wind
        shiftedPoints = xKPlusOne_AMatrixPoints+xKPlusOne_BMatrixPoints;
        h_plot4= quiver(shiftedPoints(:,1), shiftedPoints(:,2),...
            xKPlusOne_WindDisturbance(:,1),xKPlusOne_WindDisturbance(:,2),0,...
            'LineWidth',1,...
            'ShowArrowHead','on','MaxHeadSize',1,...
            'Color',0.4*[1 0 0],'MarkerSize',20);

        % Plot final location
        h_plot5 = plot(currentExpansionPoints(:,1),currentExpansionPoints(:,2),'.-',...
            'Color',0.2*[1 0 0],'MarkerSize',20,'LineWidth',2);

        % Label vertices
        for ith_point = 1:length(currentExpansionPoints(:,1))-1
            text(currentExpansionPoints(ith_point,1)+0.05,currentExpansionPoints(ith_point,2),sprintf('%.0f',ith_point));
        end

        % Set up labels, or not
        if flagFirstTimeInLoop==1
            flagFirstTimeInLoop = 0;
            set(h_plot0,'DisplayName','currentLocation');
            set(h_plot1,'DisplayName','preExpansionPoints');
            set(h_plot2,'DisplayName','xKPlusOne_AMatrixPoints');
            set(h_plot3,'DisplayName','xKPlusOne_BMatrixPoints');
            set(h_plot4,'DisplayName','xKPlusOne_WindDisturbance');
            set(h_plot5,'DisplayName','currentExpansionPoints');
        else
            set(h_plot0,'HandleVisibility','off');
            set(h_plot1,'HandleVisibility','off');
            set(h_plot2,'HandleVisibility','off');
            set(h_plot3,'HandleVisibility','off');
            set(h_plot4,'HandleVisibility','off');
            set(h_plot5,'HandleVisibility','off');
        end
    end

    %%%%
    % Find enclosing vectors. These are the vectors to the right and left
    % of the currentLocation. To find these, we first find the vectors
    % pointing from the previous expansion to the current one. Next, we
    % find which vectors have the point to "left", via a ortho projection.
    % Then we find which vectors are to the right and left of the point
    % (using a cross product)
    if ~isequal(preExpansionPoints(1,:),preExpansionPoints(end,:))
        error('Expected preExpansion points to be circular');
    end

    Nvectors = length(preExpansionPoints(:,1));
    vectorsFromPreToPostExpansion = currentExpansionPoints - preExpansionPoints;
    vectorsFromPreToEndPoint = ones(Nvectors,1)*currentLocation - preExpansionPoints;
    partialSegmentsOnPreExpansion = preExpansionPoints(2:end,:) - preExpansionPoints(1:end-1,:);
    segmentsOnPreExpansion = [partialSegmentsOnPreExpansion; nan nan];
    segmentDistances = sum(segmentsOnPreExpansion.^2,2).^0.5;
    unitSegmentsOnPreExpansion = segmentsOnPreExpansion./segmentDistances;
    unitOrthoSegmentsOnPreExpansion = unitSegmentsOnPreExpansion*[0 1; -1 0];
    projectionDistances = sum(vectorsFromPreToEndPoint.*unitOrthoSegmentsOnPreExpansion,2);
    signsOfProjectionDistances   = sign(projectionDistances)<=0;

    %dotProducts = sum(vectorsFromPreToEndPoint.*vectorsFromPreToPostExpansion,2);
    %signsOfDotProducts   = sign(dotProducts)>=0;
    crossProducts = INTERNAL_crossProduct(vectorsFromPreToPostExpansion,vectorsFromPreToEndPoint);
    signsOfCrossProducts = sign(crossProducts);
    changesInCrossProducts = [diff(signsOfCrossProducts); 0];

    % If the cross product is near the borderline, it may be changing
    crossProductsNearZero = abs(crossProducts)<0.01; 
    changesInCrossProducts(crossProductsNearZero) = 1;
    
    locationsBetween = find(signsOfProjectionDistances.*(abs(changesInCrossProducts)>0));
    if isempty(locationsBetween)
        warning('Unable to find any locations between expansion vectors');
    end


    if 1==0 && flag_do_deep_debug
        figure(484848);
        clf;
        axis equal;
        hold on;
        grid on;

        % Plot currentLocation
        plot(currentLocation(:,1),currentLocation(:,2),'.',...
            'Color',[0 1 0],'MarkerSize',40, 'DisplayName','currentLocation');

        % Plot starting expansion
        plot(preExpansionPoints(:,1),preExpansionPoints(:,2),'.-',...
            'Color',[1 0 0],'MarkerSize',20, 'DisplayName','preExpansionPoints');

        % Number the points
        for ith_point = 1:length(preExpansionPoints(:,1))-1
            text(preExpansionPoints(ith_point,1)+0.01,preExpansionPoints(ith_point,2),sprintf('%.0f',ith_point));
            text(currentExpansionPoints(ith_point,1)+0.01,currentExpansionPoints(ith_point,2),sprintf('%.0f',ith_point));
        end

        % Plot currentExpansionPoints
        plot(currentExpansionPoints(:,1),currentExpansionPoints(:,2),'.-',...
            'Color',0.2*[1 0 0],'MarkerSize',20,'LineWidth',2, 'DisplayName','currentExpansionPoints');

        for ith_location = 1:length(locationsBetween)
            thisLocation = locationsBetween(ith_location);
            indicesRange = thisLocation:thisLocation+1;

            % Plot vectorsFromPreToPostExpansion
            quiver(preExpansionPoints(indicesRange,1), preExpansionPoints(indicesRange,2),...
                vectorsFromPreToPostExpansion(indicesRange,1),vectorsFromPreToPostExpansion(indicesRange,2),0,...
                'LineWidth',1,...
                'ShowArrowHead','on','MaxHeadSize',1,...
                'Color',0.8*[0 1 0],'MarkerSize',30, 'DisplayName','vectorsFromPreToPostExpansion');

        end % Ends looping through locations

    end % Ends debug plotting



    % For some expansions, get many locations where the point is "between"
    % vectors. Need to test each.
    NlocationsToTest = length(locationsBetween);
    percentagesAlongTop = nan(NlocationsToTest,1);
    percentagesBeyondEndPoint = nan(NlocationsToTest,1);
    for ith_location = 1:NlocationsToTest
        indexOfQuery = locationsBetween(ith_location);
        [percentagesAlongTop(ith_location,1), percentagesBeyondEndPoint(ith_location,1)] = ...
            fcn_INTERNAL_percentageAlongTop(...
            currentLocation, indexOfQuery, preExpansionPoints, currentExpansionPoints, vectorsFromPreToPostExpansion, ith_location);
    end
    
    [~, goodIndex] = max(percentagesBeyondEndPoint);
    percentageAlongTop = percentagesAlongTop(goodIndex);
    indexOfQuery = locationsBetween(goodIndex);
    indexRange = indexOfQuery:indexOfQuery+1;
    sideBases   = preExpansionPoints(indexRange,:);

    if isnan(percentageAlongTop)
        % Can come in here is when point is outside both envelopes. Check
        % for this.

        goalPointInsideOuterBoundary = fcn_INTERNAL_findGoalPointsHit(currentExpansionPoints,currentLocation);
        if ~goalPointInsideOuterBoundary
            % Find closest 2 points in outside boundary, and use these
            % FORMAT:
            %      [closest_path_point,s_coordinate,path_point_yaw,....
            %      first_path_point_index,...
            %      second_path_point_index,...
            %      percent_along_length] = ...
            %      fcn_Path_snapPointOntoNearestPath(point, path, (fig_num))       
            [~,~,~,....
                first_path_point_index,...
                second_path_point_index,...
                percentageAlongTop] = ...
                fcn_Path_snapPointOntoNearestPath(currentLocation, currentExpansionPoints, (-1));
            if 1==0
                figure(585858);
                clf;
                hold on;
                plot(currentLocation(:,1),currentLocation(:,2),'b.','MarkerSize',20);
                plot(currentExpansionPoints(:,1),currentExpansionPoints(:,2),'k.-','MarkerSize',30);
                plot(currentExpansionPoints(first_path_point_index,1),currentExpansionPoints(first_path_point_index,2),...
                    'g.','MarkerSize',10);
                plot(currentExpansionPoints(second_path_point_index,1),currentExpansionPoints(second_path_point_index,2),...
                    'r.','MarkerSize',10);
            end
                
        else
            error('Nan value encountered for percentages along top. Likely to crash.');
        end
    end

    positionsAtThisStep = ...
        (1 - percentageAlongTop)*sideBases(1,:) + ...
        (percentageAlongTop)*sideBases(2,:);


    controlInputsAtBase = xKPlusOne_BMatrixPoints(indexRange,:);
    controlInputsAtThisStep = ...
        (1 - percentageAlongTop)*controlInputsAtBase(1,:) + ...
        (percentageAlongTop)*controlInputsAtBase(2,:);

    % Calculate pathXYUV
    if ith_step == stepContainingEndPoint
        lastXYUV = [currentLocation controlInputsAtThisStep];
    end

    if 0==flagPointsFilledArtificially
        pathXYAndControlUV(ith_step-1, :) = [positionsAtThisStep controlInputsAtThisStep];
        currentLocation = positionsAtThisStep;
    else
        % This step is singular, e.g. it returns to startPoint
        pathXYAndControlUV(ith_step-1, :) = [startPoint controlInputsAtThisStep];

    end % Ends check if flagPointsFilledArtificially


end

pathXYAndControlUV(end,:) = lastXYUV;


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
    allPointsBeingPlotted = [currentExpansionPoints; nan nan];

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

    % % Plot the windfield as an image
    % cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    % set(cellArrayOfPlotHandles{3},'Color',[0.6 0.6 0.6]);

    % Plot the endPoint
    plot(endPoint(:,1),endPoint(:,2),'.','Color',[1 0 0],'MarkerSize',30,'LineWidth', 2, 'DisplayName','Input: endPoint');

    % Plot the startPoint
    plot(startPoint(:,1),startPoint(:,2),'.','Color',[0 1 0],'MarkerSize',30,'LineWidth', 2, 'DisplayName','Input: startPoint');

    % Plot the final XY path in blue
    plot(pathXYAndControlUV(:,1),pathXYAndControlUV(:,2),'.-', 'LineWidth',2,...
        'MarkerSize',10,...
        'Color',[0 0 1],'DisplayName','Output: XY path')

    % number the path steps
    for ith_step = 1:length(pathXYAndControlUV(:,1))
        text(pathXYAndControlUV(ith_step,1),pathXYAndControlUV(ith_step,2),sprintf('%.0f',ith_step));
    end

    % Show arrows of control inputs in cyan
    quiver(pathXYAndControlUV(:,1), pathXYAndControlUV(:,2),...
        pathXYAndControlUV(:,3),pathXYAndControlUV(:,4),0,...
        'LineWidth',2,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[1 0 1],'DisplayName','Output: UV control inputs')

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

%% fcn_INTERNAL_findGoalPointsHit
function goalPointsHit = fcn_INTERNAL_findGoalPointsHit(regionBoundary,allGoalPointsList)
goalPointsHit = false;
if ~isempty(allGoalPointsList) && ~isempty(regionBoundary)
    flippedPoints = flipud(regionBoundary); % for some silly reason, polyshape takes points "backwards" (?!)
    uniqueFlippedPoints = unique(flippedPoints,'rows','stable');
    if length(uniqueFlippedPoints)>=3
        region = polyshape(uniqueFlippedPoints,'KeepCollinearPoints', true,'Simplify', false);
        goalPointsHit = isinterior(region,allGoalPointsList);
    else
        region = [];
        goalPointsHit = false;
    end
end
if 1==0
    figure(38383);
    clf;
    plot(region)
    hold on;
    plot(allGoalPointsList(:,1),allGoalPointsList(:,2),'r.');
end
end % Ends fcn_INTERNAL_findGoalPointsHit


%% Calculate cross products
function result = INTERNAL_crossProduct(v,w)
result = v(:,1).*w(:,2)-v(:,2).*w(:,1);
end

%% fcn_INTERNAL_percentageAlongTop - calculate percentage of inputs
function [percentageAlongTop,percentageBeyondEndPoint] = fcn_INTERNAL_percentageAlongTop(...
    currentLocation, indexOfQuery, preExpansionPoints, currentExpansionPoints, vectorsFromPreToPostExpansion, plotNumber)
% Define bounding vectors
indexRange  = indexOfQuery:indexOfQuery+1;
sideBases   = preExpansionPoints(indexRange,:);
sideVectors = vectorsFromPreToPostExpansion(indexRange,:);
topPoints   = currentExpansionPoints(indexRange,:);
topVector   = diff(topPoints,[],1);

% Assume the test point is inside the polygon area. Do some tests below
% that check this.
flag_isInside = 1;

% Make sure edge vectors are well formed. Sometimes, edges get wierd (due
% to expansion steps). A well-formed side sticks out in the "negative"
% direction of the base vector.

% Find unit vector of base
baseVector = diff(sideBases,[],1);
baseVectorMagnitude = sum(baseVector.^2,2).^0.5;
unitBaseVector = baseVector./baseVectorMagnitude;
unitOrthoBaseVector = unitBaseVector*[0 1; -1 0];
sideVectorDistancesFromBase = sum(sideVectors.*unitOrthoBaseVector,2);

if any(sideVectorDistancesFromBase>0)
    flag_isInside = 0;
end

if 1==flag_isInside
    % Make sure that the point is INSIDE the bounding area. Previous steps
    % check if the point is to the outside of the preExpansion points, and to
    % the right/left of the sides. Need to check that it is "inside" the top.
    topPointsVector = topPoints(2,:)-topPoints(1,:);
    topPointsMagnitude = sum(topPointsVector.^2,2).^0.5;
    topPointsUnitVector = topPointsVector./topPointsMagnitude;
    topPointsUnitOrthoVector = topPointsUnitVector*[0 1; -1 0];
    topVectorRelativeToLocation = currentLocation - topPoints(1,:);
    distanceToTop = sum(topPointsUnitOrthoVector.*topVectorRelativeToLocation,2);
    tol = 0.5;
    if distanceToTop<(-1*tol)
        flag_isInside = 0;
    end
end

if 1==flag_isInside
    % If the point is inside, then projections to the point from the base
    % points will either hit the top, or the "opposing" side. If it hits
    % the top, we can calculate the percentages from the ratio along the
    % top. If it hits the opposing side, we can calculate the percentages
    % from the ratio of lengths. Because the base has 2 points, we can get
    % 2 different answers. So we call another function here to calculate
    % each answer, once per each location in the base.
    percentagesAlongTop = nan(2,1);
    percentagesBeyondEndPoint = nan(2,1);
    for index = 1:2
        [percentagesAlongTop(index), percentagesBeyondEndPoint(index)] = ...
            fcn_INTERNAL_findPercentage(...
            currentLocation, sideBases, index, topPoints);
    end

    % Check if all, one, or none of the results are valid
    if all(isnan(percentagesAlongTop),'all')
        percentageAlongTop = nan;
        percentageBeyondEndPoint = nan;
    elseif any(isnan(percentagesAlongTop))
        percentageAlongTop = percentagesAlongTop(~isnan(percentagesAlongTop));
        percentageBeyondEndPoint = percentagesBeyondEndPoint(~isnan(percentagesAlongTop));
    else
        percentageAlongTop = sum(percentagesAlongTop)/2;
        percentageBeyondEndPoint = sum(percentagesBeyondEndPoint)/2;
    end
else
    % Point is not inside a well-defined polygon
    percentageAlongTop = nan;
    percentageBeyondEndPoint = nan;
end

% Used to debug the interpolation process
if (1==0)
    figure(43536);
    if 1==plotNumber
        clf;
        tiledlayout('horizontal');
    end

    nexttile

    % Plot the currentLocation
    hold on;
    grid on;
    axis equal
    plot(currentLocation(:,1),currentLocation(:,2),'.',...
        'Color',[0 0 1],'MarkerSize',30,...
        'DisplayName','Input: currentLocation');

    % Plot side vectors
    quiver(sideBases(:,1), sideBases(:,2),...
        sideVectors(:,1),sideVectors(:,2),0,...
        'LineWidth',5,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[0 0 0],'MarkerSize',30);


    % Plot base vector
    quiver(sideBases(1,1), sideBases(1,2),...
        baseVector(:,1),baseVector(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[0 0 1],'MarkerSize',30);

    % Plot top vector
    quiver(topPoints(1,1), topPoints(1,2),...
        topVector(:,1),topVector(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',0.5*[0 0 1],'MarkerSize',30);


    % Plot end vectors
    % Define vectors relative to base first position
    currentPointVector   = currentLocation - sideBases(1,:);
    quiver(sideBases(1,1), sideBases(1,2),...
        currentPointVector(:,1),currentPointVector(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[1 0 1],'MarkerSize',30);
    % Define vectors relative to base second position
    currentPointVector   = currentLocation - sideBases(2,:);
    quiver(sideBases(2,1), sideBases(2,2),...
        currentPointVector(:,1),currentPointVector(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[1 0 1],'MarkerSize',30);

    if isnan(percentageAlongTop)
        title(sprintf('BAD: %.0f',indexOfQuery));
    else
        title(sprintf('Good: %.0f',indexOfQuery));
    end
end


end % Ends fcn_INTERNAL_percentageAlongTop

%% fcn_INTERNAL_findPercentage
function [percentageAlong, percentageBeyondEndPoint] = fcn_INTERNAL_findPercentage(...
    currentLocation, sideBases, indexToCheck, topPoints)

% Define vectors relative to base first position
currentPointVector   = currentLocation - sideBases(indexToCheck,:);
sideVectorsRelativeToBasePoint = topPoints-ones(2,1)*sideBases(indexToCheck,:);

% Intersect the end vector with the top side
% FORMAT:
%      [distance, location, wall_segment, t, u] = ...
%         fcn_Path_findSensorHitOnWall(...
%         wall_start, wall_end,...
%         sensor_vector_start,sensor_vector_end,...
%         (flag_search_return_type), (flag_search_range_type), ...
%         (tolerance), (fig_num))
[hitDistanceToTop, hitLocation, ~, percentageAlong, percentageBeyondEndPoint] = ...
    fcn_Path_findSensorHitOnWall(...
    topPoints(1,:), topPoints(2,:),...
    sideBases(indexToCheck,:),currentLocation,...
    (0), (1), ...
    ([]), (-1));

if ~isnan(percentageAlong)
    % Don't allow negative distances
    if hitDistanceToTop<0
        percentageAlong = nan;
        percentageBeyondEndPoint = nan;
    end
else
    % Only way to enter here is if no intersection with top vector. So we
    % need to intersect the end vector with the opposite side
    opposingSideIndex = mod(indexToCheck,2)+1;

    [hitDistanceToSide, hitLocation, ~, percentageAlongSide, percentageBeyondEndPointSide] = ...
        fcn_Path_findSensorHitOnWall(...
        sideBases(opposingSideIndex,:), topPoints(opposingSideIndex,:),...
        sideBases(indexToCheck,:),currentLocation,...
        (0), (1), ...
        ([]), (-1)); %#ok<ASGLU>

    if hitDistanceToSide<0 || isnan(hitDistanceToSide)
        percentageAlong = nan;
        percentageBeyondEndPoint = nan;
    else
        if 1==indexToCheck
            percentageAlong = 1/percentageBeyondEndPointSide;
            percentageBeyondEndPoint = 1;            
        else
            percentageAlong = 1- 1/percentageBeyondEndPointSide;
            percentageBeyondEndPoint = 1;            
        end
        percentageAlong = max(0,min(1,percentageAlong));
        percentageBeyondEndPoint = max(0,min(1,percentageBeyondEndPoint));
    end
end


% Used to debug the interpolation process
if (1==0)
    figure(54455);
    clf;
    hold on;
    grid on;
    axis equal

    % Plot the currentLocation
    plot(currentLocation(:,1),currentLocation(:,2),'.',...
        'Color',[0 0 1],'MarkerSize',30,...
        'DisplayName','Input: currentLocation');

    % Plot end vector
    quiver(sideBases(indexToCheck,1), sideBases(indexToCheck,2),...
        currentPointVector(:,1),currentPointVector(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[1 0 1],'MarkerSize',30);

    % Plot sides
    plot([sideBases(1,1) topPoints(1,1)], [sideBases(1,2) topPoints(1,2)],'.-',...
        'LineWidth',5,...
        'Color',[0 0 1],'MarkerSize',30);
    plot([sideBases(2,1) topPoints(2,1)], [sideBases(2,2) topPoints(2,2)],'.-',...
        'LineWidth',5,...
        'Color',[0 0 1],'MarkerSize',30);


    % Plot relative side vectors
    quiver(ones(2,1)*sideBases(indexToCheck,1), ones(2,1)*sideBases(indexToCheck,2),...
        sideVectorsRelativeToBasePoint(:,1),sideVectorsRelativeToBasePoint(:,2),0,...
        'LineWidth',1,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',[0 0 0],'MarkerSize',30);

    % Plot top points
    plot(topPoints(:,1), topPoints(:,2),'.-',...
        'LineWidth',3,...
        'Color',[0 1 0],'MarkerSize',10);

    % Plot hitLocation
    plot(hitLocation(:,1),hitLocation(:,2),'.',...
        'Color',[1 0 0],'MarkerSize',30,...
        'DisplayName','relativeSideLocations');
    if isnan(percentageAlong)
        title('BAD point');
    else
        title(sprintf('Good point. Percentage along: %.2f',percentageAlong));
    end

end


end % Ends fcn_INTERNAL_findPercentage