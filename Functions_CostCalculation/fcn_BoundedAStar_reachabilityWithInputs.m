function [reachableSet, cellArrayOfIntermediateCalculations] = fcn_BoundedAStar_reachabilityWithInputs(radius, windFieldU, windFieldV, windFieldX, windFieldY, varargin)
% fcn_BoundedAStar_reachabilityWithInputs
% calculates the 1-step reachable set from a given set of points,
% startPoints, accounting for control inputs and the wind field
%
% FORMAT:
% [reachableSet, cellArrayOfIntermediateCalculations] = fcn_BoundedAStar_reachabilityWithInputs(...
%     radius, ... 
%     windFieldU,  ...
%     windFieldV,  ...
%     windFieldX,  ...
%     windFieldY,  ...
%     (startPoints),  ...
%     (flagWindRoundingType),...
%     (figNum));
%
% INPUTS:
%
%     radius: a 1x1 scalar representing the radius of travel without wind
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
%     keepOutZones:
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     reachableSet: a set of points defining the distance conversion of the
%     original travel locations to locations with wind
%
%     cellArrayOfIntermediateCalculations: intermediate outputs that are
%     used to construct the reachable set, saved in a cell array. These
%     include:
%
%             preExpansionPoints: the xK points, after simplification
%             xKPlusOne_AMatrixPoints: the A matrix calculation result
%             xKPlusOne_BMatrixPoints: the B matrix calculation result
%             xKPlusOne_WindDisturbance: the wind disturbances
%             flagPointsFilledArtificially: 1 if startPoint is 1 or 2
%             points, 0 otherwise
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_reachabilityWithInputs
% for a full test suite.
%
% This function was written on 2025_07_29 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_07_29 by K. Hayes
% - first write of function 
%   % * using fcn_BoundedAStar_matrixEnvelopeExpansion as a starter
% 2025_07_29 by K. Hayes
% - Renamed inputs x and y to windFieldX, windFieldY to clarify that these
%   % are used for wind fields, not states
% - Renamed output to reachableSet for clarity
% - Allowed startPoint to be startPoints, so we can enter a set as starting
%   % values
% - Changed fig_num to figNum to avoid underscores in names
% - Cleaned up the header comments a bit
% - Added debug plotting at start of code
%
% 2025_08_02 by S. Brennan
% Significant rewrite of fcn_BoundedAStar_reachabilityWithInputs and as
% well a "slow" form: fcn_BoundedAStar_reachabilityWithInputs_SLOW
% - Vectorized for loops for sampling speed 
% - added script_time_fcn_BoundedAStar_reachabilityWithInputs for timing 
%   tests
% - added specialized set bounds to merge search space across input points
% - modified the set methods to use edge-projection expansions for speeds
%   % * This is about 40 times faster than using set methods (!)
% - optimized code where easily changed to get fastest speeds. Single loop
%   % calls appear to take about 0.2 milliseconds with fastest settings 
% - added rule in main loop to "clean up points". Namely: the projection is
%   % only valid if the segment length created by the adjacent unit vectors
%   % has length greater than 0.5 (typically, this value is greater than
%   % .90 in real-world data)
%
% 2025_08_08 by K. Hayes
% -- added call to fcn_BoundedAStar_sampleWindField in place of call to
%    internal function
%
% 2024_08_08 by S. Brennan
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * added cellArrayOfIntermediateCalculations to outputs
%
% 2024_08_15 to 2024_08_16 by S. Brennan
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * moved meshgrid cacluation into debugging, as it is not used for
%   %   % main outputs - only for debugging
%   % * improved the header description for clarity
%   % * fixed bug where the expansion due to wind disturbance was
%   %   % after the state expansion, instead of prior
%   % * updated function to output intermediate calculations
%   % * moved set simplification steps to PRIOR to states
%
% 2024_08_17 by S. Brennan
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * Functionalized the code a bit more
%
% 2024_08_22 by S. Brennan
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * Fixed bug with NaN values produced due to typo on isequal 
%   %   % (line 926)

% TO-DO
% -- update header

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
        narginchk(5,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        fcn_DebugTools_checkInputsToFunctions(...
            radius, '1column_of_numbers',[1 1]);
    end
end

% Does user want to specify startPoints input?
startPoints = [0 0]; % Default is origin
if 1 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        startPoints = temp;
    end
end

% Does user want to specify flagWindRoundingType input?
flagWindRoundingType = 0; %#ok<NASGU> % Default is 0
if 2 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        flagWindRoundingType = temp; %#ok<NASGU>
    end
end

% Does user want to specify keepOutZones input?
keepOutZones = []; % Default is none
if 3 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        keepOutZones = temp;
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

% For debugging, plot the wind field. The remaining debug plots will be
% overlaid on top of this
if flag_do_debug
    figure(debug_figNum);
    clf;
    hold on;
    
    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',debug_figNum);

    % Get meshgrid for streamline plotting
    [meshX,meshY] = meshgrid(windFieldX,windFieldY);
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6])

    % Plot the start points
    plot(startPoints(:,1),startPoints(:,2),'.-',...
        'Color',[0 0 1],'MarkerSize',30,...
        'DisplayName','Input: startPoints');

end

if any(isnan(startPoints),'all')
    error('Nan values found in startPoints. This will crash the code');
end


%% Find pre-expansion points
% For the operations that follow, these are simplified greatly if all the
% points are in the same format, namely: no repeated points other than
% first and last, and the points enclose a region. The following function
% prepares the points for this format. In particular, it fixes the special
% case where there are only 1 or 2 non-repeated points in the startPoints
[densePreExpansionPoints, flagPointsFilledArtificially] = ...
    fcn_INTERNAL_prepareStartPoints(startPoints, radius);

uniqueDensePreExpansionPoints = unique(densePreExpansionPoints(1:end-1,:),'rows','stable');
if numel(uniqueDensePreExpansionPoints) ~= numel(densePreExpansionPoints(1:end-1,:))
    error('densePreExpansionPoints have repeating values');
end

if any(isnan(densePreExpansionPoints),'all')
    error('Nan values found in densePreExpansionPoints. This will crash the code');
end


if flag_do_debug
    figure(debug_figNum);
    plot(densePreExpansionPoints(:,1),densePreExpansionPoints(:,2),'.-',...
        'Color',[0 1 0],'MarkerSize',20,'DisplayName','densePreExpansionPoints');
end

%% Resample points to match the wind field discretization
% The points along the bounding edge may be "dense", more dense than the
% wind field discretization. This is unnecessary, since the wind can only
% act differently at spacings larger than the discretization used to define
% the wind field. So here, the boundary is re-sampled to be at least as
% large as the wind field discretization distance.
stepSize = windFieldX(2)-windFieldX(1);
preExpansionPoints = fcn_INTERNAL_resamplePointsToMatchWindDiscretization(...
    densePreExpansionPoints,stepSize);

uniquePreExpansionPoints = unique(preExpansionPoints(1:end-1,:),'rows','stable');
if numel(uniquePreExpansionPoints) ~= numel(preExpansionPoints(1:end-1,:))
    error('preExpansionPoints have repeating values');
end

if any(isnan(preExpansionPoints),'all')
    error('Nan values found in preExpansionPoints. This will crash the code');
end

if flag_do_debug
    figure(debug_figNum);
    plot(preExpansionPoints(:,1),preExpansionPoints(:,2),'.-',...
        'Color',[1 0 0],'MarkerSize',20,'DisplayName','preExpansionPoints');
end

%% Calculate the state propogation based on dynamics. 

% Create state matrices
% For now, this is
% trivial: x_(k+1) = A*x_k for A=identity gives x_(k+1) = x(k)
% A - the vehicle remains in the same position unless moved
NPreExpansionPoints = length(preExpansionPoints(:,1));
A = eye(NPreExpansionPoints);

% The following does: x_(k+1) = A*x_k 
xKPlusOne_AMatrixPoints = A*preExpansionPoints;

if any(isnan(xKPlusOne_AMatrixPoints),'all')
    error('Nan values found in xKPlusOne_AMatrixPoints. This will crash the code');
end

if flag_do_debug
    figure(debug_figNum);
    plot(xKPlusOne_AMatrixPoints(:,1),xKPlusOne_AMatrixPoints(:,2),'.-',...
        'Color',[0 1 0],'MarkerSize',20,'DisplayName','xKPlusOne_AMatrixPoints');
end

%% Calculate the effects of inputs. 

% This represents the B*u term. For now, assume B*u is an arbitrary
% direction, essentially pushing all preExpansionPoints "outward" from
% wherever they are. Note: this operation sometimes requires conditioning
% of the points, changing which startPoints are used. Thus, it passes out
% indicesKept so this can be fixed.
[~, xKPlusOne_BMatrixPoints, indicesKept] = fcn_INTERNAL_expandVerticesOutward(preExpansionPoints, radius);
preExpansionPoints = preExpansionPoints(indicesKept,:);
if ~isequal(preExpansionPoints(1,:),preExpansionPoints(end,:))
    error('First and last preExpansionPoints points must be same');
end

xKPlusOne_AMatrixPoints = xKPlusOne_AMatrixPoints(indicesKept,:);

if any(isnan(xKPlusOne_BMatrixPoints),'all')
    error('Nan values found in xKPlusOne_BMatrixPoints. This will crash the code');
end

if flag_do_debug
    figure(debug_figNum);
    summedPoints = xKPlusOne_AMatrixPoints+xKPlusOne_BMatrixPoints;
    plot(summedPoints(:,1),summedPoints(:,2),'.-',...
        'Color',[1 0 1],'MarkerSize',20,'DisplayName','xKPlusOne_BMatrixPoints');
end

%%%%
% OLD METHOD using polyshape operations: (slow)
% boundingPolytope = polyshape(startPoints);
% 
% 
% if flag_do_debug
%     figure(debug_figNum);
%     h_allPoly = plot(boundingPolytope);
% end
% 
% % Merge the polytopes created by the expansion of each of the start points
% for ith_start = firstIndex:NstartPoints
%     polyPoints = repmat(movedPoints(ith_start,:),Ndirections-1,1) + controlInputPerturbations;
%     thisPoly = polyshape(polyPoints);
%     boundingPolytope = union(boundingPolytope,thisPoly);
% 
%     if flag_do_debug
%         figure(debug_figNum);
%         set(h_allPoly,'Visible','off');
%         plot(boundingPolytope);
%     end
% 
% end
% 
% % Pull the polytope vertices
% boundingPolytopeVerticesRaw = boundingPolytope.Vertices;
% % Repeat last point to close off the boundary
% boundingPolytopeVertices = [boundingPolytopeVerticesRaw; boundingPolytopeVerticesRaw(1,:)];


%% Calculate wind disturbance

% This calculates the effect of the wind input

% FORMAT:
% function [filteredWindDisturbances, resampledWindSamplingVertices] = ...
%     fcn_INTERNAL_sampleWindAtPoints(windSamplingVertices, ...
%     windFieldX, windFieldY, windFieldU, windFieldV, ...
%     flagWindRoundingType) %#ok<INUSD>
xKPlusOne_WindDisturbance = ...
    fcn_INTERNAL_sampleWindAtPoints(preExpansionPoints, ...
    windFieldX, windFieldY, windFieldU, windFieldV,0);


%% Add up effects to calculate new reachable set

reachableSet = ...
    xKPlusOne_AMatrixPoints + ...
    xKPlusOne_BMatrixPoints + ...
    xKPlusOne_WindDisturbance;

if flag_do_debug
    figure(debug_figNum);

    % Do a quiver plot to show how wind moves the boundary?
    if 1==1
        Nvertices = length(reachableSet(:,1));
        plotEvery = 1;
        vectorLengths = reachableSet-preExpansionPoints;
        rowsToPlot = find(mod((1:Nvertices)',plotEvery)==0);
        quiver(preExpansionPoints(rowsToPlot,1),preExpansionPoints(rowsToPlot,2),...
            vectorLengths(rowsToPlot,1),vectorLengths(rowsToPlot,2),0,'filled');
    end
    plot(reachableSet(:,1),reachableSet(:,2),'y.-','LineWidth',3,'MarkerSize',30);

end

%% Save intermediate calculations
%             preExpansionPoints: the xK points, after simplification
%             xKPlusOne_AMatrixPoints: the A matrix calculation result
%             xKPlusOne_BMatrixPoints: the B matrix calculation result
%             xKPlusOne_WindDisturbance: the wind disturbances
cellArrayOfIntermediateCalculations = cell(5,1);
cellArrayOfIntermediateCalculations{1,1} = preExpansionPoints;
cellArrayOfIntermediateCalculations{2,1} = xKPlusOne_AMatrixPoints;
cellArrayOfIntermediateCalculations{3,1} = xKPlusOne_BMatrixPoints;
cellArrayOfIntermediateCalculations{4,1} = xKPlusOne_WindDisturbance;
cellArrayOfIntermediateCalculations{5,1} = flagPointsFilledArtificially;

if flag_do_debug
    figure(debug_figNum);
    shiftedPoints = xKPlusOne_AMatrixPoints;
    quiver(shiftedPoints(:,1), shiftedPoints(:,2),...
        xKPlusOne_BMatrixPoints(:,1),xKPlusOne_BMatrixPoints(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',0.6*[1 0 0],'MarkerSize',20);
    shiftedPoints = xKPlusOne_AMatrixPoints+xKPlusOne_BMatrixPoints;
    quiver(shiftedPoints(:,1), shiftedPoints(:,2),...
        xKPlusOne_WindDisturbance(:,1),xKPlusOne_WindDisturbance(:,2),0,...
        'LineWidth',3,...
        'ShowArrowHead','on','MaxHeadSize',1,...
        'Color',0.4*[1 0 0],'MarkerSize',20);
    plot(preExpansionPoints(:,1),preExpansionPoints(:,2),'.-',...
        'Color',[1 0 0],'MarkerSize',20);
    plot(reachableSet(:,1),reachableSet(:,2),'r.','LineWidth',3, 'MarkerSize',20);


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

    % Turn on legend?
    legends = findobj(figNum, 'Type', 'legend');
    if isempty(legends)
        legend('Interpreter','none','Location','northwest');
        flag_legendAlredyOn = 0;
    else
        flag_legendAlredyOn = 1;
    end

    % Plot the windfield as an image
    cellArrayOfPlotHandles = fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    set(cellArrayOfPlotHandles{3},'Color',[0.6 0.6 0.6]);

    % Plot the start points and final output
    if 0 == flag_legendAlredyOn
        plot(startPoints(:,1),startPoints(:,2),'.-','Color',[0 0 1],'MarkerSize',10,'LineWidth', 2, 'DisplayName','Input: startPoints');
        plot(reachableSet(:,1),reachableSet(:,2),'LineWidth',2,'Color',[0 0 0],'DisplayName','Output: reachableSet')
    else
        plot(startPoints(:,1),startPoints(:,2),'.-','Color',[0 0 1],'MarkerSize',10,'LineWidth', 2, 'HandleVisibility','off');
        plot(reachableSet(:,1),reachableSet(:,2),'LineWidth',2,'Color',[0 0 0],'HandleVisibility','off');
    end


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

%% fcn_INTERNAL_sampleWindAtPoints
function [filteredWindDisturbances, resampledWindSamplingVertices] = ...
    fcn_INTERNAL_sampleWindAtPoints(windSamplingVertices, ...
    windFieldX, windFieldY, windFieldU, windFieldV, ...
    flagWindRoundingType) %#ok<INUSD>

%%%%
% Apply disturbance to each vertex. To do this, we first find the indices
% in the wind disturbance matrices that match the XY location of each
% vertex

if 1==1
    % The below method is very fast:
    % (Total time: 0.230 s) for 1000 runs if set flagWindRoundType = 1

    % Assume the discretization in X and Y are the same
    spatialStep = windFieldX(2)-windFieldX(1);
    xIndices = floor((windSamplingVertices(:,1)-windFieldX(1,1))/spatialStep)+1;
    yIndices = floor((windSamplingVertices(:,2)-windFieldY(1,1))/spatialStep)+1;

    highestXindex = length(windFieldX);
    highestYindex = length(windFieldY);

    % Bound the index values to 1 to length of the vectors
    xIndices = max(1,min(highestXindex,xIndices));
    yIndices = max(1,min(highestYindex,yIndices));
    
    indices = [xIndices yIndices];

else
    % This method works well. But it's slow due to the for-loop
    % (Total time: 0.376 s) for 1000 runs if set flagWindRoundType = 1
    NboundingVertices = length(windSamplingVertices);
    indices = zeros(NboundingVertices,2);

    for ith_vertex = 1:NboundingVertices
        % Find index in wind field corresponding to the point on the convex
        % hull
        thisVertex = windSamplingVertices(ith_vertex,:);
        thisX = thisVertex(1,1);
        thisY = thisVertex(1,2);


        xIndex = find(windFieldX>thisX,1,'first');
        yIndex = find(windFieldY>thisY,1,'first');

        % Check for out-of-bounds situations
        if isempty(xIndex)
            if thisX>windFieldX(1,end)
                xIndex = length(windFieldX(1,:));
            elseif thisX<windFieldX(1,1)
                xIndex = 1;
            else
                error('unknown situation occurred matching x value: %.2f to windFieldX',thisX);
            end
        end
        if isempty(yIndex)
            if thisY>windFieldY(1,end)
                yIndex = length(windFieldY(1,:));
            elseif thisY<windFieldY(1,1)
                yIndex = 1;
            else
                error('unknown situation occurred matching y value: %.2f to windFieldY',thisY);
            end
        end
        indices(ith_vertex,:) = [xIndex yIndex];
    end
end

%%%%
% Prune the bounding vertices to match the discretization of the wind
% field?
if 1==0 % 0==flagWindRoundingType
    [uniqueIndicesRaw,vertexRowsThatAreUnique] = unique(indices,'rows','stable');
    resampledWindSamplingVerticesRaw = windSamplingVertices(vertexRowsThatAreUnique,:);    

    % Repeat the first point to last, to close the boundary. The "unique"
    % function deletes this repetition
    uniqueIndices = [uniqueIndicesRaw; uniqueIndicesRaw(1,:)];
    resampledWindSamplingVertices = [resampledWindSamplingVerticesRaw; resampledWindSamplingVerticesRaw(1,:)];

else
    uniqueIndices = indices;
    resampledWindSamplingVertices = windSamplingVertices;    
end


% Convert uniqueIndices into linear indices for indexing wind fields. Note the
% switch here from rows/columns into X and Y, for the indexing, is swapping
% the order. This is because the wind fields are transposed because they
% were created with the image toolbox. A nice to-do item later would be to
% fix this so that the XY indexing is consistent with typical matrix
% representations of points as [x y]
linearInd = sub2ind(size(windFieldU),uniqueIndices(:,2),uniqueIndices(:,1));

% Index wind fields to find wind disturbance for each point
windU = windFieldU(linearInd);
windV = windFieldV(linearInd);

% windDisturbances = [filteredWindU filteredWindV];
windDisturbances = [windU windV];

% Apply disturbances and save results
% rawWindDisturbances = resampledWindSamplingVertices + windDisturbances;
rawWindDisturbances = windDisturbances;

% Smooth the outputs? (works, but very slow)
if 1==0
    filteredWindDisturbances = fcn_INTERNAL_filterData(rawWindDisturbances);
else
    filteredWindDisturbances = rawWindDisturbances;
end

end % Ends fcn_INTERNAL_sampleWindAtPoints

%% fcn_INTERNAL_filterData
function filteredOutput = fcn_INTERNAL_filterData(unfilteredData)

% See https://www.mathworks.com/matlabcentral/answers/9900-use-filter-constants-to-hard-code-filter
% z(n) = 0;       % Creates zeros if input z is omitted
% Y = zeros(size(X));
% for m = 1:length(Y)
%     Y(m) = b(1) * X(m) + z(1);
%     for i = 2:n-1
%         z(i - 1) = b(i) * X(m) + z(i) - a(i) * Y(m);
%     end
%     z(n - 1) = b(n) * X(m) - a(n) * Y(m);  % Omit z(n), which is 0
% end
% ABOVE shows: z(1) = b(2)*X(1) + z_i(2) - a(2)*(b(1)*X(1)+zi(1))
   
flag_do_debug = 0;
fig_for_debug = 38383;

if 1==flag_do_debug
    figure(fig_for_debug);
    clf;
    Ndata = length(unfilteredData(:,1));
    indicesToPlot = (1:Ndata)';
end

% HARDCODE RESULT OF: [B,A] = butter(2,0.2);
B = [0.067455273889072   0.134910547778144   0.067455273889072];
A = [1.000000000000000  -1.142980502539901   0.412801598096189];

initialConditions = unfilteredData(1:2,:);
finalConditions = unfilteredData(end-1:end,:);
filteredOutput = 0*unfilteredData;
for ith_column = 1:size(unfilteredData,2)
    thisColumn = unfilteredData(:,ith_column);
    initialCondition = initialConditions(:,ith_column);
    finalCondition = finalConditions(:,ith_column);


    filteredForward = fcn_INTERNAL_hardCodedFilter(B,A,thisColumn, initialCondition);

    % % For debugging
    % if 1==flag_do_debug
    %     figure(fig_for_debug);
    %     h_plot = plot(indicesToPlot, thisColumn,'-');
    %     hold on;
    %     plot(indicesToPlot, filteredForward,'Color',h_plot.Color);
    % end

    filteredBackward = fcn_INTERNAL_hardCodedFilter(B,A,flipud(filteredForward),flipud(finalCondition));
    filteredOutput(:,ith_column) = flipud(filteredBackward);
    filteredOutput(1:2,ith_column) = thisColumn(1:2,:); % Force initial conditions to match

    % For debugging
    if 1==flag_do_debug
        figure(fig_for_debug);
        h_plot = plot(indicesToPlot, thisColumn,'-');
        hold on;
        plot(indicesToPlot, filteredOutput(:,ith_column),'Color',h_plot.Color);
    end
end


end % Ends fcn_INTERNAL_filterData

%% fcn_INTERNAL_hardCodedFilter
function yfilt_hardcoded = fcn_INTERNAL_hardCodedFilter( B, A, rawdata, yInit)
% Implements:
% a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
%                           - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
% with explicit initial conditions on y

flag_do_debug = 0;

Aflipped = fliplr(A);


% Implements:
% a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
%                           - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
% with explicit initial conditions on y

Ndata = length(rawdata(:,1));

% Amatrix = zeros(Ndata,Ndata);
% for ith_a = 2:length(A)
%     avector = A(ith_a)*ones(Ndata-(ith_a-1),1);
%     diagonalAddition = diag(avector,(1-ith_a));
%     Amatrix = Amatrix+diagonalAddition;
% end


Bmatrix = zeros(Ndata,Ndata);
for ith_b = 1:length(B)
    bvector = B(ith_b)*ones(Ndata-(ith_b-1),1);
    diagonalAddition = diag(bvector,(1-ith_b));
    Bmatrix = Bmatrix+diagonalAddition;
end


yfilt_hardcoded = Bmatrix*rawdata;

% Fill in initial conditions:
order = length(A)-1;
yfilt_hardcoded(1:order,:) = yInit;

for ith_sample = (order+1):Ndata
    yfilt_hardcoded(ith_sample,1) = yfilt_hardcoded(ith_sample,1) - Aflipped(1:order)*yfilt_hardcoded((ith_sample-order):(ith_sample-1),:);
end

% For debugging
if 1==flag_do_debug
    figure(234343);
    indicesToPlot = (1:Ndata)';
    clf;
    h_plot = plot(indicesToPlot, rawdata,'-');
    hold on;
    plot(indicesToPlot, yfilt_hardcoded,'Color',h_plot.Color);
    matlabFilt = filter(B,A,rawdata);
    plot(indicesToPlot, matlabFilt,'Color',[0 0 0]);
end
end % Ends fcn_INTERNAL_hardCodedFilter

%% fcn_INTERNAL_expandVerticesOutward
function [pointsMovedOutward, movementsOutward, indicesKept] = fcn_INTERNAL_expandVerticesOutward(startingPoints, radius)

% URHERE - need to externally functionalize this, and test it repeatedly

%%%%
% Need to expand all vertices outward. A method to do this is to
% calculate the projection vector at each vertex that bisects the
% angle, and thus projects straight "outward". The distance of this
% vector can be calculated such that both edges move outward by the
% same radius. The code below implements this method.

% Make sure first and last points repeat
if ~isequal(startingPoints(1,:),startingPoints(end,:))
    startingPoints = [startingPoints; startingPoints(1,:)];
end

% for debugging
if 1==0
    startingPoints = [0 0; 1 0; 2 0; 3 0; 4 0; 3 0; 2 0; 2 1];
end

% Find unit edge vectors, and ortho projections. Note: if there are N
% vertices, there will be N-1 edges. So that the angles match with the
% end of each point, we repeat the 1st edge vector onto the end so the
% number of edge vectors is equal to the number of points.
edgeVectors = startingPoints(2:end,:)-startingPoints(1:end-1,:);
edgeLengths = sum(edgeVectors.^2,2).^0.5;
unitEdgeVectors = edgeVectors./edgeLengths;
allUnitEdgeVectors = [unitEdgeVectors(end,:); unitEdgeVectors; unitEdgeVectors(1,:)];

if any(isnan(allUnitEdgeVectors),'all')
    error('Unexpected error in calculating allUnitEdgeVectors: NaN value encountered - this should not happen.');
end

%%%
% Remove "jogs"

% Sometimes vectors are created that point exactly opposite each other.
% Remove these
flagKeepGoing = 1;
count = 0;
NStartingPoints = length(startingPoints(:,1));
fixedVectors = [allUnitEdgeVectors, [NStartingPoints; (1:NStartingPoints)']];

indicesRemoved = [];
while 1==flagKeepGoing
    temp = fixedVectors(1:end-1,1:2)+fixedVectors(2:end,1:2);
    tempMag = sum(temp.^2,2);
    tol = 1E-8;
    badElement = find(tempMag<tol,1);
    if ~isempty(badElement)
        indicesRemoved = [indicesRemoved; fixedVectors(badElement:(badElement+1),3)]; %#ok<AGROW>
        fixedVectors(badElement:(badElement+1),:) = [];
    else
        flagKeepGoing = 0;
    end
    count = count+1;
    if count>=NStartingPoints
        error('Unable to clean vectors - seem to be trapped while removing forward/backward pairs.');
    end
end
indicesKept = fixedVectors(2:end,3);
fixedStartingPoints = startingPoints(indicesKept,:);
if ~isequal(fixedStartingPoints(1,:),fixedStartingPoints(end,:))
    indicesKept = [indicesKept; indicesKept(1)];
    fixedStartingPoints = startingPoints(indicesKept,:);
    fixedVectors = [fixedVectors; fixedVectors(1,:)];
end
indicesRemoved = sort(indicesRemoved);

% For debugging. Should see any forward/backward jogs removed. NOTE:
% because the analysis is done on UNIT edge vectors, it may seem like some
% of the points prior to the "jog" in/out are removed. This is normal.
if 1==0
    figure(44444);
    clf;
    plot(startingPoints(:,1),startingPoints(:,2),'b.-','MarkerSize',40, 'LineWidth',5);
    hold on;
    plot(startingPoints(indicesRemoved,1),startingPoints(indicesRemoved,2),'r.','MarkerSize',30, 'LineWidth',5);
    plot(fixedStartingPoints(:,1),fixedStartingPoints(:,2),'g.-','MarkerSize',20, 'LineWidth',2);
end

%%%
% Find offsets

% The following method avoids the use of sines, cosines, and tangents
% as those are slow to calculate. To find the vector projection from
% each point, the average of the projection vector tips at each vertex
% angle is calculated. The projection scaling distance, D, is
% calculated by using the relationship that the sin(theta) = x/D = a/x,
% then solving for D and noting that x=1. Here, theta is the half angle
% formed by the "kite" apex, created by the before/after projection of
% the orthogonal vectors from each startPoint.

% Perform a -90 degree rotation
orthoEdgeVectors = fixedVectors(:,1:2)*[0 -1; 1 0];
positionsIncomingVector = fixedStartingPoints+orthoEdgeVectors(1:end-1,:);
positionsOutgoingVector = fixedStartingPoints+orthoEdgeVectors(2:end,:);
averagePositions = (positionsIncomingVector + positionsOutgoingVector)/2;

% Remove values where average positions are zero
averagePositionsMagnitude = sum(averagePositions.^2,2);
badPoints = find(averagePositionsMagnitude<1E-6);
if ~isempty(badPoints)
    averagePositions(badPoints,:) = [];
    fixedStartingPoints(badPoints,:) = [];
    indicesKept(badPoints,:) = [];
end

averagePositionVectors = averagePositions - fixedStartingPoints;
averagePositionVectorLengths = sum(averagePositionVectors.^2,2).^0.5;

% Remove values where vector lengths are zero
badPoints2 = find(averagePositionVectorLengths<1E-6);
if ~isempty(badPoints2)
    averagePositionVectors(badPoints2,:) = [];
    averagePositionVectorLengths(badPoints2,:) = [];
    fixedStartingPoints(badPoints2,:) = [];
    indicesKept(badPoints2,:) = [];
end
unitProjectionVectors = averagePositionVectors./averagePositionVectorLengths;

if any(isnan(unitProjectionVectors),'all')
    error('Unexpected error in calculating unitProjectionVectors: NaN value encountered - this should not happen.');
end


% Use the sin(theta) method to calculate D
D = 1./averagePositionVectorLengths;
movementsOutward = radius*D.*unitProjectionVectors;
RawPointsMovedOutward = fixedStartingPoints + movementsOutward;

% Make sure first and last points repeat
if ~isequal(RawPointsMovedOutward(1,:),RawPointsMovedOutward(end,:))
    RawPointsMovedOutward(end,:) = RawPointsMovedOutward(1,:);
    indicesKept(end) = indicesKept(1);
end

% Save output
pointsMovedOutward = RawPointsMovedOutward;

% For debugging. Should see all the edges move outward by radius amount,
% equally
if 1==0
    figure(3773);
    clf;
    plot(startingPoints(:,1),startingPoints(:,2),'.-','MarkerSize',20);
    hold on;
    plot(pointsMovedOutward(:,1),pointsMovedOutward(:,2),'.-','MarkerSize',20,'LineWidth',2);
    axis equal
    grid on;

end

if any(isnan(pointsMovedOutward),'all')
    error('Unexpected error in calculating pointsMovedOutward: NaN value encountered - this should not happen.');
end

end % Ends fcn_INTERNAL_expandVerticesOutward

%% fcn_INTERNAL_prepareStartPoints
function [preExpansionPoints, flagPointsFilledArtificially] = ...
    fcn_INTERNAL_prepareStartPoints(startPoints, radius)

% Keep only the unique (non-repeating) points. Do not resort them.
uniqueStartPoints = unique(startPoints,'rows','stable');

% Fill in some variables that are used for calculations that follow, for
% cases with 1 or 2 points
Npoints = length(uniqueStartPoints(:,1));
if Npoints==1 || Npoints==2
    % Define the number of directions to be considered from each value
    % of the x0 initial set, if the initial set contains only 1 or 2 points
    % Number of directions for initial expansion. Note that the first and last
    % points are the same, so effectively this divides 360 degrees by
    % (Ndirections-1)
    Ndirections = 7;
    angles = linspace(0,2*pi,Ndirections)';
    preExpansionRadius  = (0.0001*radius); 
    flagPointsFilledArtificially = 1;
else
    flagPointsFilledArtificially = 0;
end

%%%%
% Make sure that the points form an enclosed area.
% The following takes small perturbations away from the uniqueStartPoints.
% There are three cases:
% 1) There is 1 point - in this case, a very small expansion is made in all
%    directions to create an enclosed space
% 2) There are 2 points - in this case, the expansion is away from both
%    ends, creating an enclosed space that goes around both ends
% 3) There are 3+ points - in this case, the uniqueStartPoints are assumed
%    to make an enclosed space and these are used.
if Npoints==1
    preExpansionPoints  = uniqueStartPoints(1,:) + preExpansionRadius*[cos(angles) sin(angles)];
elseif Npoints==2 
    directionVector1to2 = uniqueStartPoints(2,:)-uniqueStartPoints(1,:);
    lineAngle = atan2(directionVector1to2(2),directionVector1to2(1));
    positiveOrNegative = sin(angles);
    positiveAngles = angles(positiveOrNegative>=0);
    nPositive = sum(positiveOrNegative>=0);

    preExpansionPoints = [...
        ones(nPositive,1)*uniqueStartPoints(2,:) + preExpansionRadius*[cos(positiveAngles+lineAngle-pi/2) sin(positiveAngles+lineAngle-pi/2)];
        ones(nPositive,1)*uniqueStartPoints(1,:) + preExpansionRadius*[cos(positiveAngles+lineAngle+pi/2) sin(positiveAngles+lineAngle+pi/2)];
        ];
    preExpansionPoints = [preExpansionPoints; preExpansionPoints(1,:)];

else
    %%%%
    % Points are already an enclosed space
    preExpansionPoints  = uniqueStartPoints;
end
end % Ends fcn_INTERNAL_prepareStartPoints

%% fcn_INTERNAL_resamplePointsToMatchWindDiscretization
function resampledPolyPoints = fcn_INTERNAL_resamplePointsToMatchWindDiscretization(inputPoints,stepSize)

% Make sure points circle back on each other. Otherwise, final edge never
% gets filled
if ~isequal(inputPoints(1,:),inputPoints(end,:))
    densePreExpansionPoints = [inputPoints; inputPoints(1,:)];
else
    densePreExpansionPoints = inputPoints;
end

edgeVectors = densePreExpansionPoints(2:end,:)-densePreExpansionPoints(1:end-1,:);
edgeLengths = sum(edgeVectors.^2,2).^0.5;
unitEdgeVectors = edgeVectors./edgeLengths;

resampledPolyPoints = [];
% For each vector, break vector up into stepSize chunks. If last part is
% "small", delete last point to keep stepSize chunk, or larger, in the
% segment.
for ith_edge = 1:length(unitEdgeVectors(:,1))
    thisStartPoint = densePreExpansionPoints(ith_edge,:);
    thisLength = edgeLengths(ith_edge);
    thisUnitVector = unitEdgeVectors(ith_edge,:);

    % Count how many stepSizes fit inside thisLength
    Nresamples = floor(thisLength/stepSize);

    % Make sure remainder is not "small". If it is, just reduce the cuts by
    % one so last cut is longer than others.
    remainingDistance = thisLength - Nresamples*stepSize;
    if remainingDistance < 0.5*stepSize && thisLength>stepSize
        Nresamples = Nresamples-1;
    end
    morePoints = (0:Nresamples)'*stepSize*thisUnitVector + thisStartPoint;
    resampledPolyPoints = [resampledPolyPoints; morePoints]; %#ok<AGROW>
end

% Circle back to first point
uniqueResampledPolyPoints = unique(resampledPolyPoints(1:end-1,:),'rows','stable');
resampledPolyPoints = [uniqueResampledPolyPoints; resampledPolyPoints(1,:)];
end % Ends fcn_INTERNAL_resamplePointsToMatchWindDiscretization
