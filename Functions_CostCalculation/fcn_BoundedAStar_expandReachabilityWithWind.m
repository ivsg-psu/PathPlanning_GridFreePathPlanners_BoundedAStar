function finalReachableSet = fcn_BoundedAStar_expandReachabilityWithWind(radius, windFieldU, windFieldV, windFieldX, windFieldY, varargin)
% fcn_BoundedAStar_expandReachabilityWithWind
% performs iterative reachability expansion from a startPoint until one of
% the following criteria are met:
%     1. Entire wind field is covered
%     2. The expansion stalls where each iteration is giving same result
%     3. All user-given goal points are hit
%     4. One user-given goal points is hit
%
% FORMAT:
% finalReachableSet = fcn_BoundedAStar_expandReachabilityWithWind(...
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
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * Added option to only update "jogs" when threshold is 120 degrees
%   % * Fixes bug seen in some map expansions


% TO-DO
% -- update header

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 8; % The largest Number of argument inputs to the function
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
if 1 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        startPoints = temp;
    end
end

% Does user want to specify flagWindRoundingType input?
flagWindRoundingType = 0; % Default is 0
if 2 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        flagWindRoundingType = temp;
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

end

% Estimate number of steps
XYvectors = [windFieldX' windFieldY'];
maxXYs = max(XYvectors,[],1,'omitmissing');
minXYs = min(XYvectors,[],1,'omitmissing');
range = maxXYs - minXYs;
longestPossibleDistance = sum(range.^2,2).^0.5;

windMagnitude = (windFieldU.^2+windFieldV.^2).^0.5;
maxWindSpeed = max(windMagnitude,[],'all','omitmissing');
slowestPossibleSpeeds = radius - maxWindSpeed;

if slowestPossibleSpeeds<0
    warning(['The wind field is strong enough that some portions ' ...
        'have wind faster than the fastest vehicle speed. This may ' ...
        'produce unreachable areas and thus mission goals that are not ' ...
        'feasible. For estimation, the flight speed will be used.']);
    slowestPossibleSpeeds = radius;
end

minX = windFieldX(1);
maxX = windFieldX(end);
minY = windFieldY(1);
maxY = windFieldY(end);
deltaX = windFieldX(2) - windFieldX(1);

maxNsteps = longestPossibleDistance/slowestPossibleSpeeds; %#ok<NASGU>
Nsteps = 100; % maxNsteps;
allExpansions = cell(Nsteps,1);

for ith_step = 1:Nsteps
    allExpansions{ith_step,1} = startPoints;
    % if ith_step ==35
    %     disp('Stop here');
    % end

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
    startPointsSparse = fcn_INTERNAL_sparsifyPoints(noJogPoints,deltaX);

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

    startPoints = newStartPoints;
end

allExpansions{Nsteps+1,1} = newStartPoints;

finalReachableSet = newStartPoints;

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

    % Plot the expansion sets
    % allColors = parula(Nsteps+1);
    allColors = turbo;
    Ntotal = 25;
    for ith_expansion = 1:Nsteps+1
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

%% fcn_INTERNAL_sampleWindAtPoints
function [filteredResampledBoundingPolytopeVerticesWithWind, resampledBoundingPolytopeVertices] = ...
    fcn_INTERNAL_sampleWindAtPoints(boundingPolytopeVertices, ...
    windFieldX, windFieldY, windFieldU, windFieldV, ...
    flagWindRoundingType)

%%%%
% Apply disturbance to each vertex. To do this, we first find the indices
% in the wind disturbance matrices that match the XY location of each
% vertex

if 1==1
    % The below method is very fast:
    % (Total time: 0.230 s) for 1000 runs if set flagWindRoundType = 1

    % Assume the discretization in X and Y are the same
    spatialStep = windFieldX(2)-windFieldX(1);
    xIndices = floor((boundingPolytopeVertices(:,1)-windFieldX(1,1))/spatialStep)+1;
    yIndices = floor((boundingPolytopeVertices(:,2)-windFieldY(1,1))/spatialStep)+1;

    highestXindex = length(windFieldX);
    highestYindex = length(windFieldY);

    % Bound the index values to 1 to length of the vectors
    xIndices = max(1,min(highestXindex,xIndices));
    yIndices = max(1,min(highestYindex,yIndices));
    
    indices = [xIndices yIndices];

else
    % This method works well. But it's slow due to the for-loop
    % (Total time: 0.376 s) for 1000 runs if set flagWindRoundType = 1
    NboundingVertices = length(boundingPolytopeVertices);
    indices = zeros(NboundingVertices,2);

    for ith_vertex = 1:NboundingVertices
        % Find index in wind field corresponding to the point on the convex
        % hull
        thisVertex = boundingPolytopeVertices(ith_vertex,:);
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
if 0==flagWindRoundingType
    [uniqueIndicesRaw,vertexRowsThatAreUnique] = unique(indices,'rows','stable');
    resampledBoundingPolytopeVerticesRaw = boundingPolytopeVertices(vertexRowsThatAreUnique,:);    

    % Repeat the first point to last, to close the boundary. The "unique"
    % function deletes this repetition
    uniqueIndices = [uniqueIndicesRaw; uniqueIndicesRaw(1,:)];
    resampledBoundingPolytopeVertices = [resampledBoundingPolytopeVerticesRaw; resampledBoundingPolytopeVerticesRaw(1,:)];

else
    uniqueIndices = indices;
    resampledBoundingPolytopeVertices = boundingPolytopeVertices;    
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

% W = [filteredWindU filteredWindV];
W = [windU windV];

% Apply disturbances and save results
rawResampledBoundingPolytopeVerticesWithWind = resampledBoundingPolytopeVertices + W;

% Smooth the outputs? (works, but very slow)
if 1==0
    filteredResampledBoundingPolytopeVerticesWithWind = fcn_INTERNAL_filterData(rawResampledBoundingPolytopeVerticesWithWind);
else
    filteredResampledBoundingPolytopeVerticesWithWind = rawResampledBoundingPolytopeVerticesWithWind;
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