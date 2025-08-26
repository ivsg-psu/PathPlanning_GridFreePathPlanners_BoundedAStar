function pathXY = ...
    fcn_BoundedAStar_pathCalculation(startPoints, controlInputs, ...
    windFieldU, windFieldV, windFieldX, windFieldY, varargin)
% fcn_BoundedAStar_pathCalculation
%  given a startPoint and controlInputs, finds the resulting pathXY
%
% FORMAT:
% pathXY = ...
% fcn_BoundedAStar_pathCalculation(...
%     startPoints, controlInputs, ...
%     windFieldU,  ...
%     windFieldV,  ...
%     windFieldX,  ...
%     windFieldY,  ...
%     (figNum));
%
% INPUTS:
%
%     startPoints: a 1x2 vector representing the [x,y] values of the start
%     point, or Mx2 vector representing many start points. Defaults to
%     [0,0] if empty value is entered
%
%     controlInputs: a Nx2 vector representing the [U V] values of the
%     control input vector, in the X and Y directions respectively. N is
%     the number of simulation time steps to perform. NOTE: the N=1 value
%     of the path is the startPoint
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
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     pathXY: an Nx2 set of [X Y] points defining the resulting location
%     produced by the control inputs and wind. If there are M different
%     start points, produces a (Mx1) cellArray of paths
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_pathCalculation
% for a full test suite.
%
% This function was written on 2025_08_18 by S. Brennan
% Questions or comments? contact sbrennan@psu.edu

% REVISION HISTORY:
% 2025_08_18 by S. Brennan
% - In fcn_BoundedAStar_pathCalculation
%   % * first write of function
%   % * using fcn_BoundedAStar_matrixEnvelopeExpansion as a starter

% TO-DO
% -- update header

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 7; % The largest Number of argument inputs to the function
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
        narginchk(6,MAX_NARGIN);

        % Check the startPoint input, make sure it is Nx2
        fcn_DebugTools_checkInputsToFunctions(...
            startPoints, '2column_of_numbers',[1 2]);

        if length(startPoints(:,1))>1
            assert(iscell(controlInputs));
            for ith_input = 1:length(controlInputs)
                % Check the controlInputs input, make sure it is Nx2
                fcn_DebugTools_checkInputsToFunctions(...
                    controlInputs{ith_input}, '2column_of_numbers',[1 2]);
            end
        else
            % Check the controlInputs input, make sure it is Nx2
            fcn_DebugTools_checkInputsToFunctions(...
                controlInputs, '2column_of_numbers',[1 2]);
        end

        % windFieldU, windFieldV, windFieldX, windFieldY

    end
end

% Does user specify startPoints input?
if isempty(startPoints)
    startPoints = [0 0]; % Default is origin
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin)
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
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
    set(s,'Color',[0.6 0.6 0.6],'HandleVisibility','off')

    % Plot the startPoint
    plot(startPoints(:,1),startPoints(:,2),'.',...
        'Color',[0 0 1],'MarkerSize',40,...
        'DisplayName','Input: startPoint');

    legend('Interpreter','none','Location','best');

end

Nstates = 1;
NStartPoints = length(startPoints(:,1));
cellArrayOfPathsXY = cell(NStartPoints,1);

if iscell(controlInputs)
    cellArrayOfControlInputs = controlInputs;
else
    cellArrayOfControlInputs = {controlInputs};
end

% Loop through each startPoint
for ith_startPoint = 1:NStartPoints
    thisControlInputs = cellArrayOfControlInputs{ith_startPoint,1};
    Nsteps = length(thisControlInputs(:,1));
    pathXY = nan(Nsteps+1,2);
    currentPoint = startPoints(ith_startPoint,:);
    pathXY(1,:) = currentPoint;
    for ith_step = 1:Nsteps
        %%%%
        % Calculate the state propogation based on dynamics.

        % Create state matrices
        % For now, this is
        % trivial: x_(k+1) = A*x_k for A=identity gives x_(k+1) = x(k)
        % A - the vehicle remains in the same position unless moved
        A = eye(Nstates);

        % The following does: x_(k+1) = A*x_k
        xKPlusOne_AMatrixPoints = A*currentPoint;

        if flag_do_debug
            figure(debug_figNum);
            h_plot = plot(xKPlusOne_AMatrixPoints(:,1),xKPlusOne_AMatrixPoints(:,2),'.-',...
                'Color',[0 1 0],'MarkerSize',20);
            if ith_step==1
                set(h_plot,'DisplayName','xKPlusOne_AMatrixPoints');
            else
                set(h_plot,'HandleVisibility','off');
            end
        end

        %%%%
        % Calculate the effects of inputs.
        % This represents the B*u term. For now, assume B*u is an arbitrary
        % direction, essentially pushing all preExpansionPoints "outward" from
        % wherever they are.
        xKPlusOne_BMatrixPoints = thisControlInputs(ith_step,:);

        if flag_do_debug
            figure(debug_figNum);
            summedPoints = xKPlusOne_AMatrixPoints+xKPlusOne_BMatrixPoints;
            h_plot = plot(summedPoints(:,1),summedPoints(:,2),'.-',...
                'Color',[1 0 1],'MarkerSize',20);
            if ith_step==1
                set(h_plot,'DisplayName','xKPlusOne_BMatrixPoints');
            else
                set(h_plot,'HandleVisibility','off');
            end
        end

        %%%%
        % Calculate wind disturbance
        % This calculates the effect of the wind input

        % FORMAT:
        % function [filteredWindDisturbances, resampledWindSamplingVertices] = ...
        %     fcn_INTERNAL_sampleWindAtPoints(windSamplingVertices, ...
        %     windFieldX, windFieldY, windFieldU, windFieldV, ...
        %     flagWindRoundingType) %#ok<INUSD>
        xKPlusOne_WindDisturbance = ...
            fcn_INTERNAL_sampleWindAtPoints(currentPoint, ...
            windFieldX, windFieldY, windFieldU, windFieldV,0);


        %%%%
        % Add up effects to calculate new reachable set

        newPoints = ...
            xKPlusOne_AMatrixPoints + ...
            xKPlusOne_BMatrixPoints + ...
            xKPlusOne_WindDisturbance;

        if flag_do_debug
            figure(debug_figNum);

            % Do a quiver plot to show how wind moves the boundary?
            if 1==1
                Nvertices = length(newPoints(:,1));
                plotEvery = 1;
                vectorLengths = newPoints-currentPoint;
                rowsToPlot = find(mod((1:Nvertices)',plotEvery)==0);
                quiver(currentPoint(rowsToPlot,1),currentPoint(rowsToPlot,2),...
                    vectorLengths(rowsToPlot,1),vectorLengths(rowsToPlot,2),0,'filled',...
                    'Color',[1 1 0],...
                    'LineWidth',3,...
                    'ShowArrowHead','on','MaxHeadSize',1,...
                    'HandleVisibility','off');
            end
            h_plot = plot(newPoints(:,1),newPoints(:,2),'y.-','LineWidth',3,'MarkerSize',30);
            if ith_step==1
                set(h_plot,'DisplayName','newPoints');
            else
                set(h_plot,'HandleVisibility','off');
            end


        end

        %%%%
        % Save results
        currentPoint = newPoints;
        pathXY(ith_step+1,:) = newPoints;

    end % Ends looping through steps
    cellArrayOfPathsXY{ith_startPoint,1} = pathXY;

end % Ends looping through startPoints

if iscell(controlInputs)
    pathXY = cellArrayOfPathsXY;
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
    allPointsBeingPlotted = [newPoints; nan nan];

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
    plot(startPoints(:,1),startPoints(:,2),'.','Color',[0 0 1],'MarkerSize',20,'LineWidth', 2, 'DisplayName','Input: startPoints');

    % Plot the final paths
    for ith_path = 1:length(cellArrayOfPathsXY)
        thisPath = cellArrayOfPathsXY{ith_path,1};
        plot(thisPath(:,1),thisPath(:,2),'.-','LineWidth',2,'MarkerSize',10,...
            'DisplayName',sprintf('Output: Path %.0d',ith_path));
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

