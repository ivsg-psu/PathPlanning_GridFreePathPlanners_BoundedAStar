function reachableSet = fcn_BoundedAStar_reachabilityWithInputs(radius, windFieldU, windFieldV, windFieldX, windFieldY, varargin)
% fcn_BoundedAStar_reachabilityWithInputs
% calculates the reachable set from a given startPoints,accounting
% for control inputs and the wind field 
%
% FORMAT:
% reachableSet = fcn_BoundedAStar_reachabilityWithInputs(...
%     radius, ... 
%     windFieldU,  ...
%     windFieldV,  ...
%     windFieldX,  ...
%     windFieldY,  ...
%     (startPoints),  ...
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

flag_do_debug = 1;

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

NstartPoints = length(startPoints(:,1));

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

%%%%
% Calculate the state propogation based on dynamics. 

% Create state matrices
% For now, this is
% trivial: x_(k+1) = A*x_k for A=identity gives x_(k+1) = x(k)
% A - the vehicle remains in the same position unless moved
A = eye(NstartPoints);

% The following does: x_(k+1) = A*x_k 
movedPoints = A*startPoints;

if flag_do_debug
    figure(debug_figNum);
    plot(movedPoints(:,1),movedPoints(:,2),'.-','Color',[0 1 0],'MarkerSize',20);
end

%%%%
% Calculate the inputs for each state output. This is going to become the
% B*u term. For now, assume B*u is an arbitrary direction, essentially
% pushing all points "outward" from wherever they are.

% Define the number of directions to be considered from each value 
% of the x0 initial set
Ndirections = 360; % One for every degree
angles = linspace(0,2*pi,Ndirections)';
angles = angles(1:end-1,:);
controlInputPerturbations = radius*[cos(angles) sin(angles)];

% % For each state, there will be Ndirections "pushes" to add. So we need to
% % replicate each state Ndirections times. To do this, we change the Nx2
% % matrix into (2N x 1) column format: [x1; y1; x2; y2; etc] and then repeat
% % this matrix Ndirection times in the "row" direction
% controlInputPerturbationsRepeatedNstartPoints = repmat(controlInputPerturbations,NstartPoints,1);
% movedStatesSingleColumn = repmat(movedPoints',Ndirections,1);
% movedStatesRepeatedNdirections = reshape(movedStatesSingleColumn,2,[])';
% 
% % Perform the addition of control inputs
% movedStartStates = movedStatesRepeatedNdirections + controlInputPerturbationsRepeatedNstartPoints;
% 
% if flag_do_debug
%     figure(debug_figNum);
%     plot(movedStartStates(:,1),movedStartStates(:,2),'.-','Color',[1 0 1],'MarkerSize',20);
% end

%%%%
% For each of the points, convert its expansion into a polytope, and merge
% them all into one polytope

% Fill in first value with the region defined by the startPoints, if there
% are enough startPoints
if length(startPoints(:,1))>2
    boundingPolytope = polyshape(startPoints);
    firstIndex = 1;
else
    polyPoints = repmat(movedPoints(1,:),Ndirections-1,1) + controlInputPerturbations;
    boundingPolytope = polyshape(polyPoints);
    firstIndex = 2;
end
if flag_do_debug
    figure(debug_figNum);
    h_allPoly = plot(boundingPolytope);
end

% Merge the polytopes created by the expansion of each of the start points
for ith_start = firstIndex:NstartPoints
    polyPoints = repmat(movedPoints(ith_start,:),Ndirections-1,1) + controlInputPerturbations;
    thisPoly = polyshape(polyPoints);
    boundingPolytope = union(boundingPolytope,thisPoly);
    if flag_do_debug
        figure(debug_figNum);
        set(h_allPoly,'Visible','off');
        plot(boundingPolytope);
    end
end

% Pull the polytope vertices
boundingPolytopeVertices = boundingPolytope.Vertices;

%%%%
% Apply disturbance to each vertex. To do this, we first find the indices
% in the wind disturbance matrices that match the XY location of each
% vertex
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

% Convert indices into linear indices for indexing wind fields. Note the
% switch here from rows/columns into X and Y, for the indexing, is swapping
% the order. This is because the wind fields are transposed because they
% were created with the image toolbox. A nice to-do item later would be to
% fix this so that the XY indexing is consistent with typical matrix
% representations of points as [x y]
linearInd = sub2ind(size(windFieldU),indices(:,2),indices(:,1));

% Index wind fields to find wind disturbance for each point
Wu = windFieldU(linearInd);
Wv = windFieldV(linearInd);
W = [Wu Wv];

% Apply disturbances and save results
reachableSet = boundingPolytopeVertices + W;

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
    dimension_of_points = 2; 

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
    goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end   

    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',debug_figNum)
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6])

    % Plot the start points
    plot(startPoints(:,1),startPoints(:,2),'.-','Color',[0 0 1],'MarkerSize',30,'DisplayName','Input: startPoints');

    % Plot the final output
    plot(reachableSet(:,1),reachableSet(:,2),'LineWidth',2,'Color','black','DisplayName','Output: reachableSet')

    % Turn on legend
    legend('Interpreter','none','Location','best');

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

%% fcn_INTERNAL_plotCircle
function circle_points = fcn_INTERNAL_plotCircle(centers,radii)
% COPIED OUT OF: fcn_geometry_plotCircle in GeomClass library

% Use number of radii to calculate the number of centers
Ncircles = length(centers(:,1));

% Set angles for plotting
angles = (0:0.01:2*pi)';

% Loop through the arcs, prepping data for plotting each
if Ncircles>1
    circle_points{Ncircles} = [];
end
for ith_circle = 1:Ncircles 

    xdata = centers(ith_circle,1)+radii(ith_circle)*cos(angles);
    ydata = centers(ith_circle,2)+radii(ith_circle)*sin(angles);

    x_arc = xdata; % [x_arc; NaN; xdata]; %#ok<AGROW>
    y_arc = ydata; %[y_arc; NaN; ydata]; %#ok<AGROW>

    if Ncircles==1
        circle_points = [x_arc y_arc];
    else
        circle_points{ith_circle} = [x_arc y_arc];
    end
end
end % Ends fcn_INTERNAL_plotCircle