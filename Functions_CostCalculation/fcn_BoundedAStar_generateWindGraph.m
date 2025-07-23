function [vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, n_nodes, start, finish, varargin)
% fcn_BoundedAStar_generateWindGraph
% generates a graph in a wind field with pseudorandomly placed nodes and
% directed edge costs based on the wind field vectors that the edge passes
% through
%
% FORMAT:
% windGraph = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, start, finish, (rngSeed), (fig_num))
%
% INPUTS:
%
%     windFieldU:  a matrix containing the u-direction components of the
%     wind velocity at each grid point
%
%     windFieldV:  a matrix containing the v-direction components of the
%     wind velocity at each grid point
%
%     x: a vector containing the x values assigned to each grid point
% 
%     y: a vector containing the y values assigned to each grid point 
%
%     n_nodes: a 1x1 scalar indicating the number of nodes to be placed in
%     the graph
%
%     start: a vector containing the point (x,y) values and id for the
%     starting point of the future path plan
%
%     finish: a vector containing the point (x,y) values and id for the
%     starting point of the future path plan
% 
%     
%     (optional inputs)
%
%     rngSeed: a 1x1 scalar indicating the seed to be used for node
%     generation
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     windGraph: a structure containing the generated graph nodes, edges,
%     and cost map
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_generateWindGraph
% for a full test suite.
%
% This function was written on 2025_07_15 by K. Hayes
% Questions or comments? contact kxh1031@psu.edu

% REVISION HISTORY:
% 2025_07_15 by K. Hayes
% -- first write of function
% 2025_07_22 by K. Hayes
% -- added call to plotWindField function
% -- fixed bug where edges close to edge of field would automatically get 0
%    cost values, sometimes skewing planner outcomes
% 2025_07_23 by K. Hayes
% -- fixed bug in cost calculation 
%
% TO-DO
% -- graph scaling to match any XY_range, not just 0-1 values

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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
        narginchk(7,MAX_NARGIN);

        % Check the radius input, make sure it is '1column_of_numbers'
        % type, 1 row
        % fcn_DebugTools_checkInputsToFunctions(...
        %     radius, '1column_of_numbers',[1 1]);
    end
end

% Does user want to specify rngSeed input?
rngSeed = 1; % Default is 1
if 8 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        rngSeed = temp;
    end
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

%%%%%% Halton set pull/generation code taken from
%%%%%% fcn_polytope_generation_halton_voronoi_tiling.m

% pull halton set
rng(rngSeed)
p = haltonset(2);  % pull a 2 dimensional halton sequence of points
ps = scramble(p,'RR2'); % scramble values
low_pt = 1+rngSeed;
high_pt = n_nodes+rngSeed;

% pick values from halton set
X = ps(low_pt:high_pt,:);

% assign points to windGraph structure
vertices = X;
vertices = [vertices; start(1:2); finish(1:2)];

% construct graph edges from all points to all other points
edges = ones(size(vertices,1));

% calculate cost for each edge by comparing its direction to the direction
% of the wind vectors it passes over

% calculate direction vector for each edge
edgeVectorX = nan*ones(size(edges,1));
edgeVectorY = nan*ones(size(edges,1));
for i = 1:size(edges,1)
    for j = 1:size(edges,1)
        edgeVectorX(i,j) = vertices(j,1) - vertices(i,1);
        edgeVectorY(i,j) = vertices(j,2) - vertices(i,2);
    end
end

% Make it so that the start and end have no node directly connecting them
% to each other to force path planning to occur
edges(end-1,end) = 0;
edges(end,end-1) = 0;

%%%%% This is the stupid way to do this. I took this code from the changing
%%%%% wind function to try something out. Obviously in the final version of
%%%%% a function like this you don't have a stupidly huge loop doing numerical
%%%%% steps over the wind field in the specified direction.

Nedges = numel(edges);
stepDistance = 0.1;
costgraph = nan*ones(size(edges));
for ith_edge = 1:Nedges
    this_edge = (ith_edge);
    lengthEdge = sum((edgeVectorX(this_edge) - edgeVectorY(this_edge)).^2,2).^0.5;
    NintegrationSteps = ceil(lengthEdge/stepDistance);
    % stepDistance(1) = edgeVectorX(this_edge)/NintegrationSteps;
    % stepDistance(2) = edgeVectorY(this_edge)/NintegrationSteps;
    % get i,j coordinates for vertex search
    [thisi,thisj] = ind2sub(size(edges),this_edge);
    %%%%%
    % Numerical simulation starts here. Replace later with ODE/vehicle
    % dynamics
    
    % Set initial position
    currentPosition = [vertices(thisi,:)];
    costtotal = 0;
    for ith_step = 1:NintegrationSteps
         directionInHeading = [edgeVectorX(this_edge)*stepDistance, edgeVectorY(this_edge)*stepDistance];
        %directionInHeading = [edgeVectorX(this_edge), edgeVectorY(this_edge)];

        % windFieldU, windFieldV, x, y
        xIndex = find(x>currentPosition(1,1),1,'first');
        yIndex = find(y>currentPosition(1,2),1,'first'); 

        % Make sure we didn't wander off the map!
        if isempty(xIndex) || isempty(yIndex)
            disp(xIndex), disp(yIndex), disp(thisi), disp(thisj)
            costtotal = nan;
            break;
        end
        % if xIndex<1 || xIndex>length(x)
        %     break;
        % end
        % if yIndex<1 || yIndex>length(y)
        %     break;
        % end
        % 

        windSpeedU_here =  windFieldU(xIndex,yIndex);
        windSpeedV_here =  windFieldV(xIndex,yIndex);
        windSpeedVector = [windSpeedU_here windSpeedV_here];
        directionInWind = windSpeedVector/NintegrationSteps;

        % Get vectors for cost calculation
        edgeUnitVector = [edgeVectorX(this_edge) edgeVectorY(this_edge)]/(edgeVectorX(this_edge)^2 + edgeVectorY(this_edge)^2)^0.5;
        cDotProduct = dot(edgeUnitVector,windSpeedVector);
        
        if thisi == thisj
            cost = 0;
        else
            % cost = abs(sum(directionInHeading - directionInWind));
            cost = -cDotProduct;
        end

        costtotal = costtotal + cost;
        currentPosition = directionInHeading + directionInWind;
    end
    costgraph(ith_edge) = costtotal;
end

% Rescale cost graph to minval = 0

minval = min(min(costgraph));
costgraph = costgraph - minval*ones(size(costgraph));
maxval = max(max(costgraph));

% Re-clean cost graph to make diagonal zero and clean exceptions
for i = 1:n_nodes+2
    for j = 1:n_nodes+2
        if isnan(costgraph(i,j)) 
            costgraph(i,j) = maxval
        end
        if i == j
            costgraph(i,j) = 0;
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

    costScale = max_cost - min_cost;
    percentVec = 0:0.1:1;

    colorBreakpoints = costScale*percentVec;
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
