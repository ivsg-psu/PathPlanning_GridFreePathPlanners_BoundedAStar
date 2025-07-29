function [cost, route] = fcn_BoundedAStar_windRadiusPlan(expandedSets, windFieldU, windFieldV, x, y, n_nodes, start, finish, varargin)
% fcn_BoundedAStar_windRadiusPlan
% plans a path through a wind field 
%
% FORMAT:
% windGraph = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, start, finish, (rngSeed), (fig_num))
%
% INPUTS:
%
%     expandedSets: a 629x2x100 matrix containing the reachable radius at
%     each time step
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
%     cost: a scalar representing the cost of the chosen path
%
%     route: a vector containing the chosen path through the map
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
% 2025_07_24 by K. Hayes
% -- first write of function

%
% TO-DO
% (none)

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

%%%% Create graph connecting reachable nodes from each 

% Reshape the matrix containing all reached points
% vertices = reshape(expandedSets, [size(expandedSets,1)*size(expandedSets,3), size(expandedSets,2)]);
vertices = [];
for i = 1:size(expandedSets,3)
    vertices = vertcat(vertices,expandedSets(:,:,i));
end

% Assign start/finish
start = [0 0 length(vertices)+1 -1 0];
finish = [-2 6 length(vertices)+2 -1 0];

vertices(end+1,:) = [start(1), start(2)];
vertices(end+1,:) = [finish(1), finish(2)];

% Initialize graph
linkedGraph = nan*ones(size(vertices,1));

% Create graph
for i = 2:size(vertices,1)-1
    for j = 2:size(vertices,1)-1
        % Attach the same point in the next time step to this point
        if j+63 < size(vertices,1) && i == j
            linkedGraph(i,j+63) = 1;
        end
        
        % Attach the adjacent points to each other
        if j == i+1
            linkedGraph(i,j) = 1;
        elseif j == i-1
            linkedGraph(i,j) = 1;
        end
    end
end

% Create cost graph
all_pts = [vertices(:,1), vertices(:,2), [1:length(vertices)]', -1*ones(length(vertices),1), zeros(length(vertices),1)];
[cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, "xy spatial only");

% Plan path
[cost, route] = fcn_BoundedAStar_Astar(linkedGraph, cgraph, hvec, all_pts, start, finish, (1561));

% Plotting code
figure
hold on
grid on
for i = 1:size(vertices,1)
    for j = 1:size(vertices,1)
        if linkedGraph(i,j) == 1
                plot([vertices(i,1), vertices(j,1)], [vertices(i,2), vertices(j,2)],'-','Color','black','LineWidth',3)
                drawnow            
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
