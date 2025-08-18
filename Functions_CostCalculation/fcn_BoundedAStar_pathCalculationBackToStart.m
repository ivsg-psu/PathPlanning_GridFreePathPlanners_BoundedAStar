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

flag_do_debug = 1;

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
        'DisplayName','Input: startPoints');

end

%% Find set that the endPoint is contained by
Nexpansions = length(cellArrayOfExpansions);
flagEndPointContained = nan(Nexpansions,1);
for ith_expansion = 1:Nexpansions
    thisRegion = cellArrayOfExpansions{ith_expansion};
    flagEndPointContained(ith_expansion,1) = ...
        fcn_INTERNAL_findGoalPointsHit(thisRegion,endPoint);
    if flag_do_debug && ~isempty(thisRegion)
        figure(debug_figNum);
        h_plot1 = plot(thisRegion(:,1),thisRegion(:,2),'.-',...
            'Color',[0 1 0],'MarkerSize',20);
        if ith_expansion==1
            set(h_plot1,'DisplayName','Expansions');
        else
            set(h_plot1,'HandleVisibility','off');
        end
    end
    legend('Interpreter','none','Location','best');

end
stepContainingEndPoint = find(flagEndPointContained,1,'first');
if isempty(stepContainingEndPoint)
    error('endPoint not containted within any of the given set sequences');
end

pathXYAndControlUV = nan(stepContainingEndPoint,4);
for ith_step = stepContainingEndPoint:-1:1
    preExpansionPoints            = cellArrayOfIntermediateCalculations{ith_step,1};
    xKPlusOne_AMatrixPoints       = cellArrayOfIntermediateCalculations{ith_step,2};
    xKPlusOne_BMatrixPoints       = cellArrayOfIntermediateCalculations{ith_step,3};
    xKPlusOne_WindDisturbance     = cellArrayOfIntermediateCalculations{ith_step,4};
    flagPointsFilledArtificially  = cellArrayOfIntermediateCalculations{ith_step,5};

    if flag_do_debug 
        figure(debug_figNum);
        h_plot2 = quiver(preExpansionPoints(:,1), preExpansionPoints(:,2),...
            xKPlusOne_AMatrixPoints(:,1),xKPlusOne_AMatrixPoints(:,2),0,...
            'LineWidth',3,...
            'ShowArrowHead','on','MaxHeadSize',1,...
            'Color',0.8*[1 0 0],'MarkerSize',20);
        shiftedPoints = preExpansionPoints+xKPlusOne_AMatrixPoints;
        h_plot3 = quiver(shiftedPoints(:,1), shiftedPoints(:,2),...
            xKPlusOne_BMatrixPoints(:,1),xKPlusOne_BMatrixPoints(:,2),0,...
            'LineWidth',3,...
            'ShowArrowHead','on','MaxHeadSize',1,...
            'Color',0.6*[1 0 0],'MarkerSize',20);
        shiftedPoints = preExpansionPoints+xKPlusOne_AMatrixPoints+xKPlusOne_BMatrixPoints;
        h_plot4= quiver(shiftedPoints(:,1), shiftedPoints(:,2),...
            xKPlusOne_WindDisturbance(:,1),xKPlusOne_WindDisturbance(:,2),0,...
            'LineWidth',3,...
            'ShowArrowHead','on','MaxHeadSize',1,...
            'Color',0.4*[1 0 0],'MarkerSize',20);
        h_plot1 = plot(preExpansionPoints(:,1),preExpansionPoints(:,2),'.-',...
            'Color',[1 0 0],'MarkerSize',20);
        if ith_expansion==1
            set(h_plot1,'DisplayName','preExpansionPoints');
            set(h_plot1,'DisplayName','xKPlusOne_AMatrixPoints');
            set(h_plot1,'DisplayName','xKPlusOne_BMatrixPoints');
            set(h_plot1,'DisplayName','xKPlusOne_WindDisturbance');
        else
            set(h_plot1,'HandleVisibility','off');
            set(h_plot2,'HandleVisibility','off');
            set(h_plot3,'HandleVisibility','off');
            set(h_plot4,'HandleVisibility','off');
        end
    end

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

if flag_do_debug
    figure(debug_figNum);
    plot(xKPlusOne_AMatrixPoints(:,1),xKPlusOne_AMatrixPoints(:,2),'.-',...
        'Color',[0 1 0],'MarkerSize',20,'DisplayName','xKPlusOne_AMatrixPoints');
end

%% Calculate the effects of inputs. 

% This represents the B*u term. For now, assume B*u is an arbitrary
% direction, essentially pushing all preExpansionPoints "outward" from
% wherever they are.
[~, xKPlusOne_BMatrixPoints] = fcn_INTERNAL_expandVerticesOutward(preExpansionPoints, radius);

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
    plot(reachableSet(:,1),reachableSet(:,2),'y-','LineWidth',3);

end

%% Save intermediate calculations
%             preExpansionPoints: the xK points, after simplification
%             xKPlusOne_AMatrixPoints: the A matrix calculation result
%             xKPlusOne_BMatrixPoints: the B matrix calculation result
%             xKPlusOne_WindDisturbance: the wind disturbances
cellArrayOfIntermediateCalculations = cell(4,1);
cellArrayOfIntermediateCalculations{1,1} = preExpansionPoints;
cellArrayOfIntermediateCalculations{2,1} = xKPlusOne_AMatrixPoints;
cellArrayOfIntermediateCalculations{3,1} = xKPlusOne_BMatrixPoints;
cellArrayOfIntermediateCalculations{4,1} = xKPlusOne_WindDisturbance;
cellArrayOfIntermediateCalculations{5,1} = flagPointsFilledArtificially;


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
    plot(startPoints(:,1),startPoints(:,2),'.-','Color',[0 0 1],'MarkerSize',10,'LineWidth', 2, 'DisplayName','Input: startPoints');

    % Plot the final output
    plot(reachableSet(:,1),reachableSet(:,2),'LineWidth',2,'Color',[0 0 0],'DisplayName','Output: reachableSet')

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
    uniquePoints = flipud(regionBoundary); % for some silly reason, polyshape takes points "backwards" (?!)
    uniquePoints = unique(uniquePoints,'rows','stable');
    region = polyshape(uniquePoints(1:end-1,:),'KeepCollinearPoints', true,'Simplify', false);
    goalPointsHit = isinterior(region,allGoalPointsList);
end
end % Ends fcn_INTERNAL_findGoalPointsHit