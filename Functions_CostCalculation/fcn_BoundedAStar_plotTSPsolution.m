function fcn_BoundedAStar_plotTSPsolution(...
    startAndGoalPoints, costsFromTo, pathsFromTo, windFieldMatrices, orderedVisitSequence, varargin)
% fcn_BoundedAStar_plotTSPsolution solves the Traveling Salesman Problem
%
% FORMAT:
% fcn_BoundedAStar_plotTSPsolution(...
%    startAndGoalPoints, costsFromTo, pathsFromTo, windFieldMatrices,
%    (figNum));
%
% INPUTS:
%
%     startAndGoalPoints: an Mx2 array of the "city" points, with no
%     repeats. Each must be reachable from at least one other point. First
%     point in the list is startPoint, points in rows 2+ are goalPoints.
% 
%     costsFromTo: an MxM matrix specifying the costs to traverse
%     from a point (row) to another point (column). 
% 
%     pathsFromTo: an MxM cell array of XYUV points that lead from
%     a row point to a column point.
%
%     windFieldMatrices: specifies the wind field matrices, specifically:
%       windFieldU = windFieldMatrices{1};
%       windFieldV = windFieldMatrices{2};
%       windFieldX = windFieldMatrices{3};
%       windFieldY = windFieldMatrices{4};
%
%     orderedVisitSequence: the set of points, starting and ending with the
%     startPoint, containing the visit sequence for goalPoints.
%
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
%     (none)
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_plotTSPsolution
% for a full test suite.
%
% This function was written on 2025_09_09 by S. Brennan
% Questions or comments? contact S. Brennan sbrennan@psu.edu 
% or K. Hayes, kxh1031@psu.edu

% REVISION HISTORY:
% 2025_09_09 by S. Brennan
% - in fcn_BoundedAStar_plotTSPsolution
%   % * first write of function using fcn_BoundedAStar_solveTSP
%   %   % as a starter
%
% 2025_09_10 by S. Brennan
% - in fcn_BoundedAStar_plotTSPsolution
%   % * added shut-off of solution plotting if orderedVisitSequence is
%   empty. Defaults to plotting all trajectories.

% TO-DO
% (none)


%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 6; % The largest Number of argument inputs to the function
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
    debug_figNum = 999978; %#ok<NASGU> 
else
    debug_figNum = []; %#ok<NASGU> 
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

        % Check the startAndGoalPoints input, make sure it is Mx2
        fcn_DebugTools_checkInputsToFunctions(...
            startAndGoalPoints, '2column_of_numbers',[2 3]);

        % Check the costsFromTo input, make sure it is MxM
        M = size(startAndGoalPoints,1);
        assert(isnumeric(costsFromTo));
        assert(isequal(size(costsFromTo),[M M]));

        % Check the pathsFromTo input, make sure it is MxM
        assert(iscell(pathsFromTo));
        assert(isequal(size(pathsFromTo),[M M]));

    end
end

% % Does user want to specify cellArrayOfFunctionOptions input?
% windFieldMatrices = cell(4,1);
% windFieldMatrices{1} = []; % windFieldU
% windFieldMatrices{2} = []; % windFieldV
% windFieldMatrices{3} = []; % windFieldX
% windFieldMatrices{4} = []; % windFieldY
% if 4 <= nargin
%     temp = varargin{1};
%     if ~isempty(temp)
%         windFieldMatrices = temp;
%     end
% end
windFieldU = windFieldMatrices{1};
windFieldV = windFieldMatrices{2};
windFieldX = windFieldMatrices{3};
windFieldY = windFieldMatrices{4};

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

NumGoals   = size(startAndGoalPoints,1);
startPoint = startAndGoalPoints(1,:);
goalPoints = startAndGoalPoints(2:end,:);

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
    temp_h = figure(figNum); %#ok<NASGU>
    % flag_rescale_axis = 0;
    % if isempty(get(temp_h,'Children'))
    %     flag_rescale_axis = 1;
    % end      
    
    

    %%%%%%%%%%%%%
    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end   

    % Turn on legend
    legend('Interpreter','none','Location','northwest');

    % Plot the windfield as an image
    fcn_BoundedAStar_plotWindField(windFieldU,windFieldV,windFieldX,windFieldY,'default',figNum);
    % Get meshgrid for streamline plotting
    [meshX,meshY] = meshgrid(windFieldX,windFieldY);
    s = streamslice(meshX,meshY,windFieldU,windFieldV);
    set(s,'Color',[0.6 0.6 0.6],'HandleVisibility','off')

    colorOrder = get(gca, 'ColorOrder');
    Ncolors = length(colorOrder(:,1));

    % Plot the start point
    plot(startPoint(:,1),startPoint(:,2),'.','Color',colorOrder(1,:),'MarkerSize',30,'DisplayName','Input: startPoint');

    % Plot the goal points in different colors
    for ith_fromPoint = 1:length(goalPoints(:,1))
        thisColorRow = mod(ith_fromPoint,Ncolors)+1;
        h_plot = plot(goalPoints(ith_fromPoint,1),goalPoints(ith_fromPoint,2),'.',...
            'Color',colorOrder(thisColorRow,:),'MarkerSize',30,'LineWidth', 2);
        if ith_fromPoint ==1
            set(h_plot, 'DisplayName','Input: goalPoints');
        else
            set(h_plot,'HandleVisibility','off');
        end
    end

    % Plot the TSP result, the ordered visit sequence
    % pointSequence = feasibleAllPoints(orderedVisitSequence,:);
    if ~isempty(orderedVisitSequence) && ~isequal(orderedVisitSequence,-1)
        for ith_fromPoint = 1:length(orderedVisitSequence)-1
            thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
            thisColor = colorOrder(thisColorRow,:);

            from_index = orderedVisitSequence(ith_fromPoint,1);
            goal_index = orderedVisitSequence(ith_fromPoint+1,1);
            thisPath =  pathsFromTo{from_index,goal_index};
            h_plot = plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth',5);

            % Plot the base in green and head in red so we know directions
            plot(thisPath(1:2,1),thisPath(1:2,2),'g-','LineWidth',5,'HandleVisibility','off');
            plot(thisPath(end-1:end,1),thisPath(end-1:end,2),'r-','LineWidth',5,'HandleVisibility','off');
            % h_quiver = quiver(thisPath(end-1,1),thisPath(end-1,2),arrowMagnitude(1,1),arrowMagnitude(1,2),0,...
            %     'LineWidth',5,'Color',thisColor,'HandleVisibility','off','ShowArrowHead','on',...
            %     'MaxHeadSize',4,'AutoScale','off','AutoScaleFactor',20);

            if ith_fromPoint==1
                set(h_plot,'DisplayName','Output: TSP solution');
            else
                set(h_plot,'HandleVisibility','off');
            end
        end
    elseif orderedVisitSequence==-1
        for ith_fromPoint = 1:NumGoals
            thisColorRow = mod(ith_fromPoint-1,Ncolors)+1;
            thisColor = colorOrder(thisColorRow,:);
            from_index = ith_fromPoint;

            for jth_toPoint = 1:NumGoals
                if ith_fromPoint~=jth_toPoint
                    goal_index = jth_toPoint;
                    thisPath =  pathsFromTo{from_index,goal_index};
                    if ~isempty(thisPath)
                        plot(thisPath(:,1),thisPath(:,2),'-','Color',thisColor,'LineWidth',5,'HandleVisibility','off');

                        % Plot the base in green and head in red so we know directions
                        plot(thisPath(1:2,1),thisPath(1:2,2),'g-','LineWidth',5,'HandleVisibility','off');
                        plot(thisPath(end-1:end,1),thisPath(end-1:end,2),'r-','LineWidth',5,'HandleVisibility','off');
                    end

                end % Ends if statement to check if from/to cities are same
            end % Ends looping through "to" cities
        end % Ends looping through "from" cities
    end

    % Number the unique points
    nudge = 0.15; %sizePlot*0.006;
    for ith_fromPoint = 1:length(startAndGoalPoints)
        text(startAndGoalPoints(ith_fromPoint,1)+nudge,startAndGoalPoints(ith_fromPoint,2), ...
            sprintf('%.0f',ith_fromPoint),'FontSize',12);
    end

    % Shut the hold off?
    if flag_shut_hold_off
        hold off;
    end

    %%%%%%%%%%

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
