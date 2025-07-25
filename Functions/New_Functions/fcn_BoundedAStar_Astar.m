function [cost, route] = fcn_BoundedAStar_Astar(vgraph, cgraph, hvec, all_pts, start, finish, varargin)
% fcn_BoundedAstar_Astar
%
% A minimal version of the A* algorithm for graph searching.  Designed to contain minimal subprocesses e.g. visibility graph
%
%
%
% FORMAT:
% [cost, route] = fcn_BoundedAstar_Astar(vgraph, cgraph, hvec, all_pts, start, finish, (fig_num))
%
%
% INPUTS:
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%    cgraph: the cost graph matrix. A cost matrix is an nxn matrix where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element i-j is the cost of routing from i to j.
%
%    hvec: the heuristic cost vector. A 1xn vector where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element k is the estimated cost of routing from point k to
%      the finish based on a heuristic cost estimation method.
%
%   all_pts: the (n-2)x3 point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,id,obj_id,isStartEndPt)
%
%   start: the start point vector (x,y,id,obj_id,isStartEndPt)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,id,obj_id,isStartEndPt)
%
%
%    (optional inputs)
%
%    fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
% OUTPUTS:
%
%     cost: the total cost of the selected route
%
%    route: the matrix as produced by fcn_BoundedAstar_Astar consisting of waypoints.  Each row is a
%    waypoint, and each column is x, y, point ID, polytope/object ID, and a
%    flag indicating if the point is the start/end of the polytope
%
%
% DEPENDENCIES:
%
%    fcn_DebugTools_checkInputsToFunctions
%
%    also, several functions exist to create visibility matrices and fcn_BoundedAStar_generateCostGraph can create cost matrices (cgraph) and heuristic cost vectors (hvec)
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAstar_Astar
% for a full test suite.
%
% This function was written on spring 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, spring by Steve Harnett
% -- first write of function
% 2025_07_25 by K. Hayes, kxh1031@psu.edu
% -- reformatted function 
% -- updated function header info
% -- added input and debug checking
% -- added fig_num optional input 
%
% TO DO:
%
% -- make sure plotting follows the actual size of the data. Should work
%    for now

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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
        narginchk(6,MAX_NARGIN);

        % Check the all_points input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '5column_of_numbers');

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

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

    % set the diagonal to 0 because while points are technically visible from
    % themselves, A* should not consider them as such else the lowest cost
    % neighbor of any point is itself
    vgraph = vgraph - eye(size(vgraph,1));

    % init the open set, put the start in the open set
    num_nodes = size(vgraph,1); % number of nodes in the cost graph
    open_set = nan(1,num_nodes);
    open_set(start(3)) = start(3); % only store the ID not the whole point

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];

    % make cost matrix, g - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
    possible_gs = cgraph;
    open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
    open_set_gs(start(3)) = possible_gs(start(3),start(3)); % assign g value from possible_gs to open_set_gs for the start

    hs = hvec;

    % total cost f, is g for the open set nodes plus the corresponding h
    open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.

    % Initialize the closed list
    closed_set = nan(1,num_nodes);

    % Init. array to track the parent (predecessor) of each node
    % this represents the cheapest way to get to the node and is necessary to
    % reconstruct the cheapest path
    parents = nan(1,num_nodes);

    % while the open list is not empty...
    % the condition implies at least one nan
        while (sum(isnan(open_set)) > 0)
            % find the node with the least f on
            % the open list, call it "q"
            [f_of_q, idx_of_q] = min(open_set_fs);
            q = all_pts_plus_start_and_fin(idx_of_q,:);

            % pop q off the open list
            open_set_fs(idx_of_q) = inf;
            open_set(idx_of_q) = NaN;

            % generate q's successors (points reachable from q)
            qs_row = vgraph(idx_of_q,:);
            successor_idxs = [];
            successor_idxs = find(qs_row);

            % for each successor...
            for i = 1:length(successor_idxs)
                successor = all_pts_plus_start_and_fin(successor_idxs(i),:);
               % check if this successor is the goal, if so we're done

            if successor(3) == finish(3)
                %% execute code to recover path
                % total path cost is the cost so far to reach q, plus the distance
                % from q to the goal
                cost = open_set_gs(idx_of_q) + hs(idx_of_q);

                % initialize the route consisting of q and the finish
                route = [q; finish];

                % set the current point back one from the finish, to q
                cur_pt_idx = q(3);

                % then walk back through the parents array until the start is reached
                % recall the parent array contains the lowest cost predecessor to each node
                % at that nodes ID (i.e. parents(5) = 3 implies the best way to reach 5 is through 3,
                % thus you could then look at parents(3) to find the best way to reach 3 until you have
                % reached the start and therefore recovered the optimal path)
                if cur_pt_idx == start(3)
                    return
                end

                while parents(cur_pt_idx) ~= start(3)
                    % add cur_pt's parent to the route
                    parent = all_pts_plus_start_and_fin(parents(cur_pt_idx),:);
                    route = [parent; route];

                    % set current point to the current point's parent
                    cur_pt_idx = parents(cur_pt_idx);
                end

                % return the route
                route = [start; route];
                return
            else
            % if the finish is not a successor of q, find the cost of reaching the successor via q
            % this is the cost to reach q + the cost from q to successor
            tentative_cost = open_set_gs(idx_of_q) + possible_gs(q(3),successor(3));
            if isnan(possible_gs(q(3),successor(3)))
                my_err = sprintf('cost to go from node %i to node %i is undefined or nan',q(3),successor(3));
                error(my_err)
            end

            % if this is less than the last recorded cost to reach successor,
            % update the cost to reach successor, add successor to the open set,
            % and set successor's parent to q
            if tentative_cost < open_set_gs(successor(3))
                parents(successor(3)) = idx_of_q;
                open_set_gs(successor(3)) = tentative_cost;
                open_set_fs(successor(3)) = tentative_cost + hs(successor(3));
                open_set(successor(3)) = successor(3);
            end % end tentative cost comparison check

        end % end looping through successors
        % having checked all of q's successors, push q on the closed list
        closed_set(idx_of_q) = idx_of_q;

    end % end while loop through open set

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
    % % Prep the figure for plotting
    % temp_h = figure(fig_num);
    % flag_rescale_axis = 0;
    % if isempty(get(temp_h,'Children'))
    %     flag_rescale_axis = 1;
    % end      
    % 
    % % Is this 2D or 3D?
    % dimension_of_points = 2; 
    % 
    % % Find size of plotting domain
    % allPointsBeingPlotted = [route; nan nan nan nan nan];
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
    % goodAxis = axis;

    % Check to see if hold is already on. If it is not, set a flag to turn it
    % off after this function is over so it doesn't affect future plotting
    flag_shut_hold_off = 0;
    if ~ishold
        flag_shut_hold_off = 1;
        hold on
    end

    hold on;
    grid on;

    % Plot the polytopes
    % line_spec = 'b-'; % edge line plotting
    % line_width = 2; % linewidth of the edge
    % axes_limits = [0 1 0 1]; % x and y axes limits
    % axis_style = 'square'; % plot axes style
    % fcn_BoundedAStar_plotPolytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);

    % Plot the start and end points
    plot(start(1), start(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2,'DisplayName','Start')
    plot(finish(1), finish(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2,'DisplayName','Finish')

    % Plot the path
    plot(route(:,1), route(:,2), '-k', 'LineWidth', 2, 'DisplayName', 'Route')

    legend('Interpreter','none');
    xlabel('X-East');
    ylabel('Y-North');

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

end % end function
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