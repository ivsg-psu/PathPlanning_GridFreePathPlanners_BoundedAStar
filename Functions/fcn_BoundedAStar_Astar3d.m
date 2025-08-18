function [cost, route] = fcn_BoundedAStar_Astar3d(vgraph, cgraph, hvec, all_pts, start, finish, varargin)
% fcn_BoundedAStar_Astar3d
%
% A minimal version of the A* algorithm for graph searching.  Designed to contain minimal subproceses e.g. visibility graph
% This assumes points are 3D e.g. having an x, y, and z or t dimensions
%
% FORMAT:
% [cost, route] = fcn_BoundedAStar_Astar3d(vgraph, cgraph, hvec, all_pts, start, finish, (fig_num))
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
%   start: the start point vector (x,y,t,id)
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,t,id)
%
%   all_pts: the (n-2)x4 point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,t,id)
%
%   (optional inputs)
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
%
% OUTPUTS:
%
%     cost: the total cost of the selected route
%
%    route: the matrix as produced by fcn_BoundedAStar_Astar3d consisting of waypoints.  Each row is a
%    waypoint, and each column is x, y, t, and point ID
%
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
%    also, several functions exist to create visibility matrices and fcn_BoundedAStar_generateCostGraph can create cost matrices (cgraph) and heuristic cost vectors (hvec)
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_AStar3d
% for a full test suite.
%
% This function was written on summer 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, summer by Steve Harnett
% -- first write of function
% 2023, December by Steve Harnett
% -- remove cost graph generation code and place in fcn_algorithm_generate_cost_graph
% 2025_07_03 by S. Brennan
% -- renamed function to fcn_BoundedAStar_Astar3d from
%    fcn_algorithm_Astar3d
% 2025_07_25 by K. Hayes, kxh1031@psu.edu
% -- fixed formatting and function header details
% 2025_08_18 by K. Hayes
% -- added debug plotting 

% TO DO:
% -- fill in to-do items here.

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

        % Check the all_points input, make sure it has 4 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '4column_of_numbers');

        % Check the start input, make sure it has 4 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '4column_of_numbers');

        % Check the finish input, make sure it has 4 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '4column_of_numbers');

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
    route = [];
    % set the diagonal to 0 because while points are technically visible from
    % themselves, A* should not consider them as such else the lowest cost
    % neighbor of any point is itself
    vgraph = vgraph - eye(size(vgraph,1));

    % init the open set, put the start in the open set
    num_nodes = size(vgraph,1); % number of nodes in the cost graph
    open_set = nan(1,num_nodes);
    open_set(start(4)) = start(4); % only store the ID not the whole point

    % make new all pts list including start and end
    all_pts_plus_start_and_fin = [all_pts; start; finish];
    all_pts_plus_start_and_fin = sortrows(all_pts_plus_start_and_fin,4);

    % make cost matrix, g - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
    possible_gs = cgraph; % the cost graph is all the possible g values that can be added to the open set
    open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
    open_set_gs(start(4)) = possible_gs(start(4),start(4)); % g-value for nodes in open set.  g is the movement cost to

    % make heuristic vector, h - WARNING h and g must measure the same thing (e.g. the heuristic cannot be time while the actual cost, g, is distance)
    hs = hvec;

   % total cost f, is g for the open set nodes plus the corresponding h
    open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.

    % Initialize the closed list
    closed_set = nan(1,num_nodes);

    % Init. array to track the parent (predecessor) of each node
    % this represents the cheapest way to get to the node and is necessary to
    % reconstruct the cheapest path
    parents = nan(1,num_nodes);

    nodes_expanded = 0;
    nodes_explored = 0;
    % while the open list is not empty
    % the condition implies at least one nan
    while (sum(isnan(open_set)) > 0)
        % find the node with the least f on
        % the open list, call it "q"
        nodes_expanded = nodes_expanded + 1;
        [f_of_q, idx_of_q] = min(open_set_fs);
        q = all_pts_plus_start_and_fin(idx_of_q,:);

        % pop q off the open list
        open_set_fs(idx_of_q) = inf;
        open_set(idx_of_q) = NaN;

        % generate q's successors (points reachable from q)
        qs_row = vgraph(idx_of_q,:);
        successor_idxs = [];
        successor_idxs = find(qs_row);

    % the following block of code sorts the successors so they are explored
    % in ascending cost order.  This is important for the multiple goal
    % case as if multiple successors are goals, the lowest cost
    % option may not be selected without this
        gs_from_q = possible_gs(q(4),:);
        gs_from_q_to_successors = gs_from_q(successor_idxs);
        successors_with_gs = [gs_from_q_to_successors' successor_idxs'];
        successors_with_gs_sorted = sortrows(successors_with_gs);
        successor_idxs = successors_with_gs_sorted(:,2);

        % for each successor...
        for i = 1:length(successor_idxs)
            nodes_explored = nodes_explored + 1;
            successor = all_pts_plus_start_and_fin(successor_idxs(i),:);

            % check if this successor is the goal, if so we're done
            if ismember(successor(4), finish(:,4))
                %% execute code to recover path
                % total path cost is the cost so far to reach q, plus the distance
                % from q to the goal
                cost = open_set_gs(idx_of_q) + hs(idx_of_q);

                % in the multiple finish case, we need to know which finish was selected
                selected_finish_idx = find(successor(4) == finish(:,4));

                % initialize the route consisting of q and the finish
                route = [q; finish(selected_finish_idx,:)];
                cur_pt_idx = q(4);

                % then walk back through the parents array until the start is reached
                % recall the parent array contains the lowest cost predecessor to each node
                % at that nodes ID (i.e. parents(5) = 3 implies the best way to reach 5 is through 3,
                % thus you could then look at parents(3) to find the best way to reach 3 until you have
                % reached the start and therefore recovered the optimal path)
                if cur_pt_idx == start(4)
                    return
                end
                while parents(cur_pt_idx) ~= start(4)
                    % add cur_pt's parent to the path
                    parent = all_pts_plus_start_and_fin(parents(cur_pt_idx),:);
                    route = [parent; route];
                    % set current point to the current point's parent
                    cur_pt_idx = parents(cur_pt_idx);
                end

                % return the route
                route = [start; route];
                sprintf('total nodes expanded: \n %0f',nodes_expanded)
                sprintf('total nodes explored: \n %0f',nodes_explored)
                break
            else
                % if the finish is not a successor of q, find the cost of reaching the successor via q
                % this is the cost to reach q + the cost from q to successor
                tentative_cost = open_set_gs(idx_of_q) + possible_gs(q(4),successor(4));%sqrt((successor(1) - q(1)).^2 + ((successor(2) - q(2)).^2));
                % if this is less than the last recorded cost to reach successor,
                % update the cost to reach successor, add successor to the open set,
                % and set successor's parent to q

                if tentative_cost < open_set_gs(successor(4))
                    parents(successor(4)) = idx_of_q;
                    open_set_gs(successor(4)) = tentative_cost;
                    open_set_fs(successor(4)) = tentative_cost + hs(successor(4));
                    open_set(successor(4)) = successor(4);
                end % end tentative cost comparison check

        end % end looping through successors

        % having checked all of q's successors, push q on the closed list
        closed_set(idx_of_q) = idx_of_q;
    end % end while loop through open set
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
    figure(fig_num)
    hold on;
    box on;
    view(3);
    INTERNAL_fcn_format_timespace_plot();

    plot3(route(:,1),route(:,2),route(:,3),'-b','LineWidth',2);
    

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


function INTERNAL_fcn_format_timespace_plot()
% define figure properties
opts.width      = 12;
opts.height     = 9;
opts.fontType   = 'Times New Roman';
opts.fontSize   = 12;
fig = gcf;
% scaling
fig.Units               = 'centimeters';
fig.Position(3)         = opts.width;
fig.Position(4)         = opts.height;
set(gcf,'color','white')
% set text properties
set(fig.Children, ...
    'FontName',     'Times New Roman', ...
    'FontSize',     12);

% remove unnecessary white space
set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
xlabel('x [km]')
ylabel('y [km]')
zlabel('t [min]')
view([36 30])
end