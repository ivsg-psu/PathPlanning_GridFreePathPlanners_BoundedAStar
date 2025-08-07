function [cost, route] = fcn_BoundedAStar_greedyPlanner(vgraph, all_pts, start, finish, varargin)
% fcn_BoundedAStar_greedyPlanner
%
% uses 'greedy' planner methods to plan a path through an environement
%
% FORMAT:
% [cost, route] = fcn_BoundedAStar_greedyPlanner(vgraph, all_pts, start, finish, (polytopes), (fig_num))
%
%
% INPUTS:
%
%   vgraph: the nxn visibility graph of the map, where n is the total
%   number of vertices/points 
%
%   all_pts: the nx5 list of all points in the space to be searched, with
%   the exception of the start and finish, with columns containing the
%   following information
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number (-1 if none)
%       is beginning/end of obstacle (1 if yes, 0 if no)
%
%   start: the 1x5 vector describing the start point, with the same
%   information as all_pts
%
%   finish: the 1x5 vector describing the finish point, with the same
%   information as all_pts
%
%   (optional inputs)
%
%   polytopes: a 1-by-p seven field structure of polytopes used for plotting purposes, where
%       p = number of polytopes, with fields:
%       vertices: a v+1-by-2 matrix of xy points with row1 = rowv+1, where v is
%           the number of the individual polytope vertices
%       xv: a 1-by-v vector of vertice x-coordinates
%       yv: a 1-by-v vector of vertice y-coordinates
%       distances: a 1-by-v vector of perimeter distances from one point to the
%           next point, distances(i) = distance from vertices(i) to vertices(i+1)
%       mean: average xy coordinate of the polytope
%       area: area of the polytope
%       max_radius: distance from the mean to the furthest vertex
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%   input checking or debugging, no figures will be generated, and sets
%   up code to maximize speed. As well, if given, this forces the
%   variable types to be displayed as output and as well makes the input
%   check process verbose
%
%
% OUTPUTS:
%
%   cost: the total cost of the route planned, where cost is equivalent to
%   distance
%
%   route: the list of points in the planned route
%  
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_greedyPlanner
% for a full test suite.
%
% This function was written in January 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu
%
% REVISION HISTORY:
%
% January 2024 by Steve Harnett
% -- first write of function
% February 2024 by Steve Harnett
% -- function updated to make a right left distinction using cross products
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% -- function copied to new script from
%    fcn_algorithm_greedy_planner.m to follow library
%    conventions
% 2025_08_06 - K. Hayes
% -- updated fcn header and formatting
% -- added debug plotting capabilities
%
% TO DO:
%
% -- fill in to-do items here.

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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
        narginchk(4,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

        % Check the all_pts input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '5column_of_numbers');        

    end
end

% Does user want to specify the polytopes input?
polytopes = []; % Default is empty
if 5 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        polytopes = temp;
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

vgraph = vgraph - eye(size(vgraph,1));
% 1.  Initialize the open list: put the starting node on the open list 
%     (you can leave its f at zero)
num_nodes = size(vgraph,1); % number of nodes in the cost graph
open_set = nan(1,num_nodes);
open_set(start(3)) = start(3); % only store the ID not the whole point
open_set_gs = inf*ones(1,num_nodes); % initialize costs of open set to infinity
open_set_gs(start(3)) = 0; % g-value for nodes in open set.  g is the movement cost to
% move from the starting point to a given square on the grid, following the
% path generated to get there.
all_pts_plus_start_and_fin = [all_pts; start; finish];
% find all heuristic costs as distances from point to finish
hs = sqrt((all_pts_plus_start_and_fin(:,1) - finish(1)).^2 + (all_pts_plus_start_and_fin(:,2) - finish(2)).^2)';

%  the estimated movement cost to move from that given square on the grid to
% the final destination. This is often referred to as the heuristic
open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set.
% f is the sum of the g-value and h-value
% TODO @sjharnett f, g, and h should only depend on cost graph not including calculations

% 2.  Initialize the closed list
closed_set = nan(1,num_nodes);
%     closed_set_fs = inf*ones(1,num_nodes);
q_history = [];
q_parents = {};

% 3.  while the open list is not empty
% implies at least one nan
while (sum(isnan(open_set)) > 0)
    % a) find the node with the least f on the open list, call it "q"
    [f_of_q, idx_of_q] = min(open_set_fs);
    q = all_pts_plus_start_and_fin(idx_of_q,:);
    q_history = [q_history; q];

    % b) pop q off the open list
    open_set_fs(idx_of_q) = inf;
    % open_set_gs(idx_of_q) = inf;
    % open_set_fs = open_set_gs + hs; % f-vlaue for nodes in the open set
    open_set(idx_of_q) = NaN;

    % c) generate q's 8 successors and set their parents to q
    qs_row = vgraph(idx_of_q,:);
    successor_idxs = [];
    successor_idxs = find(qs_row);
    % TODO @sjharnett set parents to q
    q_parents{end+1} = successor_idxs;

    % d) for each successor
    for i = 1:length(successor_idxs)
        successor = all_pts_plus_start_and_fin(successor_idxs(i),:);
        % i) if successor is the goal, stop search
        if successor(3) == finish(3)
            cost = open_set_gs(idx_of_q) + hs(idx_of_q);
            route = [finish];
            parent = idx_of_q;
            route = [q; finish];
            q_history(end,:) = [];
            possible_parents = intersect(q_parents{end}, q_history(:,3));
        
            while parent ~= start(3)
                parent_gs = open_set_gs(possible_parents);
                [parent_g, idx_of_parent] = min(parent_gs);
                parent = possible_parents(idx_of_parent);
                parent_position_in_history = find(parent == q_history(:,3));
                parent_point = q_history(parent_position_in_history,:);
                route = [parent_point;route];
                q_history(parent_position_in_history:end,:) = [];
                % parent = possible_parents(idx_of_parent);
                % parent_position_in_history = find(parent == q_history(:,3));
                possible_parents = intersect(q_parents{parent_position_in_history}, q_history(:,3));
            end
            return

        else
        % ii) else, compute both g and h for successor
        successor_g = open_set_gs(idx_of_q) + sqrt((successor(1) - q(1)).^2 + ((successor(2) - q(2)).^2));
        successor_h = sqrt((successor(1) - finish(1)).^2 + ((successor(2) - finish(2)).^2));
        successor_f = successor_g + successor_h;
        open_set(successor(3)) = successor(3);
        open_set_gs(successor(3)) = successor_g;
        open_set_fs(successor(3)) = successor_f;

        end



        % iii) if a node with the same position as successor is in the OPEN list which has a
        % lower f than successor, skip this successor
        % if sum(open_set_fs < successor_f) > 0
            % continue
        % end

        % if sum(closed_set_fs < successor_f)
            % continue
        % else

        % iV) if a node with the same position as successor  is in the CLOSED list which has
        % a lower f than successor, skip this successor otherwise, add  the node to the open list
        % end (for loop)
        % end

    % e) push q on the closed list
    closed_set(idx_of_q) = idx_of_q;
    % closed_set_fs(idx_of_q) = open_set_fs(idx_of_q);
    % end (while loop)
    end

    if ~isempty(route)
        break
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
    figure(fig_num)
    hold on

    % Plot polytopes
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;
    fcn_MapGen_plotPolytopes(polytopes,plotFormat,[1 0 0 0 0.5],fig_num);

    % Plot path through field
    plot(route(:,1),route(:,2),'k-','linewidth',2)
    plot(start(1), start(2), 'gx','linewidth',2)
    plot(finish(1), finish(2), 'rx','linewidth',2)

    % Plot neighboring points
    plot(appex_x,appex_y,'o','linewidth',2)
end

        
end
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