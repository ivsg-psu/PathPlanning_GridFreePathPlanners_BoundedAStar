function [cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = ...
    fcn_BoundedAStar_straightPlanner(start,finish,all_pts,polytopes,varargin)
% fcn_BoundedAStar_straightPlanner
%
% plans a path straight through the obstacle field, traversing all encountered obstacles
%
% FORMAT:
%
% [cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = ...
% fcn_BoundedAStar_straightPlanner(start,finish,all_pts,polytopes,(fig_num))
%
% INPUTS:
%
%       start: 1-by-5 vector with the same info as route for the starting point
%       
%       finish: same as start for the finish point
% 
%       all_pts: p-by-5 matrix of all the points except start and finish
% 
%       polytopes: a 1-by-n seven field structure of shrunken polytopes,
%       where n <= number of polytopes with fields:
%           vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%           the number of the individual polytope vertices
%           xv: a 1-by-m vector of vertice x-coordinates
%           yv: a 1-by-m vector of vertice y-coordinates
%           distances: a 1-by-m vector of perimeter distances from one point to the
%           next point, distances(i) = distance from vertices(i) to vertices(i+1)
%           mean: centroid xy coordinate of the polytope
%           area: area of the polytope
%
%       fig_num: a figure number to plot results. If set to -1, skips any
%           input checking or debugging, no figures will be generated, and sets
%           up code to maximize speed. As well, if given, this forces the
%           variable types to be displayed as output and as well makes the input
%           check process verbose
%
% OUTPUTS:
%
%       cost: the cost to execute this path, scaled up by polytope traversal costs for distance
%       spent in polytopes
% 
%       distance_in_polys: the distance of the straight path inside polytope boundaries
% 
%       distance_outside_polys: the distance of the straight path outside polytope boundaries, in free space
% 
%       num_polys_traversed: the integer number of polytopes encountered and traversed
%
% DEPENDENCIES:
%   fcn_BoundedAStar_polytopesNearLine
%   fcn_BoundedAStar_plotPolytopes
%   fcn_geometry_findIntersectionOfSegments
%
% EXAMPLES:
%
% For additional examples, see: script_planning_performed_at_multiple_costs.m
%
% This function was written in 2022_05 by Steve Harentt
% Questions or comments? sjh6473@psu.edu
%

% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
% 2025_07_17 - K. Hayes
% -- copied to new function from fcn_algorithm_straight_planner to
%    follow library conventions
% 2025_08_05 - K. Hayes
% -- updated fcn header and formatting
% -- added debug plotting capabilities to fcn

% TO DO
% (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 5; % The largest Number of argument inputs to the function
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
        
        % Check the all_pts input, make sure it has 5 columns
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

% initialize variables
cost = 0;
distance_in_polys = 0;
distance_outside_polys = 0;
num_polys_traversed = 0;
a_b = sum((start(1:2) - finish(1:2)).^2,2).^0.5;
% determine which polys are near the straight path
close_polytopes = fcn_BoundedAStar_polytopesNearLine(start,finish,polytopes);

% obtain equation for line from a (start) to b (finish)
% for each poly:
%     if poly has at least one vertex above and at least one below the equation for a-b:
%         note poly as straddling the line
% for each of those polytopes...
for i = 1:length(close_polytopes)
    % determine if the straight path crosses the side
    wall_starts_xy = close_polytopes(i).vertices(1:end-1,1:2);
    wall_ends_xy = close_polytopes(i).vertices(2:end,1:2);
    % search using option 2 of findIntersectionOfSegments to find all intersections
    [dists_to_crossings,locations_of_crossings,~] = ...
        fcn_geometry_findIntersectionOfSegments(wall_starts_xy,wall_ends_xy,...
        start(1:2),finish(1:2),2);
    % there should always be 0 or 2 crossings
    assert(size(locations_of_crossings,1) == 2 || size(locations_of_crossings,1) == 0);
    % note the distance between intersection points

    if size(locations_of_crossings,1) == 2
        % figure(111111); hold on;
        % plot(locations_of_crossings(:,1),locations_of_crossings(:,2),"cx");
        distance_through_poly = sum((locations_of_crossings(1,:) - locations_of_crossings(2,:)).^2,2).^0.5;
        % increment traversed polytope counter
        num_polys_traversed = num_polys_traversed + 1;
    elseif size(locations_of_crossings,1) == 0
        distance_through_poly = 0;
    end

    % add to total distance_in_polys
    distance_in_polys = distance_in_polys + distance_through_poly;
    % scale by poly.cost, add to total_cost
    cost_through_poly = (1+close_polytopes(i).cost)*distance_through_poly;
    cost = cost + cost_through_poly;
end
% subtract distance_in_polys from dist between a-b % this is outside of polytope distance
distance_outside_polys = a_b - distance_in_polys;
% add unscaled outside distance to total_cost
cost = cost + distance_outside_polys;

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
    figure(fig_num);
    
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 3;
    hold on;
    box on;
    fcn_MapGen_plotPolytopes(polytopes,plotFormat,[1 0 0 0 0.5],fig_num);
   
    plotFormat.Color = 'red';
    plotFormat.LineWidth = 2;
    hold on;
    box on;
    fcn_MapGen_plotPolytopes(close_polytopes,plotFormat,[1 0 0 0 0.5],fig_num);
    plot(linspace(start(1),finish(1),2),linspace(start(2),finish(2),2),'LineWidth',2,'Color','green')
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
