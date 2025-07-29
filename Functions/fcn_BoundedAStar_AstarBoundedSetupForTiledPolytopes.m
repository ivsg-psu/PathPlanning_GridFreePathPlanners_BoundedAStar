function [path,cost,err] = fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes(polytopes, start, finish, planner_mode, varargin)
% fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes 
% 
% sets up the information needed for the Dijkstra
% Astar hybrid function and calls the function when the input polytopes are
% generated from the voronoi diagram
%
% FORMAT:
%
% [path,cost,err] = fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes(polytopes, start, finish, planner_mode, (bounds), (fig_num))
%
% INPUTS:
%
%     polytopes: 
%
%     start: the start point vector (x, y, id, obs_id, beg_end)
%
%     finish: the finish point matrix of all valid finishes where each row
%     is a single finish point vector (x, y, id, obs_id, beg_end)
%
%     planner_mode: string containing option for planner behavior that indicates the planner mode
%       "legacy" only goes around obstacles
%       "through at vertices" allows the planner to go through or around each obstacle
%           but only entering or exiting at vertices
%       "through or around" allows the planner to go through all obstacles or around all
%       "straight through" the planner only goes straight from the start to the goal, calculating the cost
%     
%     (optional inputs)
%
%     bounds: b-by-2 matrix of xy coordinates of the boundaries the path
%       planner must stay within, where b is the number of boundary points and
%       b>=3. If this argument not specified, there are no bounds.
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
% OUTPUTS:
%
%    path: the mx5 matrix as produced by consisting of waypoints.  Each row is a
%       waypoint, and each column is [x-coordinate, y-coordinate, point id, obstacle id,
%       beginning or end of an obstacle set (1 or 0)]
%
%    cost: the total cost of the selected route
%    
%    err: error flag for reporting errors without stopping the code
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script:
% script_test_fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes
% for a full test suite.
%
% This function was written on 2019_06_13 by Seth Tau
% Questions or comments? sat5340@psu.edu
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
% 2025_07_17 - K. Hayes
% -- Copied function from
%    fcn_algorithm_setup_bound_Astar_for_tiled_polytopes.m to follow library
%    conventions
% 2025_07_25 - K. Hayes
% -- fixed function formatting
% -- added input and debug checks
% 2025_07_29 - S. Brennan
% -- added plotting as part of standard debug output
% -- fixed incorrect argument list

% TO DO:
% -- input checking and description in header for polytopes

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

        % % Check the start input, make sure it has 2 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     start, '2column_of_numbers');
        % 
        % % Check the finish input, make sure it has 2 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     finish, '2column_of_numbers');
    end
end

% Does user want to specify the bounds input?
bounds = []; % Default is 1
if 5 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        bounds = temp;
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

% check if the start or end are now within combined polytopes
throw_error = 0; % only gives soft errors errors that don't stop the code
check_edge = 1; % checks for start or finish on polytope edges
[err startPoly, finishPoly] = fcn_BoundedAStar_polytopePointsInPolytopes(start,finish,polytopes,throw_error,check_edge); err = 0; %#ok<NCOMMA,ASGLU> % check that start and end are outside of obstacles

if err == 0 % start and finish outside the polytopes
    % add points for start and finish if on an edge
    if startPoly ~= -1 % on obstacle edge
        vertices = polytopes(startPoly).vertices;
        start_gap = fcn_BoundedAStar_polytopePointGapLocation([start.x start.y],vertices(:,1:2));
        new_verts = [vertices(1:start_gap,:); start.x start.y; vertices(start_gap+1:end,:)];
        polytopes(startPoly).vertices = new_verts;
        polytopes(startPoly).xv = new_verts(1:end-1,1)';
        polytopes(startPoly).yv = new_verts(1:end-1,2)';
        polytopes(startPoly).distances = sum((new_verts(1:end-1,:) - new_verts(2:end,:)).^2,2).^0.5;
    end

    if finishPoly ~= 0 % on obstacle edge
        vertices = polytopes(finishPoly).vertices;
        finish_gap = fcn_BoundedAStar_polytopePointGapLocation([finish.x finish.y],vertices(:,1:2));
        new_verts = [vertices(1:finish_gap,:); finish.x finish.y; vertices(finish_gap+1:end,:)];
        polytopes(finishPoly).vertices = new_verts;
        polytopes(finishPoly).xv = new_verts(1:end-1,1)';
        polytopes(finishPoly).yv = new_verts(1:end-1,2)';
        polytopes(finishPoly).distances = sum((new_verts(1:end-1,:) - new_verts(2:end,:)).^2,2).^0.5;
    end

    point_tot = length([polytopes.xv]); % total number of vertices in the convex polytopes

    % information about each point
    beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    for poly = 1:size(polytopes,2) % check each polytope
        verts = unique(polytopes(poly).vertices,'stable','rows');
        num_verts = size(verts,1);
        polytopes(poly).obs_id = ones(1,num_verts)*poly; % obs_id is the same for every vertex on a single polytope
        polytopes(poly).xv = verts(:,1)';
        polytopes(poly).yv = verts(:,2)';
        polytopes(poly).vertices = [verts; verts(1,:)];
        polytopes(poly).distances = sum((polytopes(poly).vertices(1:end-1,:) - polytopes(poly).vertices(2:end,:)).^2,2).^0.5;
        beg_end([curpt+1,curpt+num_verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
        curpt = curpt+num_verts;
        polytopes(poly).perimeter = sum(polytopes(poly).distances);
    end

    obs_id = [polytopes.obs_id];
    point_tot = length([polytopes.xv]); % need to recheck total points
    beg_end = beg_end(1:point_tot); % remove any extra points
    
    % Create all_pts for future call to path planning function
    all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]

    % give the same information to the starting and ending points
    if startPoly ~= -1 % on obstacle edge
        % find adjacent points
        adj_pts = all_pts(all_pts(:,4)==startPoly,:);

        start_id = adj_pts(start_gap+1,3);
        start_beg_end = all_pts(start_id,5);
    else
        start_id = point_tot+1;
        start_beg_end = 0;
    end
    if finishPoly ~= 0 % on obstacle edge
        % find adjacent points
        adj_pts = all_pts(all_pts(:,4)==finishPoly);

        finish_id = adj_pts(finish_gap+1,3);
        finish_beg_end = all_pts(finish_id,5);
    else
        finish_id = point_tot+2;
        finish_beg_end = 0;
    end

    % Set up start and finish points with IDs for planner processing
    start = [start.x start.y start_id startPoly start_beg_end];
    finish = [finish.x finish.y finish_id finishPoly finish_beg_end];
    
    % Impose bounds, if they exist
    if ~isempty(bounds)
        bound_pts = all_pts(inpolygon(all_pts(:,1),all_pts(:,2),bounds(:,1),bounds(:,2)),:); % bound points at the start
    else
        bound_pts = all_pts;
    end

    % find valid points
    [~,ia,ic] = unique(bound_pts(:,1:2),'rows','stable');
    h = accumarray(ic, 1);
    valid_pts = bound_pts(ia(h==1),:);

    % Create polytopes for finding ellipses %%%%%%%% needs debugging for 3+
    % verts in a straight line
    ellipse_polytopes = polytopes;
    
    % calculate path
    [cost,path] = fcn_BoundedAStar_AstarBounded(start,finish,polytopes,all_pts,valid_pts,planner_mode,ellipse_polytopes, (-1));

else % start or finish are in the polytopes
   path = [];
   cost = [];
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

    figure(fig_num);
    hold on

    allPointsPlotted = all_pts(:,1:2);
    minX = min(allPointsPlotted(:,1));
    minY = min(allPointsPlotted(:,2));
    maxX = max(allPointsPlotted(:,1));
    maxY = max(allPointsPlotted(:,2));

    scaleX = maxX - minX;
    scaleY = maxY - minY;
    percentLarger = 0.3;
    new_axis = [minX-scaleX*percentLarger maxX+scaleX*percentLarger minY-scaleY*percentLarger maxY+scaleY*percentLarger];
    axis(new_axis);


    % Plot results
    plotFormat.LineWidth = 3;
    plotFormat.MarkerSize = 10;
    plotFormat.LineStyle = '-';
    plotFormat.Color = [0 0 1];

    % fillFormat = [1 0 0 0 0.5];
    fillFormat = [];
    h_plot = fcn_MapGen_plotPolytopes(polytopes, (plotFormat),(fillFormat),(fig_num));
    set(h_plot,'DisplayName','Input: polytopes');
    plot([start(1) finish(1)],[start(2) finish(2)],'kx','LineWidth',2,'DisplayName','Input: start and end pts')
    plot(path(:,1),path(:,2),'k-','linewidth',2,'DisplayName','Output: route');
    box on
    xlabel('X Position')
    ylabel('Y Position')

    legend('Interpreter','none','Location','best');

end % Ends the flag_do_plot if statement



if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends main function



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