function [visibility_matrix, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes, varargin)
% fcn_Visibility_clearAndBlockedPointsGlobal
%
% returns an intersection matrix for a single start point, showing what was 
% intersected between that start point and numerous possible end points.
% This function wraps that function to call it on every possible start and end
% combination to provide global visibility truth tables rather than local
% intersection truth tables.
%
% FORMAT:
% [visibility_matrix, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes, (isConcave), (fig_num))
%
% INPUTS:
%
%     polytopes: a 1-by-p seven field structure of polytopes, where
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
%%%%%%%%%% delete
%     starts: p-by-5 matrix of all the possible start points
%       the information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id numberf
%         obstacle id number
%         beginning/ending indication (1 if the point is a beginning or ending
%         point and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%
%      finishes: p-by-5 matrix of all the possible finish points
%       the information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id numberf
%         obstacle id number
%         beginning/ending indication (1 if the point is a beginning or ending
%         point and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%
%      gap_size: if zero, the special fully tiled case will be handled.
%         This involves assuming that visibility is only down sides and through polytopes
%%%%%%%%%%
%
%     (optional inputs)
%
%      isConcave: set a 1 to allow for concave (i.e. non-convex) obstacles.  If this is left
%         blank or set to anyting other than 1, the function defaults to the convex behavior
%         which is more conservative (i.e. setting the flag wrong incorrectly may result in
%         suboptimal paths but not collisions). For background on what this flag does, see slides
%         in `concave_vgraph` section of Documentation/bounded_astar_documentation.pptx
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%         input checking or debugging, no figures will be generated, and sets
%         up code to maximize speed. As well, if given, this forces the
%         variable types to be displayed as output and as well makes the input
%         check process verbose
%
%
% OUTPUTS:
%
%     visibility_matrix: nxn matrix, where n is the number of points in all_pts
%       a 1 in column i and row j indicates that all_pts(i,:) is visible from
%       all_pts(j,:).  This matrix is therefore symmetric
%
%     visibility_results: 
%
% DEPENDENCIES:
% 
% fcn_DebugTools_checkInputsToFunctions
% fcn_Visibility_clearAndBlockedPoints
%
% EXAMPLES:
%
% See the script: script_test_fcn_Visibility_clearAndBlockedPointsGlobal.m
% for a full test suite.
%
% Questions or comments? contact sjh6473@psu.edu

% REVISION HISTORY:
% 2021_10_28
% -- first written by Steve Harnett
% Questions? sjh6473@psu.edu
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from
%    fcn_visibility_clear_and_blocked_points_global to follow library
%    convention
% 2025_07_31 - K. Hayes
% -- reformatted function
% -- added input checking and debug

% TO DO:
% (copied from Steve's notes)
% -- could use the Lee algorithm to speed up if necessary
%    https://github.com/davetcoleman/visibility_graph/blob/master/Visibility_Graph_Algorithm.pdf
% -- could also discard sides based on direction of normal relative to scan direction
%    there is an issue: polytopes contain points, therefore points are represented multiple times
%    need the reverse mappping of points to polytopes
%    then each point is only represented once
%    visibility graph can then be reduced
% -- fix isConcave flag input checking

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
        narginchk(3,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     finish, '5column_of_numbers');

    end
end


% Does user want to specify the isConcave input?
isConcave = 0; % Default is 0
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        isConcave = temp;
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


num_points = size(starts,1);
% for non-zero gap size, we can repeatedly call the legacy visibility functions

% if gap_size ~= 0
    visibility_matrix = NaN(num_points);
    %% loop through all points
    for j = 1:num_points
        i = starts(j,3);
        % legacy visibility function returns visibility vector for this point
        [visibility_results(i).clear_pts,visibility_results(i).blocked_pts,visibility_results(i).D,visibility_results(i).di,visibility_results(i).dj,visibility_results(i).num_int,visibility_results(i).xiP,visibility_results(i).yiP,visibility_results(i).xiQ,visibility_results(i).yiQ,visibility_results(i).xjP,visibility_results(i).yjP,visibility_results(i).xjQ,visibility_results(i).yjQ] = ...
            fcn_Visibility_clearAndBlockedPoints(polytopes,starts(j,:),finishes,isConcave,-1);
        % D is finish points on the rows and polytope sides on the columns
        % transpose this so we have column for each point
        % sum each column into a row vector so each element is the sum of number
        % of sides hit
        % if sum>0, this implies there is no visibility
        visibility_matrix(i,:) = sum(visibility_results(i).D')==0;
        % sometimes the diagonal does not contain only 1's (it always should
        % since every point is visible to itself so we overwrite this)
        visibility_matrix(i,i) = 1;

        % %% add self-blocked points
        % % points across the polytope are also visible so find self blocked pts
        % % find obs_id for cur_pt
        % cur_obs_id = all_pts(i,4);
        % % find pt ids of every point on this obstacle
        % pt_idx = find(all_pts(:,4)==cur_obs_id);
        % % at row i, and columns with values in pt_idx...
        % idx = sub2ind(size(visibility_matrix), i.*ones(size(pt_idx,1),size(pt_idx,2)), pt_idx);
        % % set a 1, indicating the self-blocked points are visible
        % visibility_matrix(idx) = 1;
    end
% elseif gap_size == 0
%     % for the zero gap size case, we can do an optimization: all points on the same polytope
%     % are visible, either along the side or across the polytope
%     % other points are not visible since there are no gaps and angles of 180 deg
%     % are not possible in a Voronoi diagram where all vertices have 3 Voronoi sides
%     deduped_pts = fcn_BoundedAStar_convertPolytopetoDedupedPoints(all_pts);
%     num_unique_pts = length(deduped_pts);
%     all_polys = NaN(num_unique_pts,3);
%     for i = 1:num_unique_pts
%         for j = 1:length(deduped_pts(i).polys)
%             all_polys(i,j) = deduped_pts(i).polys(j);
%         end
%     end
%     visibility_matrix = zeros(num_unique_pts);
%     for i = 1:num_unique_pts
%         for j = 1:3
%             poly_of_interest = all_polys(i,j);
%             if isnan(poly_of_interest)
%                 continue
%             end
%             [r,c] = find(all_polys == poly_of_interest);
%             for k = 1:length(r)
%                 visibility_matrix(i,r(k)) = 1;
%             end
%         end
%    end
% end
if isConcave
    %% check for edges entirely contained by polytopes
    % don't need to check self visible points as this will not change
    visibility_matrix_without_self_visible = visibility_matrix - eye(size(visibility_matrix,1));
    % find indeces of every other '1' or allowed edge...
    linear_idx = find(visibility_matrix_without_self_visible); % find 1s in visibility_matrix
    [rows_of_1s, cols_of_1s] = ind2sub(size(visibility_matrix_without_self_visible),linear_idx); % convert linear idx to r,c
    num_1s = length(rows_of_1s);
    for e = 1:num_1s
        start_pt = starts(rows_of_1s(e),1:2);
        end_pt = finishes(cols_of_1s(e),1:2);
        % parametric equation for line in 3D: https://math.stackexchange.com/questions/404440/what-is-the-equation-for-a-3d-line
        % [x y z]' = [a b c]'*t + [x0 y0 z0]'
        % parametric equation for line in 2D: https://math.libretexts.org/Bookshelves/Calculus/CLP-3_Multivariable_Calculus_(Feldman_Rechnitzer_and_Yeager)/01%3A_Vectors_and_Geometry_in_Two_and_Three_Dimensions/1.03%3A_Equations_of_Lines_in_2d
        % [x y]' = d'*t + [x0 y0]'
        d_vec = end_pt - start_pt;
        mid_pt = start_pt + 0.5*d_vec; % find the middle of the edge
        % for each polytope...
        num_polys = length(polytopes);
        p = 1;
        while p <= num_polys
            verts = polytopes(p).vertices;
            % get xmin and xmax also ymin and ymax
            xmax = max(verts(:,1));
            xmin = min(verts(:,1));
            ymax = max(verts(:,2));
            ymin = min(verts(:,2));
            % check axis aligned bounding box before checking for polytope containment of the midpoint
            in_AABB = (mid_pt(1) < xmax && mid_pt(1) > xmin) && (mid_pt(2) < ymax && mid_pt(2) > ymin);
            % is point between xmin xmax and ymin max? if not continue
            if ~in_AABB
                p = p+1;
                continue
            end
            % if point is in AABB make polyshape from these verts
            polyshape_p = polyshape(verts);
            % is point in but not on polyshape?
            [is_in,is_on] = isinterior(polyshape_p,mid_pt(1:2));
            % if so, remove the edge, and stop trying polytopes
            if is_in && ~ is_on
                visibility_matrix(rows_of_1s(e),cols_of_1s(e)) = 0;
                % if it is in one polytope, we needn't check any others
                p = num_polys+1;
            end
            % if not, continue to check the next polytope
            p = p + 1;
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
    figure(fig_num)
    
    % Set up valid edges subplot
    subplot(3,1,1)
    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fillFormat = [];
    fcn_MapGen_plotPolytopes(polytopes, (plotFormat), (fillFormat), (fig_num));
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('valid edges')

    % Set up blocked edges subplot
    subplot(3,1,2)
    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fcn_MapGen_plotPolytopes(polytopes, (plotFormat), (fillFormat), (fig_num));
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('blocked edges')

    % Set up all edges subplot
    subplot(3,1,3)
    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fcn_MapGen_plotPolytopes(polytopes, (plotFormat), (fillFormat), (fig_num));
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
    title('all edges')

    % Plot visibility graph edges
    for i = 1:size(visibility_matrix,1)
        for j = 1:size(visibility_matrix,1)
            if visibility_matrix(i,j) == 1
                subplot(3,1,1)
                plot([starts(i,1),finishes(j,1)],[starts(i,2),finishes(j,2)],'--g','LineWidth',2)
                subplot(3,1,3)
                plot([starts(i,1),finishes(j,1)],[starts(i,2),finishes(j,2)],'--g','LineWidth',2)
            end
            if visibility_matrix(i,j) == 0
                subplot(3,1,2)
                plot([starts(i,1),finishes(j,1)],[starts(i,2),finishes(j,2)],'--r','LineWidth',2)
                subplot(3,1,3)
                plot([starts(i,1),finishes(j,1)],[starts(i,2),finishes(j,2)],'--r','LineWidth',2)
            end
        end
    end

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
