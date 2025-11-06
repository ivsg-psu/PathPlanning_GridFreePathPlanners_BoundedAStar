function new_vgraph = fcn_Visibility_3dGraphAddPoints(old_verts, start, finish, all_surfels, speed_limit, new_pts, old_vgraph, varargin)
% fcn_Visibility_3dGraphAddPoints
%
% Forms the 3D visibility graph, the edges of which connect nodes that are connected by straight,
% collision-free path segments. This leverages the Moller-Trumbore algorithm to check potential
% graph edges for intersections with the 3D obstacles. The visibility graph can also take in a speed
%  limit, which in the case of XYT rather than XYZ is useful for pruning edges that would go backwards
% in time or traverse too much distance in too short of a time
%
% FORMAT:
% new_vgraph = fcn_Visibility_3dGraphAddPoints(verts, start, finish, all_surfels, speed_limit, new_pts, old_vgraph, (fig_num))
%
% INPUTS:
%
%    verts: matrix of all obstacle vertices in the polytope field.  Each row should be a point, and each column is x, y, and z or T
%
%    start: the start point vector (x,y,t)
%
%    finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,t)
%
%     all_surfels: a matrix with all triangular surface elements (surfels) from all timespace polytopes
%     there is one row for each surfel
%     each row has 9 columns representing the x,y,t coordinates of each point of the triangle ordered
%     x1 y1 t1 x2 y2 t2 x3 y3 t3
%
%    speed_limit: a double representing the speed limit for timespace.  Visibilty graph edges
%       violating this speed limit in distance (x y combined) over time (z or t axis) will be discarded
%       running without a speed limit creates an XYZ visibility graph rather than XYT where traversal is possible
%       backwards in time i.e. decreasing in z
%
%    new_pts:
%
%    old_vgraph:
%
%    (optional inputs)
%
%    fig_num: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose
%
%
% OUTPUTS:
%
%   new_vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
% 
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% Also, surfels can be created from polytopes using fcn_BoundedAStar_makeTriangularSurfelsFromFacets and
% vertices can be interpolated in t using fcn_BoundedAStar_interpolatePolytopesInTime
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a full test suite.
%
% This function was written on summer 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, summer by Steve Harnett
% -- first write of function
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_visibility_graph_add_points to follow
%    library convention
% 2025_07_31 - K. Hayes
% -- updated function formatting and header
% -- added input and debug capabilities
% 2025_08_05 - K. Hayes
% -- moved plotting into function debug section
%
% TO DO:
%
% -- figure out what's going on with adding points vs adding polytopes
% -- update header

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 8; % The largest Number of argument inputs to the function
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

        % Check the start input, make sure it has 3 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '3column_of_numbers');

        % Check the finish input, make sure it has 3 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '3column_of_numbers');

        % Check the all_surfels input, make sure it has 9 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_surfels, '9column_of_numbers');

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

    all_pts_old = [old_verts; start; finish];
    num_pts_old = size(all_pts_old,1); % number of rows
    all_pts_old_idx = 1:1:num_pts_old; % array of all possible pt idx
    all_pts_old = [all_pts_old all_pts_old_idx']; % add pt ID column to all_pts

    num_new_pts = size(new_pts,1);
    new_pts_idx = (num_pts_old+1):1:(num_pts_old+num_new_pts); % array of all possible pt idx
    new_pts = [new_pts new_pts_idx']; % add pt ID column to all_pts

    all_pts = [all_pts_old; new_pts];

    new_pts_to_old_pts = table2array(combinations(new_pts_idx, all_pts_old_idx)); % add new rays starting at new pts and ending at old pts

    % need to form all possible rays starting at one point and ending at another
    new_ray_starts = all_pts(new_pts_to_old_pts(:,1),:); % take all cols of all_pts at the row provided by the first col of all combos
    new_ray_ends = all_pts(new_pts_to_old_pts(:,2),:); % take all cols of all_pts at the row provided by the second col of all combos
    new_ray_dirs = new_ray_ends - new_ray_starts; % TriangleRayIntersection takes a ray direction which is end minus beginning
    num_rays = size(new_ray_starts,1);

    % need to form all possible combinations of a ray to check and a surfel it may collide with
    num_surfels = size(all_surfels,1);
    new_ray_idx = 1:1:num_rays;
    all_surfel_idx = 1:1:num_surfels;
    all_surfel_ray_combos = table2array(combinations(new_ray_idx,all_surfel_idx));
    % this means each ray and each surfel will appear more than once
    new_ray_starts_repeated = new_ray_starts(all_surfel_ray_combos(:,1),:);
    new_ray_ends_repeated = new_ray_ends(all_surfel_ray_combos(:,1),:);
    new_ray_dirs_repeated = new_ray_dirs(all_surfel_ray_combos(:,1),:);
    all_surfels_repeated = all_surfels(all_surfel_ray_combos(:,2),:);
    % now we can do the vectorized call to TriangleRayIntersection to check all rays against each surfel
    [intersects, ts, us, vs, xcoors] = TriangleRayIntersection (new_ray_starts_repeated(:,1:3), new_ray_dirs_repeated(:,1:3), all_surfels_repeated(:,1:3),all_surfels_repeated(:,4:6),all_surfels_repeated(:,7:9),'lineType','segment','border','normal');

    new_rows = ones(num_new_pts,num_pts_old); % initialize vgraph as ones, remove edges when intersection occurs
    intersects_idx = find(intersects);
    for k = 1:1:length(intersects_idx)
        i = intersects_idx(k);
        % if the intersection occured at a node that implies that the end of the ray
        % touched a plane segment, rather than the ray passing through the plane
        intersect_x = xcoors(i,1);
        intersect_y = xcoors(i,2);
        intersect_t = xcoors(i,3);
        verts_x = all_pts(:,1);
        verts_y = all_pts(:,2);
        verts_t = all_pts(:,3);
        % thus if one intersection location minus vertex location is approximately zero, the intersection is at a vertex and should not count
        diff_intersect_and_verts = abs([verts_x - intersect_x, verts_y - intersect_y, verts_t - intersect_t]);
        total_diffs = sum(diff_intersect_and_verts,2);
        small_diffs_bool = total_diffs < 10e-14; % this is just a tolerance based on the precision of MATLAB.  Numbers less than 10e-14 are effectively 0 as matlab cannot tell the difference between them and zero.
        % if an intersection occurred and was not at a vertex, we want to set the ray start to ray end and ray end to ray start as 0 in the vgraph (invalid for traversal)
        if sum(small_diffs_bool) == 0
            % plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'Color',[1 0 0],'LineWidth',1)
            % plot3(rmmissing(xcoors(i,1)),rmmissing(xcoors(i,2)),rmmissing(xcoors(i,3)),'cx','MarkerSize',10)
            start_id = new_ray_starts_repeated(i,4);
            end_id = new_ray_ends_repeated(i,4);
            new_rows((start_id-num_pts_old),end_id) = 0;
        end
    end

    vgraph_path_points_only = fcn_Visibility_3dGraphGlobal(new_pts(:,1:3), [], [], all_surfels, speed_limit, [], 5); % need to check new path points against themselves

    % need something like D = [[A; B], [B'; C]]
    new_vgraph = [[old_vgraph; new_rows], [new_rows'; vgraph_path_points_only]];

    %% discard rays too high in velocity using all pts array
    all_delta_ts = (-all_pts(:,3) + (all_pts(:,3))');
    % run is change in total length regardless of x or y
    all_delta_xs = (all_pts(:,1) - (all_pts(:,1))');
    all_delta_ys = (all_pts(:,2) - (all_pts(:,2))');
    all_delta_dist = (all_delta_xs.^2 + all_delta_ys.^2).^0.5;
    all_slopes = all_delta_ts./all_delta_dist; % slope above the horizontal plane is time/dist or 1/speed

    speed_violation_idx = find(all_slopes <= 1/speed_limit ); % find where slope (1/speed) violates the speed limit
    for l = 1:1:length(speed_violation_idx)
        i = speed_violation_idx(l);
        % remove rays that violate the speed limit.  notice this is directional because if beg to term violates speed limit, term to beg may not
        [beg,term] = ind2sub(size(all_slopes),i);
        % plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],'k','LineWidth',2)
        start_id = all_pts(beg,4);
        end_id = all_pts(term,4);
        new_vgraph(start_id,end_id) = 0;
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

    % Plot previous version of visibility graph
    subplot(1,2,1)
    hold on
    % Plot timespace polytopes
    for i = 1:size(all_surfels,1)
        fill3([all_surfels(i,1) all_surfels(i,4) all_surfels(i,7)], [all_surfels(i,2) all_surfels(i,5) all_surfels(i,8)], [all_surfels(i,3) all_surfels(i,6) all_surfels(i,9)],rand(1,3),'FaceAlpha',0.3);
    end
    
    % Plot original 3d vgraph
    for i = 1:size(old_vgraph,1)
        for j = 1:size(old_vgraph,1)
            if old_vgraph(i,j) == 1
                plot3([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],[all_pts(i,3),all_pts(j,3)],'-g')
            end
        end
    end
    view(3)
    INTERNAL_fcn_format_timespace_plot();
    title('Original vgraph')

    % Plot new version with new points
    subplot(1,2,2)
    % Plot timespace polytopes
    hold on
    for i = 1:size(all_surfels,1)
        fill3([all_surfels(i,1) all_surfels(i,4) all_surfels(i,7)], [all_surfels(i,2) all_surfels(i,5) all_surfels(i,8)], [all_surfels(i,3) all_surfels(i,6) all_surfels(i,9)],rand(1,3),'FaceAlpha',0.3);
    end
    
    % Plot new 3d vgraph
    all_pts_new = [all_pts; new_pts];
    for i = 1:size(new_vgraph,1)
        for j = 1:size(new_vgraph,1)
            if new_vgraph(i,j) == 1
                plot3([all_pts_new(i,1),all_pts_new(j,1)],[all_pts_new(i,2),all_pts_new(j,2)],[all_pts_new(i,3),all_pts_new(j,3)],'-g')
            end
        end
    end
    view(3)
    INTERNAL_fcn_format_timespace_plot();
    title('New vgraph')
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

function INTERNAL_fcn_format_timespace_plot()
% define figure properties
% opts.width      = 8.8;
% opts.height     = 6;
% opts.fontType   = 'Times New Roman';
% opts.fontSize   = 14;
% fig = gcf;
% % scaling
% fig.Units               = 'centimeters';
% fig.Position(3)         = opts.width;
% fig.Position(4)         = opts.height;

% % set text properties
% set(fig.Children, ...
%     'FontName',     'Times New Roman', ...
%     'FontSize',     14);

% remove unnecessary white space
set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
xlabel('x [km]')
ylabel('y [km]')
zlabel('t [min]')
view([36 30])
end
