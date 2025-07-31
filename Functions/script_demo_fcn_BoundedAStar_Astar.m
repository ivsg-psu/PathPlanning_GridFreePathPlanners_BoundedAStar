% script_demo_fcn_BoundedAStar_Astar.m
% a basic test of a 2D path planning scenario 
% Generate an obstacle field and calculate an A-star path through it

% Revision history
% 2025_07_07 - S. Brennan, sbrennan@psu.edu and K. Hayes, kxh1031@psu.edu
% -- created code from script_test_fcn_algorithm_Astar.m written by S. Harnett
% 2025_07_26 - S. Brennan
% -- merging updates of MapGen into this script. Not quite done as found
%    % bugs in MapGen that should be fixed for consistency
% 2025_07_28 - S. Brennan
% -- merged updates of MapGen into this script. 

clear
clc
close all

%% Code demos start here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____                              ____   __    _____          _
%  |  __ \                            / __ \ / _|  / ____|        | |
%  | |  | | ___ _ __ ___   ___  ___  | |  | | |_  | |     ___   __| | ___
%  | |  | |/ _ \ '_ ` _ \ / _ \/ __| | |  | |  _| | |    / _ \ / _` |/ _ \
%  | |__| |  __/ | | | | | (_) \__ \ | |__| | |   | |___| (_) | (_| |  __/
%  |_____/ \___|_| |_| |_|\___/|___/  \____/|_|    \_____\___/ \__,_|\___|
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Demos%20Of%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate an obstacle field and calculate an A-star path through it

%% map generation control
% repetition controls and storage
repetitions = 10;
% final information can be stored in one variable (not suggested to save
% variables of the structure type, as it will take a lot of memory)
final_info(repetitions) = struct('polytopes',[],'start',[],'finish',[],'path_x',[],'path_y',[],'appex1_x',[],'appex1_y',[],'appex2_x',[],'appex2_y',[]);

% generate Voronoi tiling from Halton points
pt_density = 100; % point density used for generation
low_pts = 1:pt_density:(pt_density*(repetitions-1)+1); % lower bound of Halton set range
high_pts = pt_density:pt_density:pt_density*repetitions; % upper bound of Halton set range

% remove the edge polytope that extend past the high and low points
xlow = 0; xhigh = 1; ylow = 0; yhigh = 1;

% shink the polytopes so that they are no longer tiled
des_radius = 0.05; % desired average maximum radius
sigma_radius = 0.002; % desired standard deviation in maximum radii
min_rad = 0.0001; % minimum possible maximum radius for any obstacle
shrink_seed = 1111; % seed used for randomizing the shrinking process
des_cost = 0; % polytope traversal cost

% starting (A) and finish (B) coordinates
A.x = 0.0; A.y = 0.5; B.x = 1; B.y = 0.5;
startPoint = [A.x A.y];
endPoint   = [B.x B.y];

% uncomment below to start inside a polytope
% A.x = 0.15; A.y = 0.54; B.x = 1; B.y = 0.5;

% plotting control
flag_do_plot = 1; % 1 if you would like to see plots, anything else if not
fig_num = 99;

%% Start of repetitions

for rep = 1:repetitions
    %%%%%%%%
    % generate a new map
    % generate Voronoi tiling from Halton points
    low_pt = low_pts(rep); high_pt = high_pts(rep)-50; % range of Halton points to use to generate the tiling
    tiled_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [low_pt high_pt],[],[],-1);
    
    % remove the edge polytope that extend past the high and low points
    trim_polytopes  = fcn_MapGen_polytopesDeleteByAABB( tiled_polytopes, [0.001 0.001, 0.999 0.999], (-1));
    
    % shink the polytopes so that they are no longer tiled
    rng(shrink_seed) % set the random number generator with the shrink seed
    shrunk_polytopes = fcn_MapGen_polytopesShrinkToRadius(trim_polytopes,des_radius,sigma_radius,min_rad, (fig_num));

    des_cost = 0.1;
    shrunk_polytopes = fcn_MapGen_polytopesSetCosts(shrunk_polytopes, des_cost, (-1));


    %%%%%
    % Calculate info needed for visibility graph generation
    % gather data on all the points
    point_tot = length([shrunk_polytopes.xv]); % total number of vertices in the polytopes
    flag_thisIsABeginningEnd = zeros(1,point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    for poly = 1:size(shrunk_polytopes,2) % check each polytope
        verts = length(shrunk_polytopes(poly).xv);
        shrunk_polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
        flag_thisIsABeginningEnd([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
        curpt = curpt+verts;
    end
    obstacle_ID = [shrunk_polytopes.obs_id];

    % Make a list of all possible visible point. These are simply the
    % vertices of the polytopes. For each vertex, save the index of the
    % vertex in the 3rd column, the index of the obstacle in 
    % 4th column column (this is called obstacle_ID), and the flag
    % indicating if the point is a beginning_end point.
    % all points has the following format:
    % [x y point_id obstacle_ID beg_end]
    all_pts = [[shrunk_polytopes.xv]; [shrunk_polytopes.yv]; 1:point_tot; obstacle_ID; flag_thisIsABeginningEnd]'; 
    Npts = size(all_pts,1);

    %% plan path
    % Add the start and finish points to the all_pts list. Give these the
    % index of Npts plus 1 or 2, so that they are indexed as the last 2
    % points. They have a special obstacle ID of -1 (because they aren't
    % obstacles), and they are both flagged as start/end points.
    start = [startPoint Npts+1 -1 1];
    finish = [endPoint Npts+2 -1 1];

    % Why are these repeated?
    finishes = [all_pts; start; finish];
    starts   = [all_pts; start; finish];

    % Calculate the visibility graph
    % TO-DO - put into a visibility library later?
    [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPoints(shrunk_polytopes, starts, finishes);

    % Generate the cost graph. Flag the cost type (using a string) to
    % calculate cost based on XY spatial distance only (not energy)
    mode = 'xy spatial only';
    [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);

    % Call the A-star algorithm to do the path plan
    [cost, path] = fcn_BoundedAStar_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

    % path: series of points [x y point_id obs_id beg_end]
    % cost: path length
    % err: marker indicating if there was an error in setup (1) or not (0)

    % plot path
    if flag_do_plot
        plot(path(:,1),path(:,2),'k-','linewidth',2,'DisplayName', 'Planned Path')
        plot(startPoint(1,1), startPoint(1,2), 'gx','linewidth',2,'DisplayName','Start Point')
        plot(endPoint(1,1), endPoint(1,2), 'rx','linewidth',2,'DisplayName','End Point')
    end

    Npath = size(path,1);

    % Make vectors that are only going to be the points not including
    % start/end
    appex_x = zeros(Npath-2,3);
    appex_y = zeros(Npath-2,3);

    % Initialize  the previous point to the start point
    prev_pt = startPoint;
    for app = 1:Npath-2
        current_point = path(app+1,:);
        current_point_ID = current_point(3);
        current_point_obstacle = current_point(4); 

        % Find the points that are next to the current contact point. These
        % are the points that are adjacent, on each polytope, to the points
        % touched by the path contact point to the polytope

        % Get X and Y of current point
        appex_x(app,1) = current_point(1);
        appex_y(app,1) = current_point(2);

        % Check if current point has a start/finish flag
        if current_point(5) == 1
            % This point is flagged as a start/finish
            
            % Get the beginning/end point that is on the obstacle that the
            % current point belongs to, making sure not to select the
            % current point. Each obstacle has 2 beginning/end points, so
            % there will always be at least one OTHER point
            other_beg_end_pt = all_pts(((all_pts(:,4)==current_point_obstacle).*(all_pts(:,5)==1).*(all_pts(:,3)~=current_point_ID))==1,:);
            if other_beg_end_pt(3) > current_point_ID
                % The other point must be at the "end" of the obstacle list
                % So, make the other point's XY coordinates equal to the
                % 2nd point on the obstacle list (why???)
                other_pt = all_pts(current_point_ID+1,1:2);
            else % pt(3) > other_beg_end_pt(3)
                % The other point must be at the "start" of the obstacle
                % list of points. Set it equal to the point equal to the XY
                % coordinates of the 2nd to last point in the obstacle list
                % (again, why??)
                other_pt = all_pts(current_point_ID-1,1:2);
            end

            % Guess: this finds distance from previous point to BOTH
            % other_beg_end_point and other_pt
            %
            % NOTE: TO_DO: use the vector sum method for better speed and to avoid
            % function call to fcn_general_calculation_euclidean_point_to_point_distance

            dist = sum((ones(2,1)*prev_pt -[other_beg_end_pt(1:2); other_pt]).^2,2).^0.5;
            
            if dist(1) < dist(2) % other_pt farther
                appex_x(app,2:3) = [other_beg_end_pt(1), other_pt(1)];
                appex_y(app,2:3) = [other_beg_end_pt(2), other_pt(2)];
            else % other_pt closer
                appex_x(app,2:3) = [other_pt(1), other_beg_end_pt(1)];
                appex_y(app,2:3) = [other_pt(2), other_beg_end_pt(2)];
            end
        else
            % This is the "normal" case, namely that the current_point is
            % NOT a start or finish point. In this case, the adjacent
            % points are easy to determine - they are just the ID-1 for the
            % one before and the ID+1 for the one after. However, the
            % polytope may be oriented such that the path is hitting the
            % obstacle in a from/to direction where the the points may be
            % out of order. So to make sure that the first adjacent point
            % is the one where the path is coming from, and the second
            % adjacent point is in the direction the path is going to, we
            % need to calculate and compare the distance from the previous
            % path point to both these candiates, and order them
            % accordingly.

            previousPolytopePoint = all_pts(current_point_ID-1,1:2); % Grabs the previous point on the polytope
            nextPolytopePoint = all_pts(current_point_ID+1,1:2); % Grabs the next point on the polytope
            dist = sum((ones(2,1)*prev_pt - [previousPolytopePoint; nextPolytopePoint]).^2,2).^0.5;
            
            if dist(1) < dist(2) % pt1 closer
                appex_x(app,2:3) = [previousPolytopePoint(1), nextPolytopePoint(1)];
                appex_y(app,2:3) = [previousPolytopePoint(2), nextPolytopePoint(2)];
            else % pt1 farther
                appex_x(app,2:3) = [nextPolytopePoint(1), previousPolytopePoint(1)];
                appex_y(app,2:3) = [nextPolytopePoint(2), previousPolytopePoint(2)];
            end
        end
        prev_pt = current_point(1:2);
    end
    % appex_x = [appex_x1 closer_x1 farther_x1; appex_x2 closer_x2 farther_x2; .... appex_xn closer_xn farther_xn]
    % appex_y = [appex_y1 closer_y1 farther_y1; appex_y2 closer_y2 farther_y2; .... appex_yn closer_yn farther_yn]

    % Make the final plot for this iteration
    if flag_do_plot
        plot(appex_x,appex_y,'o','linewidth',2)
        my_title = sprintf('Path length [m]: %.4f',cost);
        title(my_title)
        box on
        % return
        pause(2)
        figure(fig_num); clf;
    end

    %% Final Info
    % A, B, appex_x, appex_y
    final_info(rep).polytopes = shrunk_polytopes;
    final_info(rep).start = A;
    final_info(rep).finish = B;
    final_info(rep).path_x = appex_x(:,1);
    final_info(rep).path_y = appex_y(:,1);
    final_info(rep).appex1_x = appex_x(:,2);
    final_info(rep).appex1_y = appex_y(:,2);
    final_info(rep).appex2_x = appex_x(:,3);
    final_info(rep).appex2_y = appex_y(:,3);
end
