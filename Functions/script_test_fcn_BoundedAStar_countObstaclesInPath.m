% script_test_fcn_BoundedAStar_countObstaclesInPath

% test of fcn_BoundedAStar_countObstaclesInPath, which counts the number of
% obstacles encountered while traversing a given path

% Revision History:
% 2025_08_25 by K. Hayes
% -- first write of script using script_test_fcn_BoundedAStar_greedyPlanner
%    as starter

%% Set up the workspace
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
% Figures start with 1

close all;
fprintf(1,'Figure: 1XXXXXX: DEMO cases\n');
%% DEMO case: count encountered obstacles in a given path plan
fig_num = 10001;
titleString = sprintf('DEMO case: count encountered obstacles in a given path plan');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

repetitions = 1;
rep = 1;
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

% start and finish (x,y) coordinates
start_xy = [0.0, 0.5];
finish_xy = [1, 0.5];

% generate map
% generate Voronoi tiling from Halton points
low_pt = low_pts(rep); high_pt = high_pts(rep)-50; % range of Halton points to use to generate the tiling
tiled_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [low_pt,high_pt],[],[],-1);
% remove the edge polytope that extend past the high and low points    
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( tiled_polytopes, [xlow ylow xhigh yhigh], (-1));

% shink the polytopes so that they are no longer tiled
rng(shrink_seed) % set the random number generator with the shrink seed    
shrunk_polytopes = fcn_MapGen_polytopesShrinkToRadius(trim_polytopes,des_radius,sigma_radius,min_rad, -1);
shrunk_polytopes = fcn_MapGen_polytopesSetCosts(shrunk_polytopes, des_cost, (-1));

% info needed for further work
% gather data on all the points

[all_pts,start,finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes,start_xy,finish_xy,-1);

% Generate visibility graph
finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
[vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes);

% Plan path through field using greedy planner
% Could also change this to be Astar. The important part is that a path is
% generated to be used for the calculation.
[cost,route] = fcn_BoundedAStar_greedyPlanner(vgraph, all_pts, start, finish, (shrunk_polytopes), (-1));

% Call fcn to count obstacles in the path 
[obs_around, obs_through] = fcn_BoundedAStar_countObstaclesInPath(route);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(obs_around));
assert(isnumeric(obs_through));

% Check variable values
assert(isequal(obs_around,2));
assert(isequal(obs_through,4));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));



%% Test cases start here. These are very simple, usually trivial
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  _______ ______  _____ _______ _____
% |__   __|  ____|/ ____|__   __/ ____|
%    | |  | |__  | (___    | | | (___
%    | |  |  __|  \___ \   | |  \___ \
%    | |  | |____ ____) |  | |  ____) |
%    |_|  |______|_____/   |_| |_____/
%
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 2

close all;
fprintf(1,'Figure: 2XXXXXX: TEST mode cases\n');

%% TEST case: zero gap between polytopes
fig_num = 20001;
titleString = sprintf('TEST case: zero gap between polytopes');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

%% Fast Mode Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ______        _     __  __           _        _______        _
% |  ____|      | |   |  \/  |         | |      |__   __|      | |
% | |__ __ _ ___| |_  | \  / | ___   __| | ___     | | ___  ___| |_ ___
% |  __/ _` / __| __| | |\/| |/ _ \ / _` |/ _ \    | |/ _ \/ __| __/ __|
% | | | (_| \__ \ |_  | |  | | (_) | (_| |  __/    | |  __/\__ \ |_\__ \
% |_|  \__,_|___/\__| |_|  |_|\___/ \__,_|\___|    |_|\___||___/\__|___/
%
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Fast%20Mode%20Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 8

close all;
fprintf(1,'Figure: 8XXXXXX: FAST mode cases\n');

%% Basic example - NO FIGURE
fig_num = 80001;
fprintf(1,'Figure: %.0f: FAST mode, empty fig_num\n',fig_num);
figure(fig_num); close(fig_num);

%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
fig_num = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',fig_num);
figure(fig_num);
close(fig_num);

% map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.01 0.001 1e-6 1111";
% plot_flag = 1; 
% disp_name = 0; 
% 
% line_style = 'r-';
% line_width = 2;
% 
% Niterations = 10;
% 
% % Do calculation without pre-calculation
% tic;
% for ith_test = 1:Niterations
%     % Call the function
%     [polytopes, h_fig] = fcn_MapGen_generatePolysFromName(map_name, plot_flag, disp_name, ([]), (line_style), (line_width));
% end
% slow_method = toc;
% 
% % Do calculation with pre-calculation, FAST_MODE on
% tic;
% for ith_test = 1:Niterations
%     % Call the function
%     [polytopes, h_fig] = fcn_MapGen_generatePolysFromName(map_name, plot_flag, disp_name, (-1), (line_style), (line_width));
% end
% fast_method = toc;
% 
% % Make sure plot did NOT open up
% figHandles = get(groot, 'Children');
% assert(~any(figHandles==fig_num));
% 
% % Plot results as bar chart
% figure(373737);
% clf;
% hold on;
% 
% X = categorical({'Normal mode','Fast mode'});
% X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
% Y = [slow_method fast_method ]*1000/Niterations;
% bar(X,Y)
% ylabel('Execution time (Milliseconds)')
% 
% 
% % Make sure plot did NOT open up
% figHandles = get(groot, 'Children');
% assert(~any(figHandles==fig_num));

%% BUG cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ____  _    _  _____
% |  _ \| |  | |/ ____|
% | |_) | |  | | |  __    ___ __ _ ___  ___  ___
% |  _ <| |  | | | |_ |  / __/ _` / __|/ _ \/ __|
% | |_) | |__| | |__| | | (_| (_| \__ \  __/\__ \
% |____/ \____/ \_____|  \___\__,_|___/\___||___/
%
% See: http://patorjk.com/software/taag/#p=display&v=0&f=Big&t=BUG%20cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All bug case figures start with the number 9

% close all;

%% BUG

%% Fail conditions
if 1==0

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
