% script_test_fcn_BoundedAStar_checkReachability

% Tests: script_test_fcn_BoundedAStar_checkReachability

% Revision history
% 2025_08_12 - K. Hayes, kxh1031@psu.edu
% -- first write of script

% TO DO:
% -- set up fast mode tests

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
%% DEMO case: check reachability of points
fig_num = 10001;
titleString = sprintf('DEMO case: check reachability of points');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

tiled_polytopes = fcn_MapGen_haltonVoronoiTiling([1,20],[1 1]);
% remove the edge polytope that extend past the high and low points
% shink the polytopes so that they are no longer tiled
des_radius = 0.05; % desired average maximum radius
sigma_radius = 0.002; % desired standard deviation in maximum radii
min_rad = 0.0001; % minimum possible maximum radius for any obstacle
[shrunk_polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes,des_radius,sigma_radius,min_rad);

max_translation_distance = 0.15;
final_time = 20;
time_space_polytopes = fcn_make_timespace_polyhedra_from_polygons(shrunk_polytopes, max_translation_distance, final_time);

time_space_polytopes = fcn_make_facets_from_verts(time_space_polytopes);

all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes);


% define start and finish
start = [0 0.5 0];
finish = [1 0.5 0; 0.7 0.2 20]; % moving finish
dt = 5;
finish = fcn_interpolate_route_in_time(finish,dt);
num_finish_pts = size(finish,1);
starts = [start(1)*ones(num_finish_pts,1) start(2)*ones(num_finish_pts,1) start(3)*ones(num_finish_pts,1)];

% interpolate vertices in time and form all_pts matrix
[verts, time_space_polytopes] = fcn_BoundedAStar_interpolatePolytopesInTime(time_space_polytopes,dt);

verts = verts(:,1:3);
all_pts = [verts; start; finish];

num_verts = size(verts,1);

num_pts = size(all_pts,1);
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts

% if flag_do_plot
%     figure; hold on; box on; title('all vertices and start and finish')
%     INTERNAL_fcn_format_timespace_plot();
%     plot3(start(1),start(2),start(3),'gx');
%     plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
%     plot3(verts(:,1),verts(:,2),verts(:,3),'cx')
% end

% set speed limit and form visibility graph
speed_limit = 0.12;
vgraph = fcn_Visibility_3dGraphGlobal(verts, start, finish, all_surfels, speed_limit, time_space_polytopes, dt);

num_starts = size(start,1);
num_finishes = size(finish,1);

start_with_ids = all_pts(num_verts+1:num_verts+num_starts,:);
finish_with_ids = all_pts(num_verts+num_starts+1:num_verts+num_starts+num_finishes,:);
all_pts_with_ids_no_start_and_fin = all_pts(1:num_verts,:);

% form reachability graph
[is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph, start_with_ids(:,4), finish_with_ids(:,4), all_pts, fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(is_reachable));
assert(isnumeric(num_steps));
assert(islogical(rgraph));

% Check variable sizes
% Npolys = 100;
% assert(isequal(Npolys,length(polytopes))); 

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