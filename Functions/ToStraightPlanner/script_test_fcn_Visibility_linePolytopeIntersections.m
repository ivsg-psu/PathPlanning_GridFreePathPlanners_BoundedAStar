% script_test_fcn_Visibility_linePolytopeIntersections
% Tests: fcn_Visibility_linePolytopeIntersections

% a basic test of calculation of line intersections with polytopes

% Revision history
% 2025_07_31 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_visibility_line_polytope_intersections
% 2025_08_01 - K. Hayes
% -- updated script to be compatible with new MapGen
% -- moved plotting into fcn debug options
% -- reformatted script
% 2025_10_03 - K. Hayes
% -- fixed bug causing assertion in DEMO case 1 to fail

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

%% DEMO case: find edges where a line intersects a polytope
fig_num = 10001;
titleString = sprintf('DEMO case: find edges where a line intersects a polytope');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Set vertices of polytope
xv = [2 3 5 6 6 5 3 2];
yv = [3 2 2 3 5 6 6 5];
polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
polytopes.xv = xv;
polytopes.yv = yv;

% Find distances of polytope edges
polytopes.distances = sum((polytopes.vertices(1:end-1,:) - polytopes.vertices(2:end,:)).^2,2).^0.5;

% Find polytope area and centroid location
[centroid,polytope.area] = fcn_MapGen_polytopeCentroidAndArea ([[xv xv(1)]',[yv yv(1)]']);
polytopes.mean = [centroid];

% Find maximum radius of polytope
polytopes.max_radius = max(sum((polytopes.vertices(1:end-1,:) - ones(length(xv),1)*polytopes.mean).^2,2).^0.5);
polytopes.cost = 0.1;

% Append start and finish points
point_tot = length(xv);
start = [0 0 point_tot+1 -1 0];
finish = [8 8 point_tot+2 0 0];

% Find clear and blocked points
[clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_Visibility_clearAndBlockedPoints(polytopes,start,finish,[],-1);
xings = fcn_Visibility_linePolytopeIntersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes,([xjQ; yjQ]),([start; finish]),(fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isstruct(xings));
assert(isfield(xings,'points'));
assert(isfield(xings,'index'));
assert(isfield(xings,'obstacles'));

% Check variable sizes
Nxings = 2;
assert(isequal(Nxings,length(xings.points))); 

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