% script_test_fcn_BoundedAStar_Astar.m
% tests fcn_BoundedAStar_Astar.m

% Revision history
% 2025_07_28 - sbrennan@psu.edu
% -- wrote the code originally, using 
%    % script_test_fcn_Laps_breakDataIntoLapIndices as starter
% 2025_08_18 - K. Hayes, kxh1031@psu.edu
% -- updated demo cases

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

%% DEMO case: plan a path through a polytope field using A*
fig_num = 10001;
titleString = sprintf('DEMO case: plan a path through a polytope field using A*');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

repetitions = 1;
rep = 1;

% map generation control
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

% starting and finish coordinates
startPoint = [0 0.5];
endPoint   = [1 0.5];

% generate a new map
% generate Voronoi tiling from Halton points
low_pt = low_pts(rep); high_pt = high_pts(rep)-50; % range of Halton points to use to generate the tiling
tiled_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [low_pt high_pt],[],[],-1);

% remove the edge polytope that extend past the high and low points
trim_polytopes  = fcn_MapGen_polytopesDeleteByAABB( tiled_polytopes, [0.001 0.001, 0.999 0.999], (-1));

% shink the polytopes so that they are no longer tiled
rng(shrink_seed) % set the random number generator with the shrink seed
shrunk_polytopes = fcn_MapGen_polytopesShrinkToRadius(trim_polytopes,des_radius,sigma_radius,min_rad, (-1));

des_cost = 0.1;
shrunk_polytopes = fcn_MapGen_polytopesSetCosts(shrunk_polytopes, des_cost, (-1));

% create all_pts matrix
all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes, startPoint, endPoint, -1);
Npts = size(all_pts,1);

% plan path
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
[vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes, [], -1);

% Generate the cost graph. Flag the cost type (using a string) to
% calculate cost based on XY spatial distance only (not energy)
mode = 'xy spatial only';
[cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode, -1);

% Call the A-star algorithm to do the path plan
[cost, path] = fcn_BoundedAStar_Astar(vgraph, cgraph, hvec, all_pts, start, finish, shrunk_polytopes, (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(cost));
assert(isnumeric(path));

% Check variable sizes
Npoints = 6;
assert(isequal(Npoints,length(path))); 

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

%% TEST case: This one returns nothing since there is no portion of the path in criteria
fig_num = 20001;
titleString = sprintf('TEST case: This one returns nothing since there is no portion of the path in criteria');
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

dataSetNumber = 9;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [10 3 0 0]; % Radius 10, 3 points must pass near [0,0]
end_definition = [30 3 0 -60]; % Radius 30, 3 points must pass near [0,-60]
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    ([]));

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 3;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the laps starting at expected points?
assert(isequal(2,min(cell_array_of_lap_indices{1})));
assert(isequal(102,min(cell_array_of_lap_indices{2})));
assert(isequal(215,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(88,max(cell_array_of_lap_indices{1})));
assert(isequal(199,max(cell_array_of_lap_indices{2})));
assert(isequal(293,max(cell_array_of_lap_indices{3})));

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==fig_num));


%% Basic fast mode - NO FIGURE, FAST MODE
fig_num = 80002;
fprintf(1,'Figure: %.0f: FAST mode, fig_num=-1\n',fig_num);
figure(fig_num); close(fig_num);

dataSetNumber = 9;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [10 3 0 0]; % Radius 10, 3 points must pass near [0,0]
end_definition = [30 3 0 -60]; % Radius 30, 3 points must pass near [0,-60]
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    (-1));

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 3;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the laps starting at expected points?
assert(isequal(2,min(cell_array_of_lap_indices{1})));
assert(isequal(102,min(cell_array_of_lap_indices{2})));
assert(isequal(215,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(88,max(cell_array_of_lap_indices{1})));
assert(isequal(199,max(cell_array_of_lap_indices{2})));
assert(isequal(293,max(cell_array_of_lap_indices{3})));

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==fig_num));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
fig_num = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',fig_num);
figure(fig_num);
close(fig_num);

dataSetNumber = 9;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [10 3 0 0]; % Radius 10, 3 points must pass near [0,0]
end_definition = [30 3 0 -60]; % Radius 30, 3 points must pass near [0,-60]
excursion_definition = []; % empty

 
Niterations = 50;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [cell_array_of_lap_indices, ...
        cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
        fcn_Laps_breakDataIntoLapIndices(...
        tempXYdata,...
        start_definition,...
        end_definition,...
        excursion_definition,...
        ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [cell_array_of_lap_indices, ...
        cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
        fcn_Laps_breakDataIntoLapIndices(...
        tempXYdata,...
        start_definition,...
        end_definition,...
        excursion_definition,...
        (-1));
end
fast_method = toc;

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==fig_num));

% Plot results as bar chart
figure(373737);
clf;
hold on;

X = categorical({'Normal mode','Fast mode'});
X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
Y = [slow_method fast_method ]*1000/Niterations;
bar(X,Y)
ylabel('Execution time (Milliseconds)')


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==fig_num));


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
    %
      
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

