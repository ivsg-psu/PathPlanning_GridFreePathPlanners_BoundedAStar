% script_test_fcn_VGraph_clearAndBlockedPoints

% a basic test of calculation of clear and blocked points for visibility
% graph creation

% Revision history
% (In BoundedAStar)
% As: script_test_fcn_Visibility_clearAndBlockedPoints
% 2025_07_25 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_visibility_clear_and_blocked_points
% 2025_08_01 - K. Hayes
% -- cleaning and reformatting of script
% -- moved plotting capabilities into fcn debug options
% (In Visibility Graph)
% 2025_10_30 - S. Brennan
% -- set up fast mode tests
% -- improved figure captions a bit
%
% As: script_test_fcn_VGraph_clearAndBlockedPoints
% 2025_11_07 - S. Brennan
% -- Renamed script_test_fcn_Visibility_clearAndBlockedPoints to script_test_fcn_VGraph_clearAndBlockedPoints
% -- Cleared extra figure command out of Inputs section


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

%% DEMO case: find clear and blocked edges of polytopes in a map
figNum = 10001;
titleString = sprintf('DEMO case: find clear and blocked edges of polytopes in a map');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Create polytope field
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[], ([100 100]), (-1));

% Trim polytopes on edge of boundary
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0.1 0.1 99.9 99.9], (-1));

% Shrink polytopes to form obstacle field
shrunk_polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

% Get x and y coordinates of each polytope
xvert = [shrunk_polytopes.xv];
yvert = [shrunk_polytopes.yv];
point_tot = length(xvert);

% Create start and finish points
start = [0 50 point_tot+1 -1 0];
finish = [[100; xvert'] [50; yvert'] [point_tot+2; (1:point_tot)'] [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];

% Call function to determine clear and blocked points
isConcave = [];
[clear_pts,blocked_pts]=fcn_VGraph_clearAndBlockedPoints(shrunk_polytopes,start,finish,(isConcave),(figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(clear_pts));
assert(isnumeric(blocked_pts));

% Check variable sizes
Npolys = 100;
assert(isequal(Npolys,length(polytopes))); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


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
figNum = 20001;
titleString = sprintf('TEST case: zero gap between polytopes');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


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
figNum = 80001;
fprintf(1,'Figure: %.0f: FAST mode, empty figNum\n',figNum);
figure(figNum); close(figNum);

% Create polytope field
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[], ([100 100]), (-1));

% Trim polytopes on edge of boundary
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0.1 0.1 99.9 99.9], (-1));

% Shrink polytopes to form obstacle field
shrunk_polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

% Get x and y coordinates of each polytope
xvert = [shrunk_polytopes.xv];
yvert = [shrunk_polytopes.yv];
point_tot = length(xvert);

% Create start and finish points
start = [0 50 point_tot+1 -1 0];
finish = [[100; xvert'] [50; yvert'] [point_tot+2; (1:point_tot)'] [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];

% Call function to determine clear and blocked points
isConcave = [];
[clear_pts,blocked_pts]=fcn_VGraph_clearAndBlockedPoints(shrunk_polytopes,start,finish,(isConcave),([]));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(clear_pts));
assert(isnumeric(blocked_pts));

% Check variable sizes
Npolys = 100;
assert(isequal(Npolys,length(polytopes))); 


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

% Create polytope field
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[], ([100 100]), (-1));

% Trim polytopes on edge of boundary
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0.1 0.1 99.9 99.9], (-1));

% Shrink polytopes to form obstacle field
shrunk_polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

% Get x and y coordinates of each polytope
xvert = [shrunk_polytopes.xv];
yvert = [shrunk_polytopes.yv];
point_tot = length(xvert);

% Create start and finish points
start = [0 50 point_tot+1 -1 0];
finish = [[100; xvert'] [50; yvert'] [point_tot+2; (1:point_tot)'] [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];

% Call function to determine clear and blocked points
isConcave = [];
[clear_pts,blocked_pts]=fcn_VGraph_clearAndBlockedPoints(shrunk_polytopes,start,finish,(isConcave),(-1));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(clear_pts));
assert(isnumeric(blocked_pts));

% Check variable sizes
Npolys = 100;
assert(isequal(Npolys,length(polytopes))); 


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

% Load some test data 
% Create polytope field
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[], ([100 100]), (-1));

% Trim polytopes on edge of boundary
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0.1 0.1 99.9 99.9], (-1));

% Shrink polytopes to form obstacle field
shrunk_polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

% Get x and y coordinates of each polytope
xvert = [shrunk_polytopes.xv];
yvert = [shrunk_polytopes.yv];
point_tot = length(xvert);

% Create start and finish points
start = [0 50 point_tot+1 -1 0];
finish = [[100; xvert'] [50; yvert'] [point_tot+2; (1:point_tot)'] [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];

% Call function to determine clear and blocked points
isConcave = [];

Niterations = 50;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [clear_pts,blocked_pts]=fcn_VGraph_clearAndBlockedPoints(shrunk_polytopes,start,finish,(isConcave),([]));


end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [clear_pts,blocked_pts]=fcn_VGraph_clearAndBlockedPoints(shrunk_polytopes,start,finish,(isConcave),(-1));


end
fast_method = toc;

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));

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
assert(~any(figHandles==figNum));


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