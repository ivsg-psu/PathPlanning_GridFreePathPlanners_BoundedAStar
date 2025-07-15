% script_test_fcn_BoundedAStar_calcCostChangingWind
% Tests: fcn_BoundedAStar_calcCostChangingWind

% Revision history
% 2021_07_11 by Sean Brennan
% -- first write of script
% 2025_07_14 by K. Hayes, kxh1031@psu.edu
% -- cleaned script
% -- fixed issue with missing variable windVector
% -- added test cases for non-default map generation parameters
% -- added fast mode test cases
% 2025_07_15 by K. Hayes
% -- added plotting support for coarse grids (small NpointsInSide)

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

%% DEMO case: basic call to function with randomly generated wind field
fig_num = 10001;
titleString = sprintf('DEMO case: basic call to function with randomly generated wind field');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Fill wind fields (running in fast mode)
[windFieldU, windFieldV, x, y]  = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

% Call function
radius = 5;
startPoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


%% DEMO case: basic call to function with peaks mode enabled
fig_num = 10002;
titleString = sprintf('DEMO case: basic call to function with peaks mode enabled');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = 1;

% Fill wind fields (running in fast mode)
[windFieldU, windFieldV, x, y]  = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), peaksMode, (-1));

% Call function
radius = 5;
startPoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

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

%% TEST case: Non-default, non-uniform XY range for random map generation
fig_num = 20001;
titleString = sprintf('TEST case: Non-default, non-uniform XY range for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [-20, -5, 15, 25];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

radius = 5;
startpoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Non-default NpointsInSide for random map generation
fig_num = 20002;
titleString = sprintf('TEST case: Non-default NpointsInSide for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = 6;
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), 4564869);

radius = 5;
startPoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Non-default windMagnitude for random map generation
fig_num = 20003;
titleString = sprintf('TEST case: Non-default windMagnitude for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = 2;
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

radius = 5;
startPoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Non-default randomSeed for random map generation
fig_num = 20004;
titleString = sprintf('TEST case: Non-default randomSeed for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = 4584531;
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

radius = 5;
startPoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Non-default radius of travel pre-wind
fig_num = 20005;
titleString = sprintf('TEST case: Non-default radius of travel pre-wind');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

radius = 6;
startPoint = [];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Non-default start point
fig_num = 20006;
titleString = sprintf('TEST case: Non-default start point');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

radius = 5;
startPoint = [-2,4];
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (startPoint), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

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

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Fill wind fields (running in fast mode)
[windFieldU, windFieldV, x, y]  = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

% Call function
radius = 5;
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, ([]));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==fig_num));


%% Basic fast mode - NO FIGURE, FAST MODE
fig_num = 80002;
fprintf(1,'Figure: %.0f: FAST mode, fig_num=-1\n',fig_num);
figure(fig_num); close(fig_num);

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Fill wind fields (running in fast mode)
[windFieldU, windFieldV, x, y]  = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), peaksMode, (-1));

% Call function
radius = 5;
windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (-1));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(windRadius));

% Check variable sizes
Npoints = 629;
assert(size(windRadius,1)==Npoints); 
assert(size(windRadius,2)==2); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==fig_num));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
fig_num = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',fig_num);
figure(fig_num);
close(fig_num);

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Fill wind fields (running in fast mode)
[windFieldU, windFieldV, x, y]  = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), peaksMode, (-1));

radius = 5;

Niterations = 10;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    windRadius = fcn_BoundedAStar_calcCostChangingWind(radius, windFieldU, windFieldV, x, y, (-1));
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


