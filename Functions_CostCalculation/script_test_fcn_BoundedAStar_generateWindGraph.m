% script_test_fcn_BoundedAStar_generateWindGraph
% Tests: fcn_BoundedAStar_generateWindGraph

% Revision history
% 2025_07_16 by K. Hayes, kxh1031@psu.edu
% -- first write of script


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

%% DEMO case: create a graph within a randomly generated wind field
fig_num = 10001;
titleString = sprintf('DEMO case: create a graph within a randomly generated wind field');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [0 0 1 1];
peaksMode = [];
n_nodes = 5;

% Call wind field generation function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode),(-1));

% Call graph generation function
[vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, n_nodes, (randomSeed), (fig_num));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vertices));
assert(isnumeric(edges));
assert(isnumeric(costgraph));

% Check variable sizes
Npoints = n_nodes;
assert(size(vertices,1)==Npoints); 
assert(size(vertices,2)==2); 

assert(size(edges,1)==Npoints); 
assert(size(edges,2)==Npoints); 

assert(size(costgraph,1)==Npoints);
assert(size(costgraph,2)==Npoints);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% DEMO case: use A* planner to find a path between start and finish
fig_num = 10002;
titleString = sprintf('DEMO case: use A* planner to find a path between two nodes');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% Fill inputs
randomSeed = 682189;
windMagnitude = [];
NpointsInSide = [];
XY_range = [0 0 1 1];
peaksMode = [];
n_nodes = 25;
rngSeed = [];

% Call wind field generation function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode),(-1));

% Call graph generation function
start = [0, 0.3 , n_nodes+1, -1, 0];
finish = [0.8, 0.8, n_nodes+2, -1, 0];

[vertices, edges, costgraph] = fcn_BoundedAStar_generateWindGraph(windFieldU, windFieldV, x, y, n_nodes, start, finish, (rngSeed), (fig_num));

% Call A*
%hvec = sum((vertices - finish(1:2)).^2,2).^0.5';

% Use calculated costs to go from any node to end node as heuristic
% function
hvec = costgraph(:,end)';
all_pts = [vertices(1:n_nodes,:), [1:n_nodes]',-1*ones(n_nodes,1), zeros(n_nodes,1)];

[cost, route] = fcn_algorithm_Astar(edges, costgraph, hvec, all_pts, start, finish);

figure(99)
hold on
plot(vertices(:,1),vertices(:,2),'.','MarkerSize',10)
plot(route(:,1),route(:,2),'Color','black','Linewidth',3)
plot(start(1),start(2),'x','Color','red','MarkerSize',10)
plot(finish(1),finish(2),'x','Color','green','MarkerSize',10)

% Plot the wind field
    if numel(windFieldU) > 250
        NpointsInSide = length(windFieldU(:,1));
        indices = (1:NpointsInSide); % Row vector
        Xindices = repmat(indices,NpointsInSide,1);
        Yindices = repmat(indices',1,NpointsInSide);
    
        moduloX = mod(Xindices,25); % Keep only 1 of every 25
        moduloY = mod(Yindices,25); % Keep only 1 of every 25
        
        moduloXreshaped = reshape(moduloX,[],1);
        moduloYreshaped = reshape(moduloY,[],1);
    
        indicesX = find(moduloXreshaped==1);
        indicesY = find(moduloYreshaped==1);
    
        [X,Y] = meshgrid(x,y);
    
        indicesToPlot = intersect(indicesX,indicesY);
        quiver(X(indicesToPlot),Y(indicesToPlot),windFieldU(indicesToPlot),windFieldV(indicesToPlot));
    else 
        quiver(x,y,windFieldU,windFieldV);
    end

    %legend('Interpreter','none');
    xlabel('X-East');
    ylabel('Y-North');

    axis equal;

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vertices));
assert(isnumeric(edges));
assert(isnumeric(costgraph));

% Check variable sizes
Npoints = n_nodes;
assert(size(vertices,1)==Npoints+2); 
assert(size(vertices,2)==2); 

assert(size(edges,1)==Npoints+2); 
assert(size(edges,2)==Npoints+2); 

assert(size(costgraph,1)==Npoints+2);
assert(size(costgraph,2)==Npoints+2);

% Make sure plot opened up
% assert(isequal(get(gcf,'Number'),fig_num));


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



%% TEST case: Non-default NpointsInSide for random map generation
fig_num = 20002;
titleString = sprintf('TEST case: Non-default NpointsInSide for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;



%% TEST case: Non-default windMagnitude for random map generation
fig_num = 20003;
titleString = sprintf('TEST case: Non-default windMagnitude for random map generation');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;



%% TEST case: Non-default randomSeed for random map generation
fig_num = 20004;
titleString = sprintf('TEST case: Non-default randomSeed for random map generation');
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

% Fill inputs
randomSeed = [];
windMagnitude = [];
NpointsInSide = [];
XY_range = [];
peaksMode = [];

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), ([]));

% Check variable types
assert(isnumeric(windFieldU));
assert(isnumeric(windFieldV));
assert(isnumeric(x));
assert(isnumeric(y));

% Check variable sizes
Npoints = 200;
assert(size(windFieldU,1)==Npoints); 
assert(size(windFieldU,2)==Npoints); 

assert(size(windFieldV,1)==Npoints); 
assert(size(windFieldV,2)==Npoints); 

assert(size(x,1)==1);
assert(size(x,2)==Npoints);

assert(size(y,1)==1);
assert(size(y,2)==Npoints);

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

% Call function
[windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));

% Check variable types
assert(isnumeric(windFieldU));
assert(isnumeric(windFieldV));
assert(isnumeric(x));
assert(isnumeric(y));

% Check variable sizes
Npoints = 200;
assert(size(windFieldU,1)==Npoints); 
assert(size(windFieldU,2)==Npoints); 

assert(size(windFieldV,1)==Npoints); 
assert(size(windFieldV,2)==Npoints); 

assert(size(x,1)==1);
assert(size(x,2)==Npoints);

assert(size(y,1)==1);
assert(size(y,2)==Npoints);

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

Niterations = 10;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [windFieldU, windFieldV, x, y] = fcn_BoundedAStar_fillWindField( (XY_range), (NpointsInSide), (windMagnitude), (randomSeed), (peaksMode), (-1));
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
