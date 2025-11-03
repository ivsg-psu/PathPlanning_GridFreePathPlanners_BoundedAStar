% script_test_fcn_Visibility_plotVGraph
% Tests: fcn_Visibility_plotVGraph

% REVISION HISTORY:
% (in BoundedAStar)
% 2025_10_06 - S. Brennan, sbrennan@psu.edu
% * In script_test_fcn_Visibility_plotVGraph
% -- first write of the script
% 2025_10_28 - S. Brennan, sbrennan@psu.edu
% * In script_test_fcn_Visibility_plotVGraph
% -- Moved function into VGraph library
% -- fixed addNudge bug
% -- changed inputs to allow saveFile input

% TO DO:
% -- set up fast mode tests
% -- bug in assertion checking against hardcoded expected results

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

%% DEMO case: clear and blocked edges of convex polytope, no optional inputs
figNum = 10001;
titleString = sprintf('DEMO case: plotting entire visibility graph, no optional inputs');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);

% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-');
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(1,length(h_plot))); 

% Check variable values
assert(isequal(h_plot.Parent,gca)); % The parent of the plot is the axes
assert(isequal(h_plot.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(h_plot.Parent.Parent.Number,figNum)); % The current figure is this figure

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: clear and blocked edges of convex polytope, with default optional inputs
figNum = 10002;
titleString = sprintf('DEMO case: plotting entire visibility graph, with default optional inputs');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
selectedFromToIndices = [];
saveFile = [];
h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), (figNum));
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(1,length(h_plot))); 

% Check variable values
assert(isequal(h_plot.Parent,gca)); % The parent of the plot is the axes
assert(isequal(h_plot.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(h_plot.Parent.Parent.Number,figNum)); % The current figure is this figure

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% DEMO case: clear and blocked edges of convex polytope, with specified start index
figNum = 10003;
titleString = sprintf('DEMO case: plotting entire visibility graph, with specified start index');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
selectedFromToIndices = 1;
saveFile = [];
h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), (figNum));
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(1,length(h_plot))); 

% Check variable values
assert(isequal(h_plot.Parent,gca)); % The parent of the plot is the axes
assert(isequal(h_plot.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(h_plot.Parent.Parent.Number,figNum)); % The current figure is this figure

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% DEMO case: clear and blocked edges of convex polytope, with specified start and end index
figNum = 10004;
titleString = sprintf('DEMO case: plotting entire visibility graph, with specified start and end index');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
selectedFromToIndices = [1 6];
saveFile = [];
h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), (figNum));
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(1,length(h_plot))); 

% Check variable values
assert(isequal(h_plot.Parent,gca)); % The parent of the plot is the axes
assert(isequal(h_plot.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(h_plot.Parent.Parent.Number,figNum)); % The current figure is this figure

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));



%% DEMO case: plotting entire visibility graph with saveFile specified
figNum = 10005;
titleString = sprintf('DEMO case: plotting entire visibility graph with saveFile specified');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
selectedFromToIndices = [];
saveFile = fullfile(pwd,'Images','fcn_Visibility_plotVGraph.gif');
h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), (figNum));
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(length(all_pts(:,1)),length(h_plot))); 

% Check variable values
currentHandle = get(h_plot(1));
assert(isequal(currentHandle.Parent,gca)); % The parent of the plot is the axes
assert(isequal(currentHandle.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(currentHandle.Parent.Parent.Number,figNum)); % The current figure is this figure

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

% %% TEST case: zero gap between polytopes
% figNum = 20001;
% titleString = sprintf('TEST case: zero gap between polytopes');
% fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
% figure(figNum); clf;



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
fprintf(1,'Figure: %.0f: FAST mode, empty fig_num\n',figNum);
figure(figNum); clf;

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
selectedFromToIndices = [];
saveFile = [];

h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), ([]));
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(1,length(h_plot))); 

% Check variable values
assert(isequal(h_plot.Parent,gca)); % The parent of the plot is the axes
assert(isequal(h_plot.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(h_plot.Parent.Parent.Number,figNum)); % The current figure is this figure

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, fig_num=-1\n',figNum);
figure(figNum); close(figNum);

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

% Call plotting function
selectedFromToIndices = [];
saveFile = [];

h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), (-1));
axis(goodAxis);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(all(ishandle(h_plot)));

% Check variable sizes
assert(isequal(1,length(h_plot))); 

% Check variable values
assert(isequal(h_plot.Parent,gca)); % The parent of the plot is the axes
assert(isequal(h_plot.Parent.Parent,gcf)); % The parent of the axes is the current figure
assert(isequal(h_plot.Parent.Parent.Number,figNum)); % The current figure is this figure

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

selectedFromToIndices = [];
saveFile = [];

Niterations = 1;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), ([]));
end
slow_method = toc;

figure(figNum);
close(figNum);

% load data
test_case_idx = 1;
[vgraph, all_pts, goodAxis, polytopes, start, finish, addNudge] = fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx);


% Plot the polytopes
fcn_INTERNAL_plotPolytopes(polytopes, figNum)
axis(goodAxis);

% Plot the start and end points
plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
plot(finish(1),finish(2),'r.','MarkerSize',20);
text(start(:,1),start(:,2)+addNudge,'Start');
text(finish(:,1),finish(:,2)+addNudge,'Finish');

% label point ids for debugging. The last two points are start and
% finish, so do not need to be plotted and labeled.
plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

selectedFromToIndices = [];
saveFile = [];

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    h_plot = fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-', (selectedFromToIndices), (saveFile), (-1));
end
fast_method = toc;


% Plot results as bar chart
figure(373737);
clf;
hold on;

X = categorical({'Normal mode','Fast mode'});
X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
Y = [slow_method fast_method ]*1000/Niterations;
bar(X,Y)
ylabel('Execution time (Milliseconds)')


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


    
    
end % Ends bug cases


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

%% fcn_INTERNAL_plotPolytopes
function fcn_INTERNAL_plotPolytopes(polytopes, figNum)
% A wrapper function for plotPolytopes, to plot the polytopes with same
% format

% axes_limits = [0 1 0 1]; % x and y axes limits
% axis_style = 'square'; % plot axes style
plotFormat.Color = 'Blue'; % edge line plotting
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 2; % linewidth of the edge
fillFormat = [1 0 0 1 0.4];
% FORMAT: fcn_MapGen_plotPolytopes(polytopes,fig_num,line_spec,line_width,axes_limits,axis_style);
fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum));
hold on
box on
% axis([-0.1 1.1 -0.1 1.1]);
xlabel('x [m]');
ylabel('y [m]');
end % Ends fcn_INTERNAL_plotPolytopes

%% fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph
function [vgraph, all_pts_with_startfinish, goodAxis, polytopes, start, finish, addNudge] = ...
    fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph(test_case_idx)
addNudge = 0.35;
if test_case_idx == 1
    % test case 1
    % Two polytopes with clear space right down middle
    clear polytopes
    polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
    polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
    polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
    goodAxis = [-3 7 -4 4];
    start = [-2 -0.5];
    finish = start;
    finish(1) = 6;
end
if test_case_idx == 2
    % test case 2
    % Stacked sets of squares
    clear polytopes
    polytopes(1).vertices = [0 0; 10 0; 10 1; 0 1; 0 0];
    polytopes(2).vertices = polytopes(1).vertices+[0,2];
    polytopes(3).vertices = polytopes(1).vertices+[0,5];
    polytopes(4).vertices = polytopes(1).vertices+[0,10];
    polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
    start = [0,9];
    finish = start + [10,0];
end

% Make sure all have same cost
for ith_poly = 1:length(polytopes)
    polytopes(ith_poly).cost = 0.4;
end

point_tot = length([polytopes.xv]); % total number of vertices in the polytopes
beg_end = zeros(1,point_tot); % is the point the start/end of an obstacle
curpt = 0;
for poly = 1:size(polytopes,2) % check each polytope
    verts = length(polytopes(poly).xv);
    polytopes(poly).obs_id = ones(1,verts)*poly; % obs_id is the same for every vertex on a single polytope
    beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+verts;
end
obs_id = [polytopes.obs_id];
all_pts = [[polytopes.xv];[polytopes.yv];1:point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]

start = [start size(all_pts,1)+1 -1 1]; 
finish = [finish size(all_pts,1)+2 -1 1]; 
all_pts_with_startfinish = [all_pts; start; finish];

% finishes = [all_pts; start; finish];
% starts = [all_pts; start; finish];
[vgraph, ~] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, all_pts_with_startfinish, all_pts_with_startfinish,1);


end % Ends fcn_INTERNAL_loadExampleData_fcn_Visibility_plotVGraph


