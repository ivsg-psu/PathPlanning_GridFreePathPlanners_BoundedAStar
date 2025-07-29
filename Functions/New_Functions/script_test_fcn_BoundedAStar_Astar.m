% script_test_fcn_BoundedAStar_Astar.m
% tests fcn_BoundedAStar_Astar.m

% Revision history
% 2025_07_28 - sbrennan@psu.edu
% -- wrote the code originally, using 
%    % script_test_fcn_Laps_breakDataIntoLapIndices as starter

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

%% DEMO case: call the function to show it operating on the 9th data set
fig_num = 10001;
titleString = sprintf('DEMO case: call the function to show it operating on the 9th data set');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

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
    fig_num);

sgtitle(titleString, 'Interpreter','none');

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

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


% Plot the results
if 1==0
    fig_num = 9901;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: show how a lap is missed if start zone is not big enough, 9th example data
fig_num = 10002;
titleString = sprintf('DEMO case: show how a lap is missed if start zone is not big enough, 9th example data');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 9;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [6 3 0 0]; % Radius 10, 3 points must pass near [0,0]
end_definition = [30 3 0 -60]; % Radius 30, 3 points must pass near [0,-60]
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 2;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the laps starting at expected points?
assert(isequal(3,min(cell_array_of_lap_indices{1})));
assert(isequal(216,min(cell_array_of_lap_indices{2})));

% Are the laps ending at expected points?
assert(isequal(88,max(cell_array_of_lap_indices{1})));
assert(isequal(293,max(cell_array_of_lap_indices{2})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 9902;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: show the use of segment definition, 9th example data
fig_num = 10003;
titleString = sprintf('DEMO case: show the use of segment definition, 9th example data');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 9;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [10 0; -10 0]; % start at [10 0], end at [-10 0]
end_definition = [30 3 0 -60]; % Radius 30, 3 points must pass near [0,-60]
excursion_definition = []; % empty
[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(3,min(cell_array_of_lap_indices{1})));
assert(isequal(103,min(cell_array_of_lap_indices{2})));
assert(isequal(216,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(88,max(cell_array_of_lap_indices{1})));
assert(isequal(199,max(cell_array_of_lap_indices{2})));
assert(isequal(293,max(cell_array_of_lap_indices{3})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 9903;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end


%% DEMO case: call the function with 8th example data
fig_num = 10004;
titleString = sprintf('DEMO case: call the function with 8th example data');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 8;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [0 -10; 0 10]; % Line segment
end_definition = [90 0; 110 0]; % Line segment
excursion_definition = []; % empty

fig_num = 5;
[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(80,min(cell_array_of_lap_indices{1})));
assert(isequal(225,min(cell_array_of_lap_indices{2})));
assert(isequal(368,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(117,max(cell_array_of_lap_indices{1})));
assert(isequal(261,max(cell_array_of_lap_indices{2})));
assert(isequal(405,max(cell_array_of_lap_indices{3})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 8801;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: call the function to show it operating, 8th data set
fig_num = 10005;
titleString = sprintf('DEMO case: call the function to show it operating, 8th data set');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 8;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [0 10; 0 -10]; % Line segment, flipped direction
end_definition = [90 0; 110 0]; % Line segment
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(8,min(cell_array_of_lap_indices{1})));
assert(isequal(153,min(cell_array_of_lap_indices{2})));
assert(isequal(297,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(117,max(cell_array_of_lap_indices{1})));
assert(isequal(261,max(cell_array_of_lap_indices{2})));
assert(isequal(405,max(cell_array_of_lap_indices{3})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 8802;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: use a new set of laps data, 5th data set
fig_num = 10006;
titleString = sprintf('DEMO case: use a new set of laps data, 5th data set');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 5;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [20 0; -20 0]; % Line segment
end_definition = [-120 0; -80 0]; % Line segment
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(9,min(cell_array_of_lap_indices{1})));
assert(isequal(81,min(cell_array_of_lap_indices{2})));
assert(isequal(153,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(46,max(cell_array_of_lap_indices{1})));
assert(isequal(118,max(cell_array_of_lap_indices{2})));
assert(isequal(190,max(cell_array_of_lap_indices{3})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 5501;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: call the function to show it gives nothing if line segments are backwards, 5th data set
fig_num = 10007;
titleString = sprintf('DEMO case: call the function to show it gives nothing if line segments are backwards, 5th data set');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 5;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [20 0; -20 0]; % Line segment
end_definition = [ -80 0; -120 0;]; % Line segment
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 5502;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: use a new set of laps data, 6th data set
fig_num = 10008;
titleString = sprintf('DEMO case: use a new set of laps data, 6th data set');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 6;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

% Call the function to show it operating
start_definition = [20 0; -20 0]; % Line segment
end_definition = [-120 0; -80 0]; % Line segment
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

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
assert(isequal(9,min(cell_array_of_lap_indices{1})));
assert(isequal(82,min(cell_array_of_lap_indices{2})));
assert(isequal(226,min(cell_array_of_lap_indices{3})));

% Are the laps ending at expected points?
assert(isequal(46,max(cell_array_of_lap_indices{1})));
assert(isequal(190,max(cell_array_of_lap_indices{2})));
assert(isequal(334,max(cell_array_of_lap_indices{3})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


% Plot the results
if 1==0
    fig_num = 6601;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end

%% DEMO case: call the function to show gives weird results if start overlaps path twice
fig_num = 10009;
titleString = sprintf('DEMO case: call the function to show gives weird results if start overlaps path twice');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

dataSetNumber = 6;

% Load some test data 
tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

start_definition = [20 0; -20 0]; % Line segment
end_definition = []; % empty
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 6;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the laps starting at expected points?
for ith_lap = 1:Nlaps
    desiredStartPoint = [0 0];
    actualStartPoint = tempXYdata(cell_array_of_lap_indices{ith_lap}(1,1));
    assert(20>=sum((desiredStartPoint-actualStartPoint).^2,2).^0.5);
end

% Are the laps ending at expected points?
for ith_lap = 1:Nlaps
    desiredEndPoint = [0 0];
    actualEndPoint = tempXYdata(cell_array_of_lap_indices{ith_lap}(1,1));
    assert(20>=sum((desiredEndPoint-actualEndPoint).^2,2).^0.5);
end

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

% Plot the results
if 1==0
    fig_num = 6602;
    INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num);
end


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

tempXYdata = [-1 1; 1 1];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


%% TEST case: This one returns nothing since there is one point in criteria
fig_num = 20002;
titleString = sprintf('TEST case: This one returns nothing since there is one point in criteria');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

tempXYdata = [-1 1; 0 0; 1 1];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: This one returns nothing since there is only two points in criteria
fig_num = 20003;
titleString = sprintf('TEST case: This one returns nothing since there is only two points in criteria');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

tempXYdata = [-1 1; 0 0; 0.1 0; 1 1];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: returns nothing since the minimum point is at the start
% and so there is no strong minimum inside the zone
fig_num = 20004;
titleString = sprintf('TEST case: returns nothing since the minimum point is at the start');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;


tempXYdata = [-1 1; 0 0; 0.01 0; 0.02 0; 0.03 0; 1 1];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: returns nothing since the minimum point is at the end
% and so there is no strong minimum inside the zone
fig_num = 20005;
titleString = sprintf('TEST case: returns nothing since the minimum point is at the end');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

tempXYdata = [-1 1; -0.03 0; -0.02 0; -0.01 0; 0 0; 1 1];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: returns nothing since the path doesn't return to start and no end spec given
fig_num = 20006;
titleString = sprintf('TEST case: returns nothing since the path doesn''t return to start and no end spec given');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

tempXYdata = [-1 1; -0.03 0; -0.02 0; 0 0; 0.1 0; 1 1];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: returns nothing since the end is incomplete
fig_num = 20007;
titleString = sprintf('TEST case: returns nothing since the end is incomplete');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;


% Create some data to plot
full_steps = (-1:0.1:1)';
zero_full_steps = 0*full_steps;
half_steps = (-1:0.1:0)';
zero_half_steps = 0*half_steps;

tempXYdata = ...
    [full_steps zero_full_steps; zero_half_steps half_steps];
start_definition = [0.2 3 0 0]; % Located at [0,0] with radius 0.2, 3 points
end_definition = [];
excursion_definition = [];

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable sizes
Nlaps = 0;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the indices empty?
assert(isempty(cell_array_of_lap_indices));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Show that start and end points can overlap by their boundaries
fig_num = 20008;
titleString = sprintf('TEST case: Show that start and end points can overlap by their boundaries');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;


% Create some data to plot
full_steps = (-1:0.1:1)';
zero_full_steps = 0*full_steps;
half_steps = (-1:0.1:0)';
zero_half_steps = 0*half_steps; %#ok<NASGU>

tempXYdata = ...
    [full_steps zero_full_steps];
start_definition = [0.5 3 -0.5 0]; % Located at [-0.5,0] with radius 0.5, 3 points
end_definition = [0.5 3 0.5 0]; % Located at [0.5,0] with radius 0.5, 3 points
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 1;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the laps starting at expected points?
assert(isequal(2,min(cell_array_of_lap_indices{1})));

% Are the laps ending at expected points?
assert(isequal(20,max(cell_array_of_lap_indices{1})));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));


%% TEST case: show that the start and end points can be at the absolute ends
fig_num = 20009;
titleString = sprintf('TEST case: show that the start and end points can be at the absolute ends');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;


% Create some data to plot
full_steps = (-1:0.1:1)';
zero_full_steps = 0*full_steps;
half_steps = (-1:0.1:0)';
zero_half_steps = 0*half_steps;

tempXYdata = ...
    [full_steps zero_full_steps];
start_definition = [0.5 3 -1 0]; % Located at [-1,0] with radius 0.5, 3 points
end_definition = [0.5 3 1 0]; % Located at [1,0] with radius 0.5, 3 points
excursion_definition = []; % empty

[cell_array_of_lap_indices, ...
    cell_array_of_entry_indices, cell_array_of_exit_indices] = ...
    fcn_Laps_breakDataIntoLapIndices(...
    tempXYdata,...
    start_definition,...
    end_definition,...
    excursion_definition,...
    fig_num);

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(iscell(cell_array_of_lap_indices));
assert(iscell(cell_array_of_entry_indices));
assert(iscell(cell_array_of_exit_indices));

% Check variable sizes
Nlaps = 1;
assert(isequal(Nlaps,length(cell_array_of_lap_indices))); 
assert(isequal(Nlaps,length(cell_array_of_entry_indices))); 
assert(isequal(Nlaps,length(cell_array_of_exit_indices))); 

% Check variable values
% Are the laps starting at expected points?
assert(isequal(1,min(cell_array_of_lap_indices{1})));

% Are the laps ending at expected points?
assert(isequal(21,max(cell_array_of_lap_indices{1})));

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
        %% Fails because start_definition is not correct type
        clc
        start_definition = [1 2];
        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition); %#ok<*ASGLU>

        %% Fails because start_definition is not correct type
        % Radius input is negative
        clc
        start_definition = [-1 2 3 4];
        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition);

        %% Fails because start_definition is not correct type
        % Radius input is negative
        clc
        start_definition = [0 2 3 4];
        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition);

        %% Fails because start_definition is not correct type
        % Num_inputs input is not positive
        clc
        start_definition = [1 0 3 4];
        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition);

        %% Warning because start_definition is 3D not 2D
        % Start_zone definition is a 3D point [radius num_points X Y Z]
        clc
        start_definition = [1 2 3 4 5];
        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition);

        %% Warning because start_definition is 3D not 2D
        % Start_zone definition is a 3D point [X Y Z; X Y Z]
        clc
        start_definition = [1 2 3; 4 5 6];
        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition);

        %% Warning because end_definition is 3D not 2D
        % End_zone definition is a 3D point [radius num_points X Y Z]
        clc
        start_definition = [1 2 3 4];
        end_definition = [1 2 3 4 5];

        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition,...
            end_definition);

        %% Warning because excursion_definition is 3D not 2D
        % Excursion_zone definition is a 3D point [radius num_points X Y Z]
        clc
        start_definition = [1 2 3 4];
        end_definition = [1 2 3 4];
        excursion_definition = [1 2 3 4 5];

        [lap_traversals, input_and_exit_traversals] = fcn_Laps_breakDataIntoLapIndices(...
            single_lap.traversal{1},...
            start_definition,...
            end_definition,...
            excursion_definition);
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

function INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num)
figure(fig_num);
clf

% Make first subplot
subplot(1,3,1);  
axis square
hold on;
title('Laps');
legend_text = {};
    
for ith_lap = 1:length(cell_array_of_lap_indices)
    plot(tempXYdata(cell_array_of_lap_indices{ith_lap},1),tempXYdata(cell_array_of_lap_indices{ith_lap},2),'.-','Linewidth',3);
    legend_text = [legend_text, sprintf('Lap %d',ith_lap)]; %#ok<AGROW>    
end
h_legend = legend(legend_text);
set(h_legend,'AutoUpdate','off');
temp1 = axis;

% Make second subplot
subplot(1,3,2);  
axis square
hold on;
title('Entry');
legend_text = {};
    
for ith_lap = 1:length(cell_array_of_entry_indices)
    plot(tempXYdata(cell_array_of_entry_indices{ith_lap},1),tempXYdata(cell_array_of_entry_indices{ith_lap},2),'.-','Linewidth',3);
    legend_text = [legend_text, sprintf('Lap %d',ith_lap)]; %#ok<AGROW>    
end
h_legend = legend(legend_text);
set(h_legend,'AutoUpdate','off');
temp2 = axis;

% Make third subplot
subplot(1,3,3);  
axis square
hold on;
title('Exit');
legend_text = {};
    
for ith_lap = 1:length(cell_array_of_exit_indices)
    plot(tempXYdata(cell_array_of_exit_indices{ith_lap},1),tempXYdata(cell_array_of_exit_indices{ith_lap},2),'.-','Linewidth',3);
    legend_text = [legend_text, sprintf('Lap %d',ith_lap)]; %#ok<AGROW>    
end
h_legend = legend(legend_text);
set(h_legend,'AutoUpdate','off');
temp3 = axis;

% Set all axes to same value, maximum range
max_axis = max([temp1; temp2; temp3]);
min_axis = min([temp1; temp2; temp3]);
good_axis = [min_axis(1) max_axis(2) min_axis(3) max_axis(4)];
subplot(1,3,1); axis(good_axis);
subplot(1,3,2); axis(good_axis);
subplot(1,3,3); axis(good_axis);


end

%% fcn_INTERNAL_loadExampleData
function tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber)
% Call the function to fill in an array of "path" type
laps_array = fcn_Laps_fillSampleLaps(-1);


% Use the last data
tempXYdata = laps_array{dataSetNumber};
end % Ends fcn_INTERNAL_loadExampleData