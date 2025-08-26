% script_test_fcn_BoundedAStar_polytopeCalculateDualPerimeters
% tests fcn_BoundedAStar_polytopeCalculateDualPerimeters

% Revision history
% 2025_08_25 - K. Hayes, kxh1031@psu.edu
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

%% DEMO case: calculate the 'gap' at a crossing point
fig_num = 10001;
titleString = sprintf('DEMO case: calculate the "gap" at a crossing point');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

% BASIC example
vertices = [2 1; 1 2; -1 2; -2 1; -2 -1; -1 -2; 1 -2; 2 -1; 2 1];
cross1 = [-2 0];
gap1 = fcn_BoundedAStar_polytopePointGapLocation(cross1,vertices,(fig_num))

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(gap1));

% Check variable values
assert(isequal(gap1, 4));

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

%% TEST case: Different crossing point
fig_num = 20001;
titleString = sprintf('TEST case: Entry and exit are on the same edge of the polytope');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

cross2 = [2 0];
gap2 = fcn_BoundedAStar_polytopePointGapLocation(cross2,vertices,fig_num)

sgtitle(titleString, 'Interpreter','none');

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(gap2));

% Check variable values
assert(isequal(gap2, 8));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Different crossing point
fig_num = 20002;
titleString = sprintf('TEST case: Entry and exit are on the same edge of the polytope');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

cross3 = [-2 0.75];
gap3 = fcn_BoundedAStar_polytopePointGapLocation(cross3,vertices,fig_num)

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(gap3));

% Check variable values
assert(isequal(gap3, 4));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),fig_num));

%% TEST case: Different crossing point
fig_num = 20003;
titleString = sprintf('TEST case: Entry and exit are on the same edge of the polytope');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

cross4 = [2 1];
gap4 = fcn_BoundedAStar_polytopePointGapLocation(cross4,vertices,fig_num)

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(gap3));

% Check variable values
assert(isequal(gap4, 1));

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






