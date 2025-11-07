% script_test_fcn_VGraph_calculatePointsOnLines

% Tests: fcn_VGraph_calculatePointsOnLines

% Revision history
% As: script_test_fcn_BoundedAStar_calculatePointsOnLines
% 2025_08_19 - K. Hayes, kaeleahayes@psu.edu
% -- first write of script
% 2025_10_22 - K. Hayes
% -- fixed bug causing assertion failures for all demo cases
%
% As: script_test_fcn_VGraph_calculatePointsOnLines
% 2025_11_06 - S. Brennan, sbrennan@psu.edu
% -- deprecated fcn_BoundedAStar_calculatePointsOnLines
%    % * Now fcn_VGraph_calculatePointsOnLines
% -- fixed minor weird usage of ind variable in plotting section
% -- added more test cases including fast test cases

% TO DO:
% -- need to simplify the inputs to point style ([X Y]) 

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

%% DEMO case: check if points are on a line
figNum = 10001;
titleString = sprintf('DEMO case: check if points are on a line');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [-2 -1 1 2 2 1 -1 -2];
y1 = [-1 -2 -2 -1 1 2 2 1];
x2 = [-1 1 2 2 1 -1 -2 -2];
y2 = [-2 -2 -1 1 2 2 1 -1];
acc = 1e-8;

xi = 2;
yi = 0;

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = 8;
assert(isequal(Nsides,length(flagIsOnLine))); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: check if points are NOT on a line
figNum = 10002;
titleString = sprintf('DEMO case: check if points are NOT on a line');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [-2 -1 1 2 2 1 -1 -2];
y1 = [-1 -2 -2 -1 1 2 2 1];
x2 = [-1 1 2 2 1 -1 -2 -2];
y2 = [-2 -2 -1 1 2 2 1 -1];
acc = 1e-8;

xi = 0;
yi = 0;

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = 8;
assert(isequal(Nsides,length(flagIsOnLine))); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));


%% DEMO case: check if points are on a line within a tolerance
figNum = 10003;
titleString = sprintf('DEMO case: check if points are on a line within a tolerance');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [-2 -1 1 2 2 1 -1 -2];
y1 = [-1 -2 -2 -1 1 2 2 1];
x2 = [-1 1 2 2 1 -1 -2 -2];
y2 = [-2 -2 -1 1 2 2 1 -1];
acc = 1e-8;

xi = 2+1e-8;
yi = 0;

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc,(figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = 8;
assert(isequal(Nsides,length(flagIsOnLine))); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: check if points are NOT on a line within a tolerance
figNum = 10004;
titleString = sprintf('DEMO case: check if points are NOT on a line within a tolerance');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [-2 -1 1 2 2 1 -1 -2];
y1 = [-1 -2 -2 -1 1 2 2 1];
x2 = [-1 1 2 2 1 -1 -2 -2];
y2 = [-2 -2 -1 1 2 2 1 -1];
acc = 1e-8;

xi = 2+2e-8;
yi = 0;
TF3 = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc,(figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(islogical(TF3));

% Check variable sizes
Nsides = 8;
assert(isequal(Nsides,length(TF3))); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: check if two points on a line
figNum = 10005;
titleString = sprintf('DEMO case: check if two points on a line');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [-2 -1 1 2 2 1 -1 -2];
y1 = [-1 -2 -2 -1 1 2 2 1];
x2 = [-1 1 2 2 1 -1 -2 -2];
y2 = [-2 -2 -1 1 2 2 1 -1];
acc = 1e-8;

xi = [2 3];
yi = [0 0];

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = length(x1);
Npoints = length(xi);
assert(size(flagIsOnLine,1)==Npoints); 
assert(size(flagIsOnLine,2)==Nsides); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: check many random points on a line segment
figNum = 10006;
titleString = sprintf('DEMO case: check many random points on a line segment');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [0 -0.5 0.25 -0.5];
y1 = [0 -0.75 -0.5 0];
x2 = [0.5 -0.5 0.75 -0.4];
y2 = [0.5 -0.25 -0.5 0.7];
acc = 0.2;

randomPoints = rand(10000,2);
xi = 2*randomPoints(:,1)' - 1;
yi = 2*randomPoints(:,2)' - 1;

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = length(x1);
Npoints = length(xi);
assert(size(flagIsOnLine,1)==Npoints); 
assert(size(flagIsOnLine,2)==Nsides); 

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

%% TEST case: no intersection due to tolerance too small
figNum = 20001;
titleString = sprintf('TEST case: no intersection due to tolerance too small');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

x1 = [-2 -1 1 2 2 1 -1 -2];
y1 = [-1 -2 -2 -1 1 2 2 1];
x2 = [-1 1 2 2 1 -1 -2 -2];
y2 = [-2 -2 -1 1 2 2 1 -1];
acc = 1e-8;

xi = -2;
yi = -1;

TF4 = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc,(figNum));

sgtitle(titleString, 'Interpreter', 'none')

% Check variable types
assert(islogical(TF4));

% Check variable sizes
Nsides = 8;
assert(isequal(Nsides,length(TF4))); 

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

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

x1 = [0 -0.5 0.25 -0.5];
y1 = [0 -0.75 -0.5 0];
x2 = [0.5 -0.5 0.75 -0.4];
y2 = [0.5 -0.25 -0.5 0.7];
acc = 0.2;

randomPoints = rand(10000,2);
xi = 2*randomPoints(:,1)' - 1;
yi = 2*randomPoints(:,2)' - 1;

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, ([]));

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = length(x1);
Npoints = length(xi);
assert(size(flagIsOnLine,1)==Npoints); 
assert(size(flagIsOnLine,2)==Nsides); 

% Check variable values
% Check this manually

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

x1 = [0 -0.5 0.25 -0.5];
y1 = [0 -0.75 -0.5 0];
x2 = [0.5 -0.5 0.75 -0.4];
y2 = [0.5 -0.25 -0.5 0.7];
acc = 0.2;

randomPoints = rand(10000,2);
xi = 2*randomPoints(:,1)' - 1;
yi = 2*randomPoints(:,2)' - 1;

flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, (-1));

% Check variable types
assert(islogical(flagIsOnLine));

% Check variable sizes
Nsides = length(x1);
Npoints = length(xi);
assert(size(flagIsOnLine,1)==Npoints); 
assert(size(flagIsOnLine,2)==Nsides); 

% Check variable values
% Check this manually

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

x1 = [0 -0.5 0.25 -0.5];
y1 = [0 -0.75 -0.5 0];
x2 = [0.5 -0.5 0.75 -0.4];
y2 = [0.5 -0.25 -0.5 0.7];
acc = 0.2;

randomPoints = rand(10000,2);
xi = 2*randomPoints(:,1)' - 1;
yi = 2*randomPoints(:,2)' - 1;

Niterations = 10;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    flagIsOnLine = fcn_VGraph_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc, (-1));
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