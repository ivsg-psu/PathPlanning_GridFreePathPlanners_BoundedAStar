% script_test_fcn_VGraph_polytopesGenerateAllPtsTable.m
% tests fcn_VGraph_polytopesGenerateAllPtsTable.m

% Revision history
% As: script_test_fcn_BoundedAStar_polytopesGenerateAllPtsTable
% 2025_08_05 - K. Hayes, kxh1031@psu.edu
% -- initial write of script, using script_test_fcn_BoundedAStar_Astar as
%    starter
%
% As: fcn_Visibility_polytopesGenerateAllPtsTable
% 2025_11_01 - S. Brennan
% -- renamed function 
%    % from: fcn_BoundedAStar_polytopesGenerateAllPtsTable
%    % to: fcn_Visibility_polytopesGenerateAllPtsTable
% -- redid nearly all the test cases
%
% As: fcn_VGraph_addObstacle
% 2025_11_07 - S. Brennan
% -- Renamed fcn_Visibility_polytopesGenerateAllPtsTable to fcn_VGraph_polytopesGenerateAllPtsTable

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

%% DEMO case: a polytope field
figNum = 10001;
titleString = sprintf('DEMO case: a polytope field');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Create polytope field
raw_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 25],[], ([100 100]), (-1));

% Trim polytopes on edge of boundary
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( raw_polytopes, [0.1 0.1 99.9 99.9], (-1));

% Shrink polytopes to form obstacle field
polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

% Set x and y coordinates of each polytope
startXY = [0 0];
finishXY = [80 50];

% Generate pointsWithData table
[pointsWithData, startPointData, finishPointData] = ...
    fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
    (startXY), (finishXY), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pointsWithData));
assert(isnumeric(startPointData));
assert(isnumeric(finishPointData));

% Check variable sizes
Npoly = 10;
assert(isequal(Npoly,length(polytopes)));

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

%% TEST case: One polytope
figNum = 20001;
titleString = sprintf('TEST case: One polytope');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
% polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
for ith_poly = 1:length(polytopes)
    polytopes(ith_poly).cost = 0.2;
end

startXY = [-2 -0.5];
finishXY = startXY;
finishXY(1) = 6;

% Generate pointsWithData table
[pointsWithData, startPointData, finishPointData] = ...
    fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
    (startXY), (finishXY), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pointsWithData));
assert(isnumeric(startPointData));
assert(isnumeric(finishPointData));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(pointsWithData,1)==NpolyVertices+2);
assert(size(pointsWithData,2)==5);
assert(size(startPointData,1)==1);
assert(size(startPointData,2)==5);
assert(size(finishPointData,1)==1);
assert(size(finishPointData,2)==5);

% Check variable values
% Check this manually

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: Two polytopes
figNum = 20002;
titleString = sprintf('TEST case: Two polytopes');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
for ith_poly = 1:length(polytopes)
    polytopes(ith_poly).cost = 0.2;
end

startXY = [-2 -0.5];
finishXY = startXY;
finishXY(1) = 6;

% Generate pointsWithData table
[pointsWithData, startPointData, finishPointData] = ...
    fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
    (startXY), (finishXY), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(pointsWithData));
assert(isnumeric(startPointData));
assert(isnumeric(finishPointData));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(pointsWithData,1)==NpolyVertices+2);
assert(size(pointsWithData,2)==5);
assert(size(startPointData,1)==1);
assert(size(startPointData,2)==5);
assert(size(finishPointData,1)==1);
assert(size(finishPointData,2)==5);

% Check variable values
% Check this manually

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

clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
for ith_poly = 1:length(polytopes)
    polytopes(ith_poly).cost = 0.2;
end

startXY = [-2 -0.5];
finishXY = startXY;
finishXY(1) = 6;

% Generate pointsWithData table
[pointsWithData, startPointData, finishPointData] = ...
    fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
    (startXY), (finishXY), ([]));

% Check variable types
assert(isnumeric(pointsWithData));
assert(isnumeric(startPointData));
assert(isnumeric(finishPointData));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(pointsWithData,1)==NpolyVertices+2);
assert(size(pointsWithData,2)==5);
assert(size(startPointData,1)==1);
assert(size(startPointData,2)==5);
assert(size(finishPointData,1)==1);
assert(size(finishPointData,2)==5);

% Check variable values
% Check this manually

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
for ith_poly = 1:length(polytopes)
    polytopes(ith_poly).cost = 0.2;
end

startXY = [-2 -0.5];
finishXY = startXY;
finishXY(1) = 6;

% Generate pointsWithData table
[pointsWithData, startPointData, finishPointData] = ...
    fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
    (startXY), (finishXY), (-1));

% Check variable types
assert(isnumeric(pointsWithData));
assert(isnumeric(startPointData));
assert(isnumeric(finishPointData));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(pointsWithData,1)==NpolyVertices+2);
assert(size(pointsWithData,2)==5);
assert(size(startPointData,1)==1);
assert(size(startPointData,2)==5);
assert(size(finishPointData,1)==1);
assert(size(finishPointData,2)==5);

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

clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
for ith_poly = 1:length(polytopes)
    polytopes(ith_poly).cost = 0.2;
end

startXY = [-2 -0.5];
finishXY = startXY;
finishXY(1) = 6;

Niterations = 50;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [pointsWithData, startPointData, finishPointData] = ...
        fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
        (startXY), (finishXY), ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [pointsWithData, startPointData, finishPointData] = ...
        fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, ...
        (startXY), (finishXY), ([]));
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
    %
    %% Fails because start_definition is not correct type

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


