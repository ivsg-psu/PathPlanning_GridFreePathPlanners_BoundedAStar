% script_test_fcn_VGraph_addObstacle

% a basic test of adding obstacles to existing visibility graphs

% Revision history
% As: fcn_Visibility_addObstacle
% 2025_08_01 - K. Hayes, kxh1031@psu.edu
% -- first write of script, using
% script_test_fcn_Visibility_clearAndBlockedPoints as a starter
% 2025_08_04 - K. Hayes
% -- moved plotting into fcn_Visibility_addObstacle debug
%
% As: fcn_VGraph_addObstacle
% 2025_11_07 - S. Brennan
% -- Renamed fcn_Visibility_addObstacle to fcn_VGraph_addObstacle

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

%% DEMO case: add a polytope to the map
figNum = 10001;
titleString = sprintf('DEMO case: add a polytope to the map');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

dataFileName = 'DATA_fcn_VGraph_addObstacle_polytopeMapForTesting.mat';
fullDataFileWithPath = fullfile(pwd,'Data',dataFileName);
if exist(fullDataFileWithPath, 'file')
    load(fullDataFileWithPath,'polytopes');
else
    % Create polytope field
    raw_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 25],[], ([100 100]), (-1));

    % Trim polytopes on edge of boundary
    trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( raw_polytopes, [0.1 0.1 99.9 99.9], (-1));

    % Shrink polytopes to form obstacle field
    polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

    % Set costs to uniform values
    for ith_poly = 1:length(polytopes)
        polytopes(ith_poly).cost = 0.4;
    end

    save(fullDataFileWithPath,'polytopes');

end

% Create pointsWithData matrix
startXY = [0, 50];
finishXY = [100, 50];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY, -1);

startPointData = pointsWithData(end-1,:);
finishPointData = pointsWithData(end,:);

% Create visibility graph
isConcave = [];
visibilityMatrix =fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(-1));

% add a polytope
polytopeToAdd = polytopes(1);
polytopeToAdd.xv = 0.5*polytopeToAdd.xv + 55;
polytopeToAdd.yv = 0.5*polytopeToAdd.yv - 10;
polytopeToAdd.vertices = [polytopeToAdd.xv' polytopeToAdd.yv'];

% Update visibilityMatrix with new polytope added
[newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
    fcn_VGraph_addObstacle(...
    visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(newVisibilityMatrix));
assert(isnumeric(newPointsWithData));
assert(isnumeric(newStartPointData));
assert(isnumeric(newFinishPointData));
assert(isstruct(newPolytopes));

% Check variable sizes
NpointsOriginal= length(pointsWithData(:,1));
NpointsAdded = length(polytopeToAdd.vertices(:,1));
NpointsNew = NpointsOriginal + NpointsAdded;
assert(size(newVisibilityMatrix,1)==NpointsNew); 
assert(size(newVisibilityMatrix,2)==NpointsNew); 
assert(size(newPointsWithData,1)==NpointsNew); 
assert(size(newPointsWithData,2)==5); 
assert(size(newStartPointData,1)==1); 
assert(size(newStartPointData,2)==5); 
assert(size(newFinishPointData,1)==1); 
assert(size(newFinishPointData,2)==5); 
assert(size(newPolytopes,1)==1); 
assert(size(newPolytopes,2)==size(polytopes,2)+1); 

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

dataFileName = 'DATA_fcn_VGraph_addObstacle_polytopeMapForTesting.mat';
fullDataFileWithPath = fullfile(pwd,'Data',dataFileName);
if exist(fullDataFileWithPath, 'file')
    load(fullDataFileWithPath,'polytopes');
else
    % Create polytope field
    raw_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 25],[], ([100 100]), (-1));

    % Trim polytopes on edge of boundary
    trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( raw_polytopes, [0.1 0.1 99.9 99.9], (-1));

    % Shrink polytopes to form obstacle field
    polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

    % Set costs to uniform values
    for ith_poly = 1:length(polytopes)
        polytopes(ith_poly).cost = 0.4;
    end

    save(fullDataFileWithPath,'polytopes');

end

% Create pointsWithData matrix
startXY = [0, 50];
finishXY = [100, 50];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY, -1);

startPointData = pointsWithData(end-1,:);
finishPointData = pointsWithData(end,:);

% Create visibility graph
isConcave = [];
visibilityMatrix =fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(-1));

% add a polytope
polytopeToAdd = polytopes(1);
polytopeToAdd.xv = 0.5*polytopeToAdd.xv + 55;
polytopeToAdd.yv = 0.5*polytopeToAdd.yv - 10;
polytopeToAdd.vertices = [polytopeToAdd.xv' polytopeToAdd.yv'];

% Update visibilityMatrix with new polytope added
[newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
    fcn_VGraph_addObstacle(...
    visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, ([]));

% Check variable types
assert(isnumeric(newVisibilityMatrix));
assert(isnumeric(newPointsWithData));
assert(isnumeric(newStartPointData));
assert(isnumeric(newFinishPointData));
assert(isstruct(newPolytopes));

% Check variable sizes
NpointsOriginal= length(pointsWithData(:,1));
NpointsAdded = length(polytopeToAdd.vertices(:,1));
NpointsNew = NpointsOriginal + NpointsAdded;
assert(size(newVisibilityMatrix,1)==NpointsNew); 
assert(size(newVisibilityMatrix,2)==NpointsNew); 
assert(size(newPointsWithData,1)==NpointsNew); 
assert(size(newPointsWithData,2)==5); 
assert(size(newStartPointData,1)==1); 
assert(size(newStartPointData,2)==5); 
assert(size(newFinishPointData,1)==1); 
assert(size(newFinishPointData,2)==5); 
assert(size(newPolytopes,1)==1); 
assert(size(newPolytopes,2)==size(polytopes,2)+1); 

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);


dataFileName = 'DATA_fcn_VGraph_addObstacle_polytopeMapForTesting.mat';
fullDataFileWithPath = fullfile(pwd,'Data',dataFileName);
if exist(fullDataFileWithPath, 'file')
    load(fullDataFileWithPath,'polytopes');
else
    % Create polytope field
    raw_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 25],[], ([100 100]), (-1));

    % Trim polytopes on edge of boundary
    trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( raw_polytopes, [0.1 0.1 99.9 99.9], (-1));

    % Shrink polytopes to form obstacle field
    polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

    % Set costs to uniform values
    for ith_poly = 1:length(polytopes)
        polytopes(ith_poly).cost = 0.4;
    end

    save(fullDataFileWithPath,'polytopes');

end

% Create pointsWithData matrix
startXY = [0, 50];
finishXY = [100, 50];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY, -1);

startPointData = pointsWithData(end-1,:);
finishPointData = pointsWithData(end,:);

% Create visibility graph
isConcave = [];
visibilityMatrix =fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(-1));

% add a polytope
polytopeToAdd = polytopes(1);
polytopeToAdd.xv = 0.5*polytopeToAdd.xv + 55;
polytopeToAdd.yv = 0.5*polytopeToAdd.yv - 10;
polytopeToAdd.vertices = [polytopeToAdd.xv' polytopeToAdd.yv'];

% Update visibilityMatrix with new polytope added
[newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
    fcn_VGraph_addObstacle(...
    visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, (-1));

% Check variable types
assert(isnumeric(newVisibilityMatrix));
assert(isnumeric(newPointsWithData));
assert(isnumeric(newStartPointData));
assert(isnumeric(newFinishPointData));
assert(isstruct(newPolytopes));

% Check variable sizes
NpointsOriginal= length(pointsWithData(:,1));
NpointsAdded = length(polytopeToAdd.vertices(:,1));
NpointsNew = NpointsOriginal + NpointsAdded;
assert(size(newVisibilityMatrix,1)==NpointsNew); 
assert(size(newVisibilityMatrix,2)==NpointsNew); 
assert(size(newPointsWithData,1)==NpointsNew); 
assert(size(newPointsWithData,2)==5); 
assert(size(newStartPointData,1)==1); 
assert(size(newStartPointData,2)==5); 
assert(size(newFinishPointData,1)==1); 
assert(size(newFinishPointData,2)==5); 
assert(size(newPolytopes,1)==1); 
assert(size(newPolytopes,2)==size(polytopes,2)+1); 

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);


dataFileName = 'DATA_fcn_VGraph_addObstacle_polytopeMapForTesting.mat';
fullDataFileWithPath = fullfile(pwd,'Data',dataFileName);
if exist(fullDataFileWithPath, 'file')
    load(fullDataFileWithPath,'polytopes');
else
    % Create polytope field
    raw_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 25],[], ([100 100]), (-1));

    % Trim polytopes on edge of boundary
    trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( raw_polytopes, [0.1 0.1 99.9 99.9], (-1));

    % Shrink polytopes to form obstacle field
    polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

    % Set costs to uniform values
    for ith_poly = 1:length(polytopes)
        polytopes(ith_poly).cost = 0.4;
    end

    save(fullDataFileWithPath,'polytopes');

end

% Create pointsWithData matrix
startXY = [0, 50];
finishXY = [100, 50];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY, -1);

startPointData = pointsWithData(end-1,:);
finishPointData = pointsWithData(end,:);

% Create visibility graph
isConcave = [];
visibilityMatrix =fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(-1));

% add a polytope
polytopeToAdd = polytopes(1);
polytopeToAdd.xv = 0.5*polytopeToAdd.xv + 55;
polytopeToAdd.yv = 0.5*polytopeToAdd.yv - 10;
polytopeToAdd.vertices = [polytopeToAdd.xv' polytopeToAdd.yv'];

Niterations = 3;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Update visibilityMatrix with new polytope added
    [newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
        fcn_VGraph_addObstacle(...
        visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Update visibilityMatrix with new polytope added
    [newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
        fcn_VGraph_addObstacle(...
        visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, (-1));
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