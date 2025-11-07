% script_test_fcn_VGraph_clearAndBlockedPointsGlobal
% Tests: fcn_VGraph_clearAndBlockedPointsGlobal

% REVISION HISTORY:
% As: ????
% 2022_10_28 by S. Harnett
% -- first write of script
% 2025_07_08 - K. Hayes, kaeleahayes@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method
% 2025_08_01 - K. Hayes
% -- cleaned script formatting
% -- updated functions for compatibility with MapGen library
% 2025_10_03 - K. Hayes
% -- fixed bug causing Npoly assertion failures in DEMO cases
% -- fixed bug with missing variables in DEMO case 3
% -- fixed bug with missing variables in TEST case 1
% 2025_10_07 - S. Brennan
% -- replaced fcn_MapGen_haltonVoronoiTiling call
%    % with fcn_MapGen_generatePolysFromSeedGeneratorNames
% -- added example plotting of visibility for 2025 MECC paper
% 2025_11_01 - S. Brennan
% -- staged script to move out of BoundedAStar and into Visibility Graph
% -- matched script's variable names to those inside the function, for
%    % clarity
%
% As: script_test_fcn_Visibility_clearAndBlockedPointsGlobal
% 2025_11_02 - S. Brennan
% -- changed fcn_BoundedAStar_polytopesGenerateAllPtsTable 
%    % to fcn_Visibility_polytopesGenerateAllPtsTable
%
% As: script_test_fcn_VGraph_clearAndBlockedPointsGlobal
% 2025_11_07 - S. Brennan
% -- Renamed script_test_fcn_Visibility_clearAndBlockedPointsGlobal to script_test_fcn_VGraph_clearAndBlockedPointsGlobal

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

%% DEMO case: clear and blocked edges of single convex polytope
figNum = 10001;
titleString = sprintf('DEMO case: clear and blocked edges of single convex polytope');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-2.5, 1];
finishXY = startXY + [4 0];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% Calculate visibility graph
isConcave = [];
[visibilityMatrix, visibilityDetailsEachFromPoint] = fcn_VGraph_clearAndBlockedPointsGlobal(polytopes,pointsWithData,pointsWithData,(isConcave),(figNum));
sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(visibilityMatrix,1)==NpolyVertices+2);
assert(size(visibilityMatrix,2)==NpolyVertices+2);
assert(size(visibilityDetailsEachFromPoint,2)==NpolyVertices+2);

% Check variable values
% Check manually

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: clear and blocked edges of two convex polytopes
figNum = 10002;
titleString = sprintf('DEMO case: clear and blocked edges of two convex polytopes');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
convex_polytope(2).vertices = [1.5 1.5; 2 0.5; 3 3; 1.5 2; 1.5 1.5];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-2.5, 1];
finishXY = [4 1];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% Calculate visibility graph
isConcave = [];
[visibilityMatrix, visibilityDetailsEachFromPoint] = fcn_VGraph_clearAndBlockedPointsGlobal(polytopes,pointsWithData,pointsWithData,(isConcave),(figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(visibilityMatrix,1)==NpolyVertices+2);
assert(size(visibilityMatrix,2)==NpolyVertices+2);
assert(size(visibilityDetailsEachFromPoint,2)==NpolyVertices+2);

% Check variable values
% Check this manually

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: clear and blocked edges of concave polytope
figNum = 10002;
titleString = sprintf('DEMO case: clear and blocked edges of concave polytope');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% nonconvex polytope
concave_polytope(1).vertices = [0 0; 1 1; 0.5, 2.5; -2, 2.5; -1 2; -2 1; -1 0; 0 0];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(concave_polytope,1);

% generate pointsWithData table
startXY = [-2.5, 1];
finishXY = startXY + [4 0];
pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% calculate visibility graph
isConcave = 1;
[visibilityMatrix, visibilityDetailsEachFromPoint] = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopes,pointsWithData,pointsWithData,(isConcave),(figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(visibilityMatrix,1)==NpolyVertices+2);
assert(size(visibilityMatrix,2)==NpolyVertices+2);
assert(size(visibilityDetailsEachFromPoint,2)==NpolyVertices+2);

% Check variable values
% Check this manually

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: compute a complex visibility graph
figNum = 10003;
titleString = sprintf('DEMO case: compute a complex visibility graph');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% generate map
Halton_seed = 10;
low_pt = 1+Halton_seed; high_pt = 11+Halton_seed; % range of Halton points to use to generate the tiling
trim_polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset',[low_pt,high_pt]);

% shink the polytopes so that they are no longer tiled
gap_size = 0.05; % desired average maximum radius
polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes,gap_size);

% plot the map

% axes_limits = [0 1 0 1]; % x and y axes limits
% axis_style = 'square'; % plot axes style
plotFormat.Color = 'Blue'; % edge line plotting
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 2; % linewidth of the edge
fillFormat = [1 0 0 1 0.4];
%fcn_MapGen_plotPolytopes(polytopes,figNum,line_spec,line_width,axes_limits,axis_style);
fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum));
hold on
box on
axis([-0.1 1.1 -0.1 1.1]);
xlabel('x [km]')
ylabel('y [km]')


% generate pointsWithData table
startXY = [0 0.5];
finishXY = [1 0.5];
pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);


% calculate visibility graph
isConcave = [];
[visibilityMatrix, visibilityDetailsEachFromPoint] = fcn_VGraph_clearAndBlockedPointsGlobal(polytopes,pointsWithData,pointsWithData,(isConcave),(-1));


filename = 'vGraphAnimation.gif'; % Specify the output file name
delayTime = 0.1; % Delay between frames in seconds
loopCount = Inf; % Loop indefinitely (0 for no loop)

% plot visibility graph edges
if 1==1

    fcn_VGraph_plotVGraph(visibilityMatrix, pointsWithData, 'g-', []);

    % Npoints = size(visibilityMatrix,1);
    % for ith_fromIndex = 1:Npoints
    %     pointsToPlot = [];
    %     for jth_toIndex = 1:size(visibilityMatrix,1)
    %         if visibilityMatrix(ith_fromIndex,jth_toIndex) == 1
    %             % plot([pointsWithData(i,1),pointsWithData(j,1)],[pointsWithData(i,2),pointsWithData(j,2)],'-g')
    %             % pause(0.01);
    %             pointsToPlot = [pointsToPlot; [pointsWithData(ith_fromIndex,1:2); pointsWithData(jth_toIndex,1:2); nan(1,2)]]; %#ok<AGROW>
    %
    %         end
    %     end
    %     plot(pointsToPlot(:,1),pointsToPlot(:,2),'-g')
    %     drawnow;
    %
    %     if 1==0 % To save movie
    %         % Capture the current frame
    %         frame = getframe(gcf);
    %         im = frame2im(frame);
    %         [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
    %
    %         % Write the frame to the GIF
    %         if ith_fromIndex == 1
    %             % Create a new GIF file for the first frame
    %             imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
    %         else
    %             % Append subsequent frames
    %             imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    %         end
    %     end
    % end


    % Plot just one result in a different color
    for ith_fromIndex = 1:1
        pointsToPlot = [];
        for jth_toIndex = 1:size(visibilityMatrix,1)
            if visibilityMatrix(ith_fromIndex,jth_toIndex) == 1
                % plot([pointsWithData(i,1),pointsWithData(j,1)],[pointsWithData(i,2),pointsWithData(j,2)],'-g')
                % pause(0.01);
                pointsToPlot = [pointsToPlot; [pointsWithData(ith_fromIndex,1:2); pointsWithData(jth_toIndex,1:2); nan(1,2)]]; %#ok<AGROW>

            end
        end
        plot(pointsToPlot(:,1),pointsToPlot(:,2),'r-','LineWidth',2)
        drawnow;

        if 1==0 % To save movie
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image

            % Write the frame to the GIF
            if ith_fromIndex == 1
                % Create a new GIF file for the first frame
                imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
            else
                % Append subsequent frames
                imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end
    end
end

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(visibilityMatrix,1)==NpolyVertices+2);
assert(size(visibilityMatrix,2)==NpolyVertices+2);
assert(size(visibilityDetailsEachFromPoint,2)==NpolyVertices+2);

% Check variable values
% Check this manually

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

%% TEST case: zero gap between polytopes, make sure visibility is same as very small gap
figNum = 20001;
titleString = sprintf('TEST case: zero gap between polytopes, make sure visibility is same as very small gap');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% convex polytope with no gaps
convex_polytope(1).vertices = [0 0; 1 0; 1 1; 0 1; 0 0];
convex_polytope(2).vertices = convex_polytope(1).vertices + ones(5,1)*[1 0];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-1, 0.5];
finishXY = [2.5 0.5];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% Calculate visibility graph
isConcave = [];
visibilityMatrixNoGaps = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(figNum*100));

% convex polytope with gaps
coreSquare = [0 0; 1 0; 1 1; 0 1; 0 0];
convex_polytope(1).vertices = coreSquare*0.95;
convex_polytope(2).vertices = coreSquare*0.95 + ones(5,1)*[1 0];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-1, 0.5];
finishXY = [2.5 0.5];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% Calculate visibility graph
isConcave = [];
visibilityMatrixWithGaps = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check equivalency
assert(~isequal(visibilityMatrixNoGaps, visibilityMatrixWithGaps))

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: zero gap between polytopes, make sure visibility is same as very small gap
figNum = 20002;
titleString = sprintf('TEST case: zero gap between polytopes, make sure visibility is same as very small gap');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


flag_do_plot = 1;

% test zero gap case
% generate map
Halton_seed = 10;
low_pt = 1+Halton_seed; high_pt = 11+Halton_seed; % range of Halton points to use to generate the tiling
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset',[low_pt,high_pt]);
% shink the polytopes so that they are no longer tiled
gap_size = 0.01; % desired average maximum radius
if gap_size ~=0
    polytopesWithGap = fcn_MapGen_polytopesShrinkEvenly(polytopes,gap_size);
end

% plot the maps both with and without gaps
if flag_do_plot
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style

    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fillFormat = [];
    fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum));

    hold on
    box on
    axis square
    axis(axes_limits);
    xlabel('x [km]')
    ylabel('y [km]')

    plotFormat.Color = [1 0 0]; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 1; % linewidth of the edge
    fillFormat = [];
    fcn_MapGen_plotPolytopes(polytopesWithGap,(plotFormat),(fillFormat),(figNum));

end

% generate pointsWithData table for both cases
startXY = [-2.5, 1];
finishXY = startXY + [4 0];
pointsWithDataNoGaps   = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);
pointsWithDataWithGaps = fcn_VGraph_polytopesGenerateAllPtsTable(polytopesWithGap, startXY, finishXY,-1);

% calculate visibility graphs
visibilityMatrixNoGaps = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopes,pointsWithDataNoGaps,pointsWithDataNoGaps);
visibilityMatrixWithGaps = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopesWithGap,pointsWithDataWithGaps,pointsWithDataWithGaps);

deduped_pts = fcn_VGraph_convertPolytopetoDedupedPoints(pointsWithDataNoGaps);


% deduped_pts = fcn_convert_polytope_struct_to_deduped_points(pointsWithDataNoGaps);
% % plot visibility graph edges
% figure(figNum)
% if flag_do_plot && gap_size ==0
%     for ith_fromIndex = 1:size(visibilityMatrix,1)
%         for jth_toIndex = 1:size(visibilityMatrix,1)
%             if visibilityMatrix(ith_fromIndex,jth_toIndex) == 1
%                 plot([deduped_pts(ith_fromIndex).x,deduped_pts(jth_toIndex).x],[deduped_pts(ith_fromIndex).y,deduped_pts(jth_toIndex).y],'--g','LineWidth',1)
%             end
%         end
%     end
% end
% if flag_do_plot && gap_size ~=0
%     for ith_fromIndex = 1:size(visibilityMatrix,1)
%         for jth_toIndex = 1:size(visibilityMatrix,1)
%             if visibilityMatrix(ith_fromIndex,jth_toIndex) == 1
%                 plot([pointsWithData(ith_fromIndex,1),pointsWithData(jth_toIndex,1)],[pointsWithData(ith_fromIndex,2),pointsWithData(jth_toIndex,2)],'--g','LineWidth',2)
%             end
%         end
%     end
% end

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));


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

% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
convex_polytope(2).vertices = [1.5 1.5; 2 0.5; 3 3; 1.5 2; 1.5 1.5];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-2.5, 1];
finishXY = [4 1];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% Calculate visibility graph
isConcave = [];
[visibilityMatrix, visibilityDetailsEachFromPoint] = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),([]));

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(visibilityMatrix,1)==NpolyVertices+2);
assert(size(visibilityMatrix,2)==NpolyVertices+2);
assert(size(visibilityDetailsEachFromPoint,2)==NpolyVertices+2);

% Check variable values
% Check this manually

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
convex_polytope(2).vertices = [1.5 1.5; 2 0.5; 3 3; 1.5 2; 1.5 1.5];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-2.5, 1];
finishXY = [4 1];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

% Calculate visibility graph
isConcave = [];
[visibilityMatrix, visibilityDetailsEachFromPoint] = ...
    fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(-1));

% Check variable types
assert(isnumeric(visibilityMatrix));
assert(isstruct(visibilityDetailsEachFromPoint));

% Check variable sizes
NpolyVertices = length([polytopes.xv]);
assert(size(visibilityMatrix,1)==NpolyVertices+2);
assert(size(visibilityMatrix,2)==NpolyVertices+2);
assert(size(visibilityDetailsEachFromPoint,2)==NpolyVertices+2);

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

% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
convex_polytope(2).vertices = [1.5 1.5; 2 0.5; 3 3; 1.5 2; 1.5 1.5];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate pointsWithData table
startXY = [-2.5, 1];
finishXY = [4 1];
isConcave = [];

pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

Niterations = 10;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [visibilityMatrix, visibilityDetailsEachFromPoint] = ...
        fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [visibilityMatrix, visibilityDetailsEachFromPoint] = ...
        fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, (isConcave),(-1));
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


    % broken test for 0 gap size special case that is not implemented
    % generate map
    polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset',[low_pt,high_pt]);
    % shink the polytopes so that they are no longer tiled
    gap_size = 0; % desired average maximum radius
    if gap_size ~=0
        polytopes = fcn_MapGen_polytopesShrinkFromEdges(polytopes,gap_size);
    end

    % plot the map
    if flag_do_plot
        fig = 299; % figure to plot on
        line_spec = 'b-'; % edge line plotting
        line_width = 2; % linewidth of the edge
        axes_limits = [0 1 0 1]; % x and y axes limits
        axis_style = 'square'; % plot axes style
        fcn_MapGen_plotPolytopes(polytopes,fig,line_spec,line_width,axes_limits,axis_style);
        hold on
        box on
        xlabel('x [km]')
        ylabel('y [km]')
    end

    % generate pointsWithData table
    pointsWithData = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes, startXY, finishXY,-1);

    % calculate vibility graph
    tic;
    visibilityMatrix = fcn_VGraph_clearAndBlockedPointsGlobal(polytopes,pointsWithData,pointsWithData);
    toc;
    deduped_pts = fcn_convert_polytope_struct_to_deduped_points(pointsWithData);
    % plot visibility graph edges
    if flag_do_plot && gap_size ==0
        for ith_fromIndex = 1:size(visibilityMatrix,1)
            for jth_toIndex = 1:size(visibilityMatrix,1)
                if visibilityMatrix(ith_fromIndex,jth_toIndex) == 1
                    plot([deduped_pts(ith_fromIndex).x,deduped_pts(jth_toIndex).x],[deduped_pts(ith_fromIndex).y,deduped_pts(jth_toIndex).y],'--g','LineWidth',1)
                end
            end
        end
    end
    if flag_do_plot && gap_size ~=0
        for ith_fromIndex = 1:size(visibilityMatrix,1)
            for jth_toIndex = 1:size(visibilityMatrix,1)
                if visibilityMatrix(ith_fromIndex,jth_toIndex) == 1
                    plot([pointsWithData(ith_fromIndex,1),pointsWithData(jth_toIndex,1)],[pointsWithData(ith_fromIndex,2),pointsWithData(jth_toIndex,2)],'--g','LineWidth',2)
                end
            end
        end
    end

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