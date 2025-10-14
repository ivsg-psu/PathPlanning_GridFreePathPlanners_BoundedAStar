% script_fcn_BoundedAStar_generateDilationRobustnessMatrix
% Tests: fcn_fcn_BoundedAStar_generateDilationRobustnessMatrix

%
% REVISION HISTORY:
% As: script_test_fcn_algorithm_generate_dilation_robustness_matrix
% 2024_02_01 by S. Harnett
% -- first write of script
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal
% -- removed calls to fcn_algorithm_generate_dilation_robustness_matrix,
%    % replaced with fcn_BoundedAStar_generateDilationRobustnessMatrix
% -- removed calls to fcn_MapGen_fillPolytopeFieldsFromVertices,
%    % replaced with fcn_MapGen_polytopesFillFieldsFromVertices


close all;

for test_case_idx = 1:2
    if test_case_idx == 1
        % test case 1
        % % Two polytopes with clear space right down middle
        % clear polytopes
        % polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
        % polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
        % polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
        % goodAxis = [-3 7 -4 4];
        % start = [-2 -0.5];
        % finish = start;
        % finish(1) = 6;
        % selectedFromToToPlot = [1 6];
        figNum = 1;
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
        figNum = 2;
    end

    % Make sure all have same cost
    for ith_poly = 1:length(polytopes)
        polytopes(ith_poly).cost = 0.4;
    end

    % nudge = 1.05;
    % addNudge = 0.25;
    % 
    % % Plot the polytopes
    % % figNum = figNum + 1;
    % fcn_INTERNAL_plotPolytopes(polytopes, figNum)
    % axis(goodAxis);
    % title('polytope map')
    % 
    % % Plot the start and end points
    % plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
    % plot(finish(1),finish(2),'r.','MarkerSize',20);
    % text(start(:,1),start(:,2)+addNudge,'Start');
    % text(finish(:,1),finish(:,2)+addNudge,'Finish');

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
    
    start = [start size(all_pts,1)+1 -1 1]; %#ok<AGROW>
    finish = [finish size(all_pts,1)+2 -1 1]; %#ok<AGROW>

    finishes = [all_pts; start; finish];
    starts = [all_pts; start; finish];
    [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes,1);

    % % plot visibility graph edges
    % if 1==flag_do_plot_slow

        % Plot the polytopes
        % figNum = figNum + 1;
        % fcn_INTERNAL_plotPolytopes(polytopes, figNum)

        % for i = 1:size(vgraph,1)
        %     for j = 1:size(vgraph,1)
        %         if vgraph(i,j) == 1
        %             plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'g-','LineWidth',1)
        %         end
        %         if vgraph(i,j) == 0
        %             % plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--r','LineWidth',2)
        %         end
        %     end
        % end
        % title('visibility graph');

        % % Plot portion of visibility graph for selectedFromToToPlot
        % for i = selectedFromToToPlot % 1:size(vgraph,1)
        %     for j = 1:size(vgraph,1)
        %         if vgraph(i,j) == 1
        %             plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'-','Color',[0 0.5 0],'LineWidth',3)
        %         end
        %         if vgraph(i,j) == 0
        %             % plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--r','LineWidth',2)
        %         end
        %     end
        % end
        % title(sprintf('visibility graph, only from-node %.0f',selectedFromToToPlot));

    % end
    mode = '2d';

    % fcn_Visibility_plotVGraph(vgraph, [all_pts; start; finish], 'g-');

    plottingOptions.axis = goodAxis;
    plottingOptions.selectedFromToToPlot = [1 6];
    %%
    figure(figNum); clf;
    dilation_robustness_matrix = ...
        fcn_BoundedAStar_generateDilationRobustnessMatrix(...
        all_pts, start, finish, vgraph, mode, polytopes,...
        (plottingOptions), (figNum)); %#ok<NASGU>
    
    %% 
    % 
    % % Find the maximum value, not including infinity
    % % Select left or right maxima?
    % if 1==0
    %     dilation_robustness_values = dilation_robustness_matrix(:,:,left_or_right)';
    %     dilation_robustness_values = dilation_robustness_values(:)';
    %     max_dilation_robustness_excluding_inf = max(dilation_robustness_values(~isinf(dilation_robustness_values) & ~isinf(-dilation_robustness_values)));
    % else
    %     max_dilation_robustness_excluding_inf = max(dilation_robustness_matrix(~isinf(dilation_robustness_matrix)),[],"all");
    % end
    % normalizedDilationRobustnessMatrix = dilation_robustness_matrix./max_dilation_robustness_excluding_inf;

    % show the difference between measuring to the right and to the left
    % for left_or_right = [1,2]
    % 
    %     % plot corridor width approximation graph edges
    %     % Plot the polytopes
    %     % figNum = figNum + 1;
    %     % fcn_INTERNAL_plotPolytopes(polytopes, figNum)
    %     if left_or_right==1
    %         title('dilation robustness, left');
    %     else
    %         title('dilation robustness, right');
    %     end
    % 
    %     for i = 1:size(vgraph,1)
    %         for j = 1:size(vgraph,1)
    %             if vgraph(i,j) == 1
    %                 % alpha = dilation_robustness_matrix(i,j,left_or_right)/max_dilation_robustness_excluding_inf;
    %                 alpha = normalizedDilationRobustnessMatrix(i,j,left_or_right);
    %                 if alpha == inf %| alpha == -inf
    %                     continue % don't plot infinite values
    %                 end
    %                 plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'-','Color',[alpha 0 1-alpha],'LineWidth',3)
    %             end
    %         end
    %     end
    %     map = [(linspace(0,1,100))' zeros(100,1) (linspace(1,0,100))'];
    %     colormap(map)
    %     set(gca,'CLim',sort([0 1]*max_dilation_robustness_excluding_inf));
    %     c = colorbar;
    %     c.Label.String = 'dilation robustness';
    % 
    %     % plot corridor width approximation values
    %     if flag_do_plot
    %         % figNum = figNum + 1;
    %         figure(figNum); hold on; box on;
    %         for j = 1:length(polytopes)
    %              fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    %         end
    %         hold on; box on;
    %         xlabel('x [m]');
    %         ylabel('y [m]');
    %         l_or_r_string = {'left','right'};
    %         title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
    %         vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
    %         for i = 1:size(vgraph,1)
    %             for j = 1:size(vgraph,1)
    %                 % plot only start and finish for assymetry checking
    %                 if ~(i == start(3) && j == finish(3)) && ~(i == 7 && j == 8) &&  ~(i == 15 && j == 16)
    %                     continue
    %                 end
    %                 if vgraph(i,j) == 1
    %                     % plot a nice gray line
    %                     quiver(starts(i,1),starts(i,2),starts(j,1)-starts(i,1),starts(j,2)-starts(i,2), 0, '-','Color',0.4*ones(1,3),'LineWidth',2);
    %                     % label the dilation robustness
    %                     text((starts(i,1)+starts(j,1))/2 ,(starts(i,2)+starts(j,2))/2, string(dilation_robustness_matrix(i,j,left_or_right)));
    %                 end
    %             end % inner vgraph loop
    %         end % outer vgraph loop
    %     end % if do plot loop
    % 
    %     % plot corridor width approximation values
    %     if flag_do_plot
    %         % figNum = figNum + 1;
    %         figure(figNum); hold on; box on;
    %         for j = 1:length(polytopes)
    %              fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    %         end
    %         hold on; box on;
    %         xlabel('x [m]');
    %         ylabel('y [m]');
    %         l_or_r_string = {'left','right'};
    %         title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
    %         vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
    %         for i = 1:size(vgraph,1)
    %             for j = 1:size(vgraph,1)
    %                 % skip start and finish for plotting clarity
    %                 if (i == start(3) || j == finish(3) || i == finish(3) || j == start(3))
    %                     continue
    %                 end
    %                 if vgraph(i,j) == 1
    %                     % plot a nice gray line
    %                     quiver(starts(i,1),starts(i,2),starts(j,1)-starts(i,1),starts(j,2)-starts(i,2),0,'-','Color',0.4*ones(1,3),'LineWidth',2);
    %                     % label the dilation robustness
    %                     text((starts(i,1)+starts(j,1))/2 ,(starts(i,2)+starts(j,2))/2, string(dilation_robustness_matrix(i,j,left_or_right)));
    %                 end
    %             end % inner vgraph loop
    %         end % outer vgraph loop
    %     end % if do plot loop
    % end % left or right loop
end % test case loop


URHERE

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

%% DEMO case: Two polytopes with clear space right down middle, edge 1 to 6 only
figNum = 10001;
titleString = sprintf('DEMO case: Two polytopes with clear space right down middle, edge 1 to 6 only');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


% Load some test data 
%tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

% Two polytopes with clear space right down middle
clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
goodAxis = [-3 7 -4 4];
start = [-2 -0.5];
finish = start;
finish(1) = 6;

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

finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
isConcave = 1;
[vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes, isConcave,-1);
% fcn_Visibility_plotVGraph(vgraph, [all_pts; start; finish], 'g-');


% end
mode = '2d';

plottingOptions.axis = goodAxis;
plottingOptions.selectedFromToToPlot = [1 6];
plottingOptions.filename = 'dilationAnimation.gif'; % Specify the output file name

% Call the function
dilation_robustness_matrix = ...
    fcn_BoundedAStar_generateDilationRobustnessMatrix(...
    all_pts, start, finish, vgraph, mode, polytopes,...
    (plottingOptions), (figNum));


sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(dilation_robustness_matrix));

% Check variable sizes
Npoints = size(vgraph,1);
assert(size(dilation_robustness_matrix,1)==Npoints); 
assert(size(dilation_robustness_matrix,1)==Npoints); 

% Check variable values
% 1 is left, 2 is right
valueToTest = dilation_robustness_matrix(plottingOptions.selectedFromToToPlot(1), plottingOptions.selectedFromToToPlot(2),1);
roundedValueToTest = round(valueToTest,2);
assert(isequal(roundedValueToTest,0.97));
valueToTest = dilation_robustness_matrix(plottingOptions.selectedFromToToPlot(1), plottingOptions.selectedFromToToPlot(2),2);
roundedValueToTest = round(valueToTest,2);
assert(isequal(roundedValueToTest,0.97));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: Two polytopes with clear space right down middle, all edges
figNum = 10002;
titleString = sprintf('DEMO case: Two polytopes with clear space right down middle, all edges');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


% URHERE
% Bug is that points that are not on polytopes are counted as "hits". Need
% to check ONLY edges that end on polys.

% Load some test data 
%tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

% Two polytopes with clear space right down middle
clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
goodAxis = [-3 7 -4 4];
start = [-2 -0.5];
finish = start;
finish(1) = 6;

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

finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
isConcave = 1;
[vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes, isConcave,-1);
% fcn_Visibility_plotVGraph(vgraph, [all_pts; start; finish], 'g-');


% end
mode = '2d';

plottingOptions.axis = goodAxis;
plottingOptions.selectedFromToToPlot = [];
plottingOptions.filename = []; % Specify the output file name

% Call the function
dilation_robustness_matrix = ...
    fcn_BoundedAStar_generateDilationRobustnessMatrix(...
    all_pts, start, finish, vgraph, mode, polytopes,...
    (plottingOptions), (figNum));


sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(dilation_robustness_matrix));

% Check variable sizes
Npoints = size(vgraph,1);
assert(size(dilation_robustness_matrix,1)==Npoints); 
assert(size(dilation_robustness_matrix,1)==Npoints); 

% Check variable values
% Nothing to check

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

%% TEST case: Two polytopes with clear space right down middle, edge 5 to 8 on polytope
figNum = 20001;
titleString = sprintf('TEST case: Two polytopes with clear space right down middle, edge 5 to 8 on polytope');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load some test data 
%tempXYdata = fcn_INTERNAL_loadExampleData(dataSetNumber);

% Two polytopes with clear space right down middle
clear polytopes
polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
goodAxis = [-3 7 -4 4];
start = [-2 -0.5];
finish = start;
finish(1) = 6;

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

finishes = [all_pts; start; finish];
starts = [all_pts; start; finish];
isConcave = 1;
[vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes, isConcave,-1);
% fcn_Visibility_plotVGraph(vgraph, [all_pts; start; finish], 'g-');


% end
mode = '2d';

plottingOptions.axis = goodAxis;
plottingOptions.selectedFromToToPlot = [5 8];
plottingOptions.filename = 'dilationAnimation.gif'; % Specify the output file name

% Call the function
dilation_robustness_matrix = ...
    fcn_BoundedAStar_generateDilationRobustnessMatrix(...
    all_pts, start, finish, vgraph, mode, polytopes,...
    (plottingOptions), (figNum));


sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(dilation_robustness_matrix));

% Check variable sizes
Npoints = size(vgraph,1);
assert(size(dilation_robustness_matrix,1)==Npoints); 
assert(size(dilation_robustness_matrix,1)==Npoints); 

% Check variable values
% 1 is left, 2 is right
valueToTest = dilation_robustness_matrix(plottingOptions.selectedFromToToPlot(1), plottingOptions.selectedFromToToPlot(2),1);
roundedValueToTest = round(valueToTest,2);
assert(isequal(roundedValueToTest,00));
valueToTest = dilation_robustness_matrix(plottingOptions.selectedFromToToPlot(1), plottingOptions.selectedFromToToPlot(2),2);
roundedValueToTest = round(valueToTest,2);
assert(isinf(roundedValueToTest));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));



%% TEST case: This one returns nothing since there is one point in criteria
figNum = 20002;
titleString = sprintf('TEST case: This one returns nothing since there is one point in criteria');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: This one returns nothing since there is only two points in criteria
figNum = 20003;
titleString = sprintf('TEST case: This one returns nothing since there is only two points in criteria');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: returns nothing since the minimum point is at the start
% and so there is no strong minimum inside the zone
figNum = 20004;
titleString = sprintf('TEST case: returns nothing since the minimum point is at the start');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: returns nothing since the minimum point is at the end
% and so there is no strong minimum inside the zone
figNum = 20005;
titleString = sprintf('TEST case: returns nothing since the minimum point is at the end');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: returns nothing since the path doesn't return to start and no end spec given
figNum = 20006;
titleString = sprintf('TEST case: returns nothing since the path doesn''t return to start and no end spec given');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: returns nothing since the end is incomplete
figNum = 20007;
titleString = sprintf('TEST case: returns nothing since the end is incomplete');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));

%% TEST case: Show that start and end points can overlap by their boundaries
figNum = 20008;
titleString = sprintf('TEST case: Show that start and end points can overlap by their boundaries');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


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
    figNum);

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
assert(isequal(get(gcf,'Number'),figNum));


%% TEST case: show that the start and end points can be at the absolute ends
figNum = 20009;
titleString = sprintf('TEST case: show that the start and end points can be at the absolute ends');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


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
    figNum);

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
fprintf(1,'Figure: %.0f: FAST mode, empty fig_num\n',figNum);
figure(figNum); close(figNum);

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
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, fig_num=-1\n',figNum);
figure(figNum); close(figNum);

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
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

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

function INTERNAL_plot_results(tempXYdata,cell_array_of_entry_indices,cell_array_of_lap_indices,cell_array_of_exit_indices,fig_num) %#ok<DEFNU>
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