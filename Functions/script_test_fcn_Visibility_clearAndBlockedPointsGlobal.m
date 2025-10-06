% script_test_fcn_Visibility_clearAndBlockedPointsGlobal
% Tests: fcn_Visibility_clearAndBlockedPointsGlobal

% REVISION HISTORY:
%
% 2022_10_28 by S. Harnett
% -- first write of script
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
% 2025_08_01 - K. Hayes
% -- cleaned script formatting
% -- updated functions for compatibility with MapGen library

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

%% DEMO case: clear and blocked edges of convex polytope
figNum = 10001;
titleString = sprintf('DEMO case: clear and blocked edges of convex polytope');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% convex polytope
convex_polytope(1).vertices = [0 0; 1 1; -1 2; -2 1; -1 0; 0 0];
polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(convex_polytope);

% generate all_pts table
start = [-2.5, 1];
finish = start + [4 0];

all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start, finish,-1);

% hardcode what we expect the visibility graph to be for the convex polytope example
convex_obstacle_vgraph = [1 1 0 0 1 0 1;
                          1 1 1 0 0 0 1;
                          0 1 1 1 0 1 1;
                          0 0 1 1 1 1 0;
                          1 0 0 1 1 1 0;
                          0 0 1 1 1 1 0;
                          1 1 1 0 0 0 1];

% Calculate visibility graph
isConcave = [];
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,(isConcave),(figNum));
sgtitle(titleString, 'Interpreter','none');

assert(isequal(vgraph,concave_obstacle_vgraph_sub_optimal_result));

% Check variable types
assert(isnumeric(vgraph));
assert(isstruct(visibility_results));

% Check variable sizes
Npolys = 11;
assert(isequal(Npolys,length(polytopes))); 

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

% generate all_pts table
start = [-2.5, 1];
finish = start + [4 0];
all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start, finish,-1);
% hard code what we expect the concave obstacle vgraph to be when it is treated as a convex obstalce in vgraph calculation
concave_obstacle_vgraph_sub_optimal_result = [1 1 0 0 0 0 1 0 1;
                                              1 1 1 0 0 0 0 0 1;
                                              0 1 1 1 0 0 0 0 1;
                                              0 0 1 1 1 0 0 1 0;
                                              0 0 0 1 1 1 0 1 0;
                                              0 0 0 0 1 1 1 1 0;
                                              1 0 0 0 0 1 1 1 0;
                                              0 0 0 1 1 1 1 1 0;
                                              1 1 1 0 0 0 0 0 1];
concave_obstacle_vgraph_optimal_result = [1 1 0 0 0 0 1 0 1;
                                          1 1 1 0 0 0 0 0 1;
                                          0 1 1 1 0 0 0 0 1;
                                          0 0 1 1 1 1 0 1 0;
                                          0 0 0 1 1 1 0 1 0;
                                          0 0 0 1 1 1 1 1 0;
                                          1 0 0 0 0 1 1 1 0;
                                          0 0 0 1 1 1 1 1 0;
                                          1 1 1 0 0 0 0 0 1];

% calculate visibility graph
isConcave = 1;
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,(isConcave),(figNum));
sgtitle(titleString, 'Interpreter','none');

assert(isequal(vgraph,concave_obstacle_vgraph_sub_optimal_result));

% Check variable types
assert(isnumeric(vgraph));
assert(isstruct(visibility_results));

% Check variable sizes
Npolys = 11;
assert(isequal(Npolys,length(polytopes))); 

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
fillFormat = [1 0.5 0.5 1 0.4]; % [Y/N, R, G, B, alpha]
%fcn_MapGen_plotPolytopes(polytopes,fig_num,line_spec,line_width,axes_limits,axis_style);
fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum))
hold on
box on
axis([-0.1 1.1 -0.1 1.1]);
xlabel('x [km]')
ylabel('y [km]')

start = [-2.5, 1];
finish = start + [4 0];

% generate all_pts table
all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start, finish,-1);

% calculate visibility graph
isConcave = [];
tic
[vgraph, visibility_results] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts,(isConcave),(-1));
toc

filename = 'vGraphAnimation.gif'; % Specify the output file name
delayTime = 0.1; % Delay between frames in seconds
loopCount = Inf; % Loop indefinitely (0 for no loop)

% plot visibility graph edges
if 1==1
    Npoints = size(vgraph,1);
    for i = 1:Npoints
        goodFromIndices = zeros(Npoints,1);
        goodToIndices = zeros(Npoints,1);
        pointsToPlot = [];
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                % plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
                % pause(0.01);
                pointsToPlot = [pointsToPlot; [all_pts(i,1:2); all_pts(j,1:2); nan(1,2)]]; %#ok<AGROW>
                        
            end
        end
        plot(pointsToPlot(:,1),pointsToPlot(:,2),'-g')
        drawnow;

        if 1==0 % To save movie
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image

            % Write the frame to the GIF
            if i == 1
                % Create a new GIF file for the first frame
                imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
            else
                % Append subsequent frames
                imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end
    end


    % Plot just one result in a different color
    for i = 1:1
        goodFromIndices = zeros(Npoints,1);
        goodToIndices = zeros(Npoints,1);
        pointsToPlot = [];
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                % plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
                % pause(0.01);
                pointsToPlot = [pointsToPlot; [all_pts(i,1:2); all_pts(j,1:2); nan(1,2)]]; %#ok<AGROW>

            end
        end
        plot(pointsToPlot(:,1),pointsToPlot(:,2),'r-')
        drawnow;

        if 1==0 % To save movie
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image

            % Write the frame to the GIF
            if i == 1
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
assert(isnumeric(vgraph));
assert(isstruct(visibility_results));

% Check variable sizes
Npolys = 11;
assert(isequal(Npolys,length(polytopes))); 

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

flag_do_plot = 1;

% test zero gap case
% generate map
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset',[low_pt,high_pt]);
% shink the polytopes so that they are no longer tiled
gap_size = 0.01; % desired average maximum radius
if gap_size ~=0
    polytopes = fcn_MapGen_polytopesShrinkEvenly(polytopes,gap_size);
end

% plot the map
if flag_do_plot
    axes_limits = [0 1 0 1]; % x and y axes limits
    axis_style = 'square'; % plot axes style
    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fillFormat = [];
    %fcn_MapGen_plotPolytopes(polytopes,fig_num,line_spec,line_width,axes_limits,axis_style);
    fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum))
    hold on
    box on
    xlabel('x [km]')
    ylabel('y [km]')
end

% generate all_pts table
all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start, finish,-1);

% calculate vibility graph
tic
vgraph = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
toc
deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
% plot visibility graph edges
figure(figNum)
if flag_do_plot && gap_size ==0
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([deduped_pts(i).x,deduped_pts(j).x],[deduped_pts(i).y,deduped_pts(j).y],'--g','LineWidth',1)
            end
        end
    end
end
if flag_do_plot && gap_size ~=0
    for i = 1:size(vgraph,1)
        for j = 1:size(vgraph,1)
            if vgraph(i,j) == 1
                plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
            end
        end
    end
end

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(vgraph));
assert(isstruct(visibility_results));

% Check variable sizes
Npolys = 11;
assert(isequal(Npolys,length(polytopes))); 

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

% map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.01 0.001 1e-6 1111";
% plot_flag = 1; 
% disp_name = 0; 
% 
% line_style = 'r-';
% line_width = 2;
% 
% % Call the function
% [polytopes, h_fig] = fcn_MapGen_generatePolysFromName(map_name, plot_flag, disp_name, ([]), (line_style), (line_width));
% 
% % Check variable types
% assert(isstruct(polytopes));
% assert(isfield(polytopes,'vertices'));
% assert(isfield(polytopes,'xv'));
% assert(isfield(polytopes,'yv'));
% assert(isfield(polytopes,'distances'));
% assert(isfield(polytopes,'mean'));
% assert(isfield(polytopes,'area'));
% assert(isfield(polytopes,'max_radius'));
% assert(isfield(polytopes,'min_radius'));
% assert(isfield(polytopes,'mean_radius'));
% assert(isfield(polytopes,'radii'));
% assert(isfield(polytopes,'cost'));
% assert(isfield(polytopes,'parent_poly_id'));
% assert(isempty(h_fig));
% 
% % Check variable sizes
% Npolys = 100;
% assert(isequal(Npolys,length(polytopes))); 
% % assert(isempty((size(h_fig),[1 1]));
% 
% % Check variable values
% % assert(isequal(h_fig.Number,fig_num));
% 
% % Make sure plot did NOT open up
% figHandles = get(groot, 'Children');
% assert(~any(figHandles==fig_num));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, fig_num=-1\n',figNum);
figure(figNum); close(figNum);

% map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.01 0.001 1e-6 1111";
% plot_flag = 1; 
% disp_name = 0; 
% 
% line_style = 'r-';
% line_width = 2;
% 
% % Call the function
% [polytopes, h_fig] = fcn_MapGen_generatePolysFromName(map_name, plot_flag, disp_name, (-1), (line_style), (line_width));
% 
% % Check variable types
% assert(isstruct(polytopes));
% assert(isfield(polytopes,'vertices'));
% assert(isfield(polytopes,'xv'));
% assert(isfield(polytopes,'yv'));
% assert(isfield(polytopes,'distances'));
% assert(isfield(polytopes,'mean'));
% assert(isfield(polytopes,'area'));
% assert(isfield(polytopes,'max_radius'));
% assert(isfield(polytopes,'min_radius'));
% assert(isfield(polytopes,'mean_radius'));
% assert(isfield(polytopes,'radii'));
% assert(isfield(polytopes,'cost'));
% assert(isfield(polytopes,'parent_poly_id'));
% assert(isempty(h_fig));
% 
% % Check variable sizes
% Npolys = 100;
% assert(isequal(Npolys,length(polytopes))); 
% % assert(isempty((size(h_fig),[1 1]));
% 
% % Check variable values
% % assert(isequal(h_fig.Number,fig_num));
% 
% % Make sure plot did NOT open up
% figHandles = get(groot, 'Children');
% assert(~any(figHandles==fig_num));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

% map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.01 0.001 1e-6 1111";
% plot_flag = 1; 
% disp_name = 0; 
% 
% line_style = 'r-';
% line_width = 2;
% 
% Niterations = 10;
% 
% % Do calculation without pre-calculation
% tic;
% for ith_test = 1:Niterations
%     % Call the function
%     [polytopes, h_fig] = fcn_MapGen_generatePolysFromName(map_name, plot_flag, disp_name, ([]), (line_style), (line_width));
% end
% slow_method = toc;
% 
% % Do calculation with pre-calculation, FAST_MODE on
% tic;
% for ith_test = 1:Niterations
%     % Call the function
%     [polytopes, h_fig] = fcn_MapGen_generatePolysFromName(map_name, plot_flag, disp_name, (-1), (line_style), (line_width));
% end
% fast_method = toc;
% 
% % Make sure plot did NOT open up
% figHandles = get(groot, 'Children');
% assert(~any(figHandles==fig_num));
% 
% % Plot results as bar chart
% figure(373737);
% clf;
% hold on;
% 
% X = categorical({'Normal mode','Fast mode'});
% X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
% Y = [slow_method fast_method ]*1000/Niterations;
% bar(X,Y)
% ylabel('Execution time (Milliseconds)')
% 
% 
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
    
    % generate all_pts table
    all_pts = fcn_BoundedAStar_polytopesGenerateAllPtsTable(polytopes, start, finish,-1);
    
    % calculate vibility graph
    tic
    vgraph = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,all_pts,all_pts);
    toc
    deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
    % plot visibility graph edges
    if flag_do_plot && gap_size ==0
        for i = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(i,j) == 1
                    plot([deduped_pts(i).x,deduped_pts(j).x],[deduped_pts(i).y,deduped_pts(j).y],'--g','LineWidth',1)
                end
            end
        end
    end
    if flag_do_plot && gap_size ~=0
        for i = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(i,j) == 1
                    plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'--g','LineWidth',2)
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