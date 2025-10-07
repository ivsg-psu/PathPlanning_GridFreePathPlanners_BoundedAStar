% REVISION HISTORY:
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_util_load_test_map, replaced with fcn_BoundedAStar_loadTestMap
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal
% -- removed calls to fcn_polytopes_generate_all_pts_table,
%    % replaced with fcn_BoundedAStar_polytopesGenerateAllPtsTable
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability



% clear; close all; clc
% addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));
% addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'
% addpath 'C:\Users\sjhar\Desktop\gif\gif'
% addpath 'C:\Users\sjh6473\Desktop\gif\gif'

%% plotting flags
flag_do_plot = 1;
flag_do_plot_slow= 0;
flag_do_threadpulling = 1;
flag_save_plots = 1;
navigated_portion = 0.4; % portion of initial path to be completed prior to triggering replanning

map_idx = 6 %
[shrunk_polytopes, start_inits, finish_inits,~, length_cost_weights] = fcn_BoundedAStar_loadTestMap(map_idx); % relative weighting of cost function, cost = w*length_cost + (1-w)*dilation_robustness_cost

mission_idx = 4;

w = length_cost_weights(mission_idx);
start_init = start_inits(mission_idx,:);
finish_init = finish_inits(mission_idx,:);
x_shift = -995;
y_shift = 4722;
polytope_size_increases = 0.2;
for nominal_or_width_based = [1, 2]
    trial_identifier = sprintf('map idx: %i, nominal or corridor-width-based: %i,\npolytope size increase [km]: %.2f',str2num(strcat(num2str(map_idx),num2str(mission_idx))), nominal_or_width_based,polytope_size_increases)
    %% plan the initial path
    % all_pts array creation
    [all_pts, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes, start_init, finish_init);
    % find vgraph
    finishes = [all_pts; start; finish];
    starts = [all_pts; start; finish];
    [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes,1);
    orig_vgraph = vgraph; % note the original to compare it to the reduced vgraph

    % find rgraph
    [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
    if ~is_reachable
        error('initial mission, prior to edge deletion, is not possible')
    end

    % make cgraph
    mode = "xy spatial only";
    [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);

    % make dilation robustness matrix
    mode = '2d';
    dilation_robustness_tensor = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode, shrunk_polytopes);
    dilation_robustness_matrix = min(dilation_robustness_tensor(:,:,1), dilation_robustness_tensor(:,:,2)); % combine the left and right sides as a max
    dilation_robustness_matrix_for_variance = dilation_robustness_matrix(:)'; % extract vector of all values
    dilation_robustness_matrix_for_variance(dilation_robustness_matrix_for_variance == 0) = []; % remove 0s
    dilation_robustness_matrix_for_variance(isinf(dilation_robustness_matrix_for_variance)) = []; % remove infs
    variance_of_corridor_widths = var(dilation_robustness_matrix_for_variance); % find variance of corridor width/dilation robustness
    if nominal_or_width_based == 2
        % make cost function
        inv_corridor_width = 1./dilation_robustness_matrix; % invert such that large corridors cost less
        infinite_idx = find(inv_corridor_width==inf); % find inf
        inv_corridor_width(infinite_idx) = 10000; % set "infinity" to a large value so cost is finite
        cgraph = w*cgraph + (1-w)*inv_corridor_width;
    end
    % plan initial route
    [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

    if nominal_or_width_based==2
        % backup initial route for comparison
        init_route_original = init_route;
        % find initial route length
        route_x = init_route(:,1);
        route_y = init_route(:,2);
        lengths = diff([route_x(:) route_y(:)]);
        init_route_length_original= sum(sqrt(sum(lengths.*lengths,2)));
        % create all points and start/finish for threadpulling from initial route
        all_pts_tp = init_route(2:(end-1),:);
        start_tp = init_route(1,:);
        finish_tp = init_route(end,:);
        all_pts_tp(:,3) = [1:size(all_pts_tp,1)];
        start_tp(3) = size(all_pts_tp,1) + 1;
        finish_tp(3) = size(all_pts_tp,1) + 2;
        % make vgraph again
        finishes_tp = [all_pts_tp; start_tp; finish_tp];
        starts_tp = [all_pts_tp; start_tp; finish_tp];
        [vgraph_tp, visibility_results_tp] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts_tp, finishes_tp,1);
        % make rgraph again
        [is_reachable_tp, num_steps_tp, rgraph_tp] = fcn_BoundedAStar_checkReachability(vgraph_tp,start_tp(3),finish_tp(3));
        if ~is_reachable_tp
            % we don't want to break if replanning is impossible, we want to save the data for what caused this
            error('threadpulling is impossible')
        end % end is_reachable condition for replanning
        % make cgraph again
        mode = "xy spatial only";
        [cgraph_tp, hvec_tp] = fcn_algorithm_generate_cost_graph(all_pts_tp, start_tp, finish_tp, mode);
        % replan path
        [cost_tp, route_tp] = fcn_algorithm_Astar(vgraph_tp, cgraph_tp, hvec_tp, all_pts_tp, start_tp, finish_tp);
        % overwrite route and length with threadpulled versions of these
        init_route = route_tp;
        init_cost = cost_tp;
        % find initial route length
        route_x = init_route(:,1);
        route_y = init_route(:,2);
        lengths = diff([route_x(:) route_y(:)]);
        init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
    end % end flag_do_threadpulling

    % find initial route length
    route_x = init_route(:,1);
    route_y = init_route(:,2);
    lengths = diff([route_x(:) route_y(:)]);
    init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
    % only need to calculate station distance for nominal case
    if nominal_or_width_based == 1
        nominal_init_route_length = sum(sqrt(sum(lengths.*lengths,2)));
    end

    %% find midpoint of route
    navigated_distance = nominal_init_route_length*navigated_portion; % distance along init path to place replanning point
    St_points_input = [navigated_distance 0]; % ST coordinates so station is the navigated distance and T is 0 (i.e. we're on the path)
    referencePath = init_route(:,1:2);
    flag_snap_type = 1;
    start_midway = fcn_Path_convertSt2XY(referencePath,St_points_input, flag_snap_type);

    %% enlarge polytopes
    enlarged_polytopes = fcn_MapGen_polytopesExpandEvenlyForConcave(shrunk_polytopes,polytope_size_increases);

    % enlarging polytopes may have put the midway start inside a polytope
    % for each polytope, check if this point is inside the polytope and snap to a nearby vertex if so
    pts_to_test = [start_midway; finish_init];
    output_pts = fcn_MapGen_snapInteriorPointToVertex(enlarged_polytopes, pts_to_test);
    start_midway = output_pts(1,:);
    finish_init = output_pts(2,:);

    %% plan the new path
    % generate updated all_pts array
    [all_pts_new, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(enlarged_polytopes, start_midway, finish_init);

    % make vgraph again
    finishes = [all_pts_new; start; finish];
    starts = [all_pts_new; start; finish];
    [new_vgraph, visibility_results_all_pts_new] = fcn_Visibility_clearAndBlockedPointsGlobal(enlarged_polytopes, starts, finishes,1);
    reduced_vgraph = new_vgraph;
    % get vgraph stats
    num_edges_initially = sum(sum(vgraph));
    num_edges_finally = sum(sum(new_vgraph));
    num_edges_removed = num_edges_initially - num_edges_finally;
    pct_edges_removed = (num_edges_removed)/num_edges_initially*100;
    % make rgraph again
    [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(new_vgraph,start(3),finish(3));
    if ~is_reachable
        % we don't want to break if replanning is impossible, we want to save the data for what caused this
        error('mission replanning is impossible')
    end % end is_reachable condition for replanning

    % make cgraph again
    mode = "xy spatial only";
    [cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts_new, start, finish, mode);
    % replan path
    [replan_cost, replan_route] = fcn_algorithm_Astar(new_vgraph, cgraph, hvec, all_pts_new, start, finish);

    % find replan route length
    route_x = replan_route(:,1);
    route_y = replan_route(:,2);
    lengths = diff([route_x(:) route_y(:)]);
    replan_route_length = sum(sqrt(sum(lengths.*lengths,2)));

    %% plot single trial
    % plot field, initial path, replan path, and midway point
    if flag_do_plot
        figure; hold on; box on;
        xlabel('x [km]');
        ylabel('y [km]');
        plot(start_init(1)+x_shift,start_init(2)+y_shift,'xg','MarkerSize',6);
        plot(finish(1)+x_shift,finish(2)+y_shift,'xr','MarkerSize',6);
        plot(init_route(:,1)+x_shift,init_route(:,2)+y_shift,'k','LineWidth',2);
        if flag_do_threadpulling && nominal_or_width_based==2
            plot(init_route_original(:,1)+x_shift, init_route_original(:,2)+y_shift,'--','Color',[0.5 0.5 0.5],'LineWidth',2);
        end
        plot(start_midway(1)+x_shift,start_midway(2)+y_shift,'dm','MarkerSize',6,'MarkerFaceColor','m')
        plot(replan_route(:,1)+x_shift,replan_route(:,2)+y_shift,'--g','LineWidth',2);
        for j = 1:length(enlarged_polytopes)
             fill(enlarged_polytopes(j).vertices(:,1)'+x_shift,enlarged_polytopes(j).vertices(:,2)+y_shift,[0 0 1],'FaceColor','r','FaceAlpha',0.3)
        end
        for j = 1:length(shrunk_polytopes)
             fill(shrunk_polytopes(j).vertices(:,1)'+x_shift,shrunk_polytopes(j).vertices(:,2)+y_shift,[137 207 240]./255,'FaceAlpha',1)
        end
        title_string = sprintf('map idx: %i, nominal or corridor-width-based: %i,\npolytope size increase [km]: %.2f',str2num(strcat(num2str(map_idx),num2str(mission_idx))), nominal_or_width_based,polytope_size_increases);
        title(title_string);
        leg_str = {'start','finish','initial route','replanning point','replanned route','enlarged obstacles'};
        if flag_do_threadpulling && nominal_or_width_based==2
            leg_str = {'start','finish','initial route, shortened','initial route','replanning point','replanned route','enlarged obstacles'};
        end
        for i = 1:length(shrunk_polytopes)-1
            leg_str{end+1} = '';
        end
        leg_str{end+1} = 'obstacles';
        for i = 1:length(shrunk_polytopes)-1
            leg_str{end+1} = '';
        end
        leg_str{end+1} = 'blocked initial path segment';
        legend(leg_str,'Location','northeast');
        FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
        for iFig = 1:length(FigList)
          FigHandle = FigList(iFig);
          FigName   = get(FigHandle, 'Number');
          savefig(FigHandle, strcat(num2str(FigName), '.fig'));
          saveas(FigHandle, strcat(num2str(FigName), '.png'));
        end
    end % end flag_do_plot condition

    % fcn_animate_timespace_path_plan
    %
    % Uses the gif library to plot the vehicle position, route progress, and polytope positions
    % at each time step, then creating a gif frame from this plot, and saving all plotted frames
    % as a gif for animating timespace (XYZ) routes and obstalces as a series of 2D (XY) plots.
    %
    %
    % FORMAT:
    %
    % fcn_animate_timespace_path_plan(start, finish, time_space_polytopes, route_dense, dt, xlims, ylims)
    %
    % INPUTS:
    %
    %     start: the start point vector (x,y,t)
    %     finish: the finish point vector (x,y,t)
    %     time_space_polytopes: the 3D (XYT) polytope struct array of the form generated by the function
    %         fcn_make_timespace_polyhedra_from_polygons i.e. a struct array with a verts field holding the vertices of each polytope
    %         vertices consists of 4 columns: x position, y position, time (z-axis position) and vertex id
    %         the vertex ID is necessary for correctly mapping a vertex at one time to its position at the next time
    %     route_dense: the matrix representing the interpolated route consisting of waypoints.  Each row is a
    %     waypoint, and each column is x, y, t, and point ID
    %     dt: the desired time step for interpolating the route waypoints
    %     xlims: the xlimits for plotting as a 1x2 vector consisting of the lower and upper limits
    %     ylims: the ylimits for plotting as a 1x2 vector consisting of the lower and upper limits
    %
    %
    % OUTPUTS:
    %
    %       none, but it will save a gif locally
    %
    % DEPENDENCIES:
    %    the gif library which can be found on the MATLAB file exchange here: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif
    %
    %
    % EXAMPLES:
    %
    % See the script: script_test_3d* files for examples of function calls
    % Example outputs are shown in the README for this repo
    %
    % This function was written on summer 2023 by Steve Harnett
    % Questions or comments? contact sjharnett@psu.edu

    %
    % REVISION HISTORY:
    %
    % 2023, summer by Steve Harnett
    % -- first write of function
    %
    % TO DO:

    spacing = 0.25;
    dt = spacing;
    init_route_dense = fcn_interpolate_route_spatially(init_route, spacing);
    % find closest route point to midpoint
    [~, midway_idx] = min((start_midway(1)-init_route_dense(:,1)).^2+(start_midway(2)-init_route_dense(:,2)).^2);
    init_route_dense_to_midway = init_route_dense(1:midway_idx,:);
    replan_route_dense = fcn_interpolate_route_spatially(replan_route, spacing);
    actual_route_dense = [init_route_dense_to_midway; replan_route_dense];
    num_frames = size(init_route_dense_to_midway,1) + size(replan_route_dense,1);
    num_frames_before_replan = size(init_route_dense_to_midway,1);
    polytope_size_per_frame = linspace(0, polytope_size_increases, num_frames_before_replan);
    close all; % close all figures so they aren't included as a gif frame

    % loop through all time steps
    for i = 1:num_frames
        hold on; box on;
        xlabel('x [km]')
        ylabel('y [km]')
        set(gcf,'color','white')
        % pose = [1000 458 1.5*560 1.5*420]
        % fig = gcf;
        % scaling
        % fig.Units               = 'pixels';
        % fig.Position         = pose;
        % xlim([min(all_pts(:,1))*0.95 max(all_pts(:,1))*1.05]);
        % ylim([min(all_pts(:,2))*0.95 max(all_pts(:,2))*1.05]);
        % always show start and goal
        p_start = plot(start_init(1)+x_shift,start_init(2)+y_shift,'xg','MarkerSize',6);
        p_finish = plot(finish(1)+x_shift,finish(2)+y_shift,'xr','MarkerSize',6);
        leg_str = {'start','finish'};
        % for each polytope,
        % create a fill from this poly's verts
        if i <= midway_idx
            enlarged_polytopes = fcn_MapGen_polytopesExpandEvenlyForConcave(shrunk_polytopes, polytope_size_per_frame(i));
        end
        j = 1;
        p_poly_enlarged = fill(enlarged_polytopes(j).vertices(:,1)'+x_shift,enlarged_polytopes(j).vertices(:,2)+y_shift,[0 0 1],'FaceColor','r','FaceAlpha',0.3);
        leg_str{end+1} = 'enlarged obstacles';
        for j = 2:length(enlarged_polytopes)
            p_poly_enlarged = fill(enlarged_polytopes(j).vertices(:,1)'+x_shift,enlarged_polytopes(j).vertices(:,2)+y_shift,[0 0 1],'FaceColor','r','FaceAlpha',0.3);
            leg_str{end+1} = '';
        end
        j = 1;
        p_poly = fill(shrunk_polytopes(j).vertices(:,1)'+x_shift,shrunk_polytopes(j).vertices(:,2)+y_shift,[137 207 240]./255,'FaceAlpha',1);
            leg_str{end+1} = 'obstacles';
        for j = 2:length(shrunk_polytopes)
            p_poly = fill(shrunk_polytopes(j).vertices(:,1)'+x_shift,shrunk_polytopes(j).vertices(:,2)+y_shift,[137 207 240]./255,'FaceAlpha',1);
            leg_str{end+1} = '';
        end

        cur_route_idx = i;
        % if the midpoint has not been hit, plot the midpoint as a pink diamond
        if i < midway_idx
            % plot initial route as green dotted line
            p_plan = plot(init_route_dense(:,1)+x_shift, init_route_dense(:,2)+y_shift,'g--','LineWidth',2);
            leg_str{end+1} = 'planned path';
            p_midway = plot(NaN,NaN);
            leg_str{end+1} = '';
            p_plan_old = plot(NaN,NaN);
            leg_str{end+1} = '';

        else
        % if the midpoint has been hit, plot the midpoint as a pink diamond
            % plot the initial route as a grey dotted line
            delete(p_plan)
            p_plan = plot(replan_route_dense(:,1)+x_shift, replan_route_dense(:,2)+y_shift,'g--','LineWidth',2);
            leg_str{end+1} = 'planned path';
            p_plan_old = plot(init_route_dense(:,1)+x_shift, init_route_dense(:,2)+y_shift,'--','Color','r','LineWidth',2);
            leg_str{end+1} = 'outdated plan';
            p_midway = plot(start_midway(1)+x_shift,start_midway(2)+y_shift,'dm','MarkerSize',6,'MarkerFaceColor','m');
            leg_str{end+1} = 'replanning point';
            % plot the replanned path as a green dotted line
        end
        % plot current progress as black path
        p_route = plot(actual_route_dense(1:cur_route_idx,1)+x_shift,actual_route_dense(1:cur_route_idx,2)+y_shift,'-k','LineWidth',2);
        leg_str{end+1} = 'path history';
        % also want to plot current position
        p_pose = plot(actual_route_dense(cur_route_idx,1)+x_shift,actual_route_dense(cur_route_idx,2)+y_shift,'xk','MarkerSize',6);
        leg_str{end+1} = 'current position';
        legend(leg_str)
        % first call of the gif function is different from subsequent calls
        if i == 1
            % gif('timespace_animation.gif','LoopCount',1,'DelayTime',dt/10) % notice frame duration is dt/10 to speed up animations for convenient viewing
            gif_name = sprintf('replanning_animation_%i.gif',nominal_or_width_based);
            gif(gif_name,'DelayTime',dt) % notice frame duration is dt/10 to speed up animations for convenient viewing
        else
            gif
        end
        % delete graphics objects before beginning to plot the next frame
        delete(gca)
        delete(p_route)
        delete(p_pose)
        delete(p_midway)
        delete(p_plan)
        delete(p_start)
        delete(p_finish)
    end % end gif frame loop

    %% plot last frame as a still
    i = num_frames;
    figure;
    hold on; box on;
    xlabel('x [km]')
    ylabel('y [km]')
    % xlim([min(all_pts(:,1))*0.95 max(all_pts(:,1))*1.05]);
    % ylim([min(all_pts(:,2))*0.95 max(all_pts(:,2))*1.05]);
    % always show start and goal
    p_start = plot(start_init(1)+x_shift,start_init(2)+y_shift,'xg','MarkerSize',6);
    p_finish = plot(finish(1)+x_shift,finish(2)+y_shift,'xr','MarkerSize',6);
    leg_str = {'start','finish'};
    % for each polytope,
    % create a fill from this poly's verts
    if i <= midway_idx
        enlarged_polytopes = fcn_MapGen_polytopesExpandEvenlyForConcave(shrunk_polytopes, polytope_size_per_frame(i));
    end
    j = 1;
    p_poly_enlarged = fill(enlarged_polytopes(j).vertices(:,1)'+x_shift,enlarged_polytopes(j).vertices(:,2)+y_shift,[0 0 1],'FaceColor','r','FaceAlpha',0.3);
    leg_str{end+1} = 'enlarged obstacles';
    for j = 2:length(enlarged_polytopes)
        p_poly_enlarged = fill(enlarged_polytopes(j).vertices(:,1)'+x_shift,enlarged_polytopes(j).vertices(:,2)+y_shift,[0 0 1],'FaceColor','r','FaceAlpha',0.3);
        leg_str{end+1} = '';
    end
    j = 1;
    p_poly = fill(shrunk_polytopes(j).vertices(:,1)'+x_shift,shrunk_polytopes(j).vertices(:,2)+y_shift,[137 207 240]./255,'FaceAlpha',1);
        leg_str{end+1} = 'obstacles';
    for j = 2:length(shrunk_polytopes)
        p_poly = fill(shrunk_polytopes(j).vertices(:,1)'+x_shift,shrunk_polytopes(j).vertices(:,2)+y_shift,[137 207 240]./255,'FaceAlpha',1);
        leg_str{end+1} = '';
    end

    cur_route_idx = i;
    % if the midpoint has not been hit, plot the midpoint as a pink diamond
    if i < midway_idx
        % plot initial route as green dotted line
        p_plan = plot(init_route_dense(:,1)+x_shift, init_route_dense(:,2)+y_shift,'g--','LineWidth',2);
        leg_str{end+1} = 'planned path';
        p_midway = plot(NaN,NaN);
        leg_str{end+1} = '';
        p_plan_old = plot(NaN,NaN);
        leg_str{end+1} = '';

    else
    % if the midpoint has been hit, plot the midpoint as a pink diamond
        % plot the initial route as a grey dotted line
        p_plan_old = plot(init_route_dense(:,1)+x_shift, init_route_dense(:,2)+y_shift,'--','Color','r','LineWidth',2);
        leg_str{end+1} = 'outdated path plan';
        p_midway = plot(start_midway(1)+x_shift,start_midway(2)+y_shift,'dm','MarkerSize',6,'MarkerFaceColor','m');
        leg_str{end+1} = 'replanning point';
        % plot the replanned path as a green dotted line
    end
    % plot current progress as black path
    p_route = plot(actual_route_dense(1:cur_route_idx,1)+x_shift,actual_route_dense(1:cur_route_idx,2)+y_shift,'-k','LineWidth',2);
    leg_str{end+1} = 'path';
    legend(leg_str)

    FolderName = sprintf('./replanning_still_%i',nominal_or_width_based);
    mkdir(FolderName)  % Your destination folder

    FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
    cd(FolderName)
    for iFig = 1:length(FigList)
      FigHandle = FigList(iFig);
      FigName   = get(FigHandle, 'Number');
      savefig(FigHandle, strcat(num2str(FigName), '.fig'));
      saveas(FigHandle, strcat(num2str(FigName), '.png'));
    end
    cd ..

end % end nominal or feature loop
