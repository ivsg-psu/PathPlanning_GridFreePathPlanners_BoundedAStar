% script_test_polytope_canyon_corridor_width_incentive_weighting
% example of routing through a field of polytopes with a large chokepoint in the middle
% reachability and visibilty incentive cost functions can be used to route around the choke point

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
% -- removed calls to fcn_algorithm_generate_cost_graph,
%    % replaced with fcn_BoundedAStar_generateCostGraph
% -- removed calls to fcn_algorithm_generate_dilation_robustness_matrix,
%    % replaced with fcn_BoundedAStar_generateDilationRobustnessMatrix
% 2025_11_01 - S. Brennan
% -- removed calls to fcn_BoundedAStar_loadTestMap, replaced with fcn_MapGen_loadTestMap
% -- replaced fcn_BoundedAStar_generateDilationRobustnessMatrix, 
%    % with fcn_Visibility_generateDilationRobustnessMatrix
% 2025_11_02 - S. Brennan
% -- changed fcn_BoundedAStar_polytopesGenerateAllPtsTable 
%    % to fcn_Visibility_polytopesGenerateAllPtsTable
%    % WARNING: inputs/outputs to this changed slightly. Function needs to 
%    % be rechecked


% clear; close all; clc
% addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

%% plotting flags
flag_do_plot = 1;
flag_do_plot_slow = 0;

%% mission options
map_idx = 7;
[shrunk_polytopes, start_inits, finish_inits] = fcn_MapGen_loadTestMap(map_idx);

for mission_idx = 1:size(start_inits,1)
    start_init = start_inits(mission_idx,:);
    finish_init = finish_inits(mission_idx,:);
    
    %% all_pts array creation
    if 1==1
        warning('The function fcn_Visibility_polytopesGenerateAllPtsTable is not a direct replacement for the BoundedAStar version. The function needs to be updated from this point onward.')
        [all_pts, start, finish] = fcn_Visibility_polytopesGenerateAllPtsTable(shrunk_polytopes, start_init, finish_init);
    else
        % % OLD:
        %     [all_pts, start, finish] = fcn_BoundedAStar_polytopesGenerateAllPtsTable(shrunk_polytopes, start_init, finish_init);
    end

    % Plot the polytopes
    % axes_limits = [0 1 0 1]; % x and y axes limits
    % axis_style = 'square'; % plot axes style
    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fillFormat = [1 0 0 1 0.4];
    %fcn_MapGen_plotPolytopes(polytopes,fig_num,line_spec,line_width,axes_limits,axis_style);
    fcn_MapGen_plotPolytopes(shrunk_polytopes,(plotFormat),(fillFormat),(272727))
    hold on
    box on
    % axis([-0.1 1.1 -0.1 1.1]);
    xlabel('x [km]')
    ylabel('y [km]')

    %% loop over different relative cost function term weights
    for w = 0.1:0.1:1
        % make vgraph
        finishes = [all_pts; start; finish];
        starts = [all_pts; start; finish];
        [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(shrunk_polytopes, starts, finishes,1);

        % make rgraph
        [is_reachable, num_steps, rgraph] = fcn_BoundedAStar_checkReachability(vgraph,start(3),finish(3));
        if ~is_reachable
            error('initial mission, is not possible')
        end

        % make cgraph
        mode = "xy spatial only";
        [cgraph, hvec] = fcn_BoundedAStar_generateCostGraph(all_pts, start, finish, mode);

        % make dilation robustness matrix
        mode = '2d';
        dilation_robustness_tensor = fcn_Visibility_generateDilationRobustnessMatrix(all_pts, start, finish, vgraph, mode, shrunk_polytopes);
        dilation_robustness_matrix = max(dilation_robustness_tensor(:,:,1) , dilation_robustness_tensor(:,:,2)); % combine the left and right sides as a max
        dilation_robustness_matrix_for_variance = dilation_robustness_matrix(:)'; % extract vector of all values
        dilation_robustness_matrix_for_variance(dilation_robustness_matrix_for_variance == 0) = []; % remove 0s
        dilation_robustness_matrix_for_variance(isinf(dilation_robustness_matrix_for_variance)) = []; % remove infs
        variance_of_corridor_widths = var(dilation_robustness_matrix_for_variance); % find variance of corridor width/dilation robustness

        % make cost function
        inv_corridor_width = 1./dilation_robustness_matrix; % invert such that large corridors cost less
        infinite_idx = find(inv_corridor_width==inf); % find inf
        inv_corridor_width(infinite_idx) = 10000; % set "infinity" to a large value so cost is finite
        cgraph = w*cgraph + (1-w)*inv_corridor_width; % apply relative weighting to get linear combination of distance and width cost
        
        % plan route
        [init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);

        % find route length
        route_x = init_route(:,1);
        route_y = init_route(:,2);
        lengths = diff([route_x(:) route_y(:)]);
        init_route_length = sum(sqrt(sum(lengths.*lengths,2)));

        % find route choke point
        route_chokes = nan(1,size(init_route,1)-1);
        for route_idx = 1:(size(init_route,1)-1)
            % for route to route + 1...
            beg_seg = init_route(route_idx,3);
            end_seg = init_route(route_idx+1,3);
            % get the dilation robustness for this route segment
            route_seg_choke = dilation_robustness_matrix(beg_seg,end_seg);
            % append it to the route chokes, the min of these is the worst choke in the route
            route_chokes(route_idx) = route_seg_choke;
        end
        if flag_do_plot
            figure; hold on; box on;
            xlabel('x [km]');
            ylabel('y [km]');
            title_str = sprintf("length cost weighting is %.1f \n path length is %.2f [km]\n smallest corridor is %.3f [km]",w,init_route_length,min(route_chokes));
            plot(start_init(1),start_init(2),'xg','MarkerSize',6);
            plot(finish(1),finish(2),'xr','MarkerSize',6);
            plot(init_route(:,1),init_route(:,2),'k','LineWidth',2);
            for j = 1:length(shrunk_polytopes)
                 fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
            end
            title(title_str);
            leg_str = {'start','finish','route','obstacles'};
            for i = 1:length(shrunk_polytopes)-1
                leg_str{end+1} = '';
            end
        end % end flag_do_plot condition
    end % end cost function weight loop
end % end mission (i.e., start goal pair) loop

% function INTERNAL_fcn_format_timespace_plot()
%     box on
%     % define figure properties
%     opts.width      = 8;
%     opts.height     = 6;
%     opts.fontType   = 'Times';
%     opts.fontSize   = 9;
%     fig = gcf;
%     % scaling
%     fig.Units               = 'centimeters';
%     fig.Position(3)         = opts.width;
%     fig.Position(4)         = opts.height;
% 
%     % set text properties
%     set(fig.Children, ...
%         'FontName',     'Times', ...
%         'FontSize',     9);
% 
%     % remove unnecessary white space
%     set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
%     xlabel('x [m]')
%     ylabel('y [m]')
%     zlabel('t [s]')
%     view([36 30])
% end
