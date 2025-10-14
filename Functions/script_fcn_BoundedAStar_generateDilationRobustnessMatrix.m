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

flag_do_plot = 1;
flag_do_plot_slow = 1;

line_width = 3;

for test_case_idx = 1:2
    if test_case_idx == 1
        % test case 1
        % Two polytopes with clear space right down middle
        clear polytopes
        polytopes(1).vertices = [0 0; 4,0; 4 2; 2 2.5; 0 0];
        polytopes(2).vertices = [0 -1; 4 -1; 5 -2; 3 -3; 0 -1];
        polytopes = fcn_MapGen_polytopesFillFieldsFromVertices(polytopes);
        goodAxis = [-3 7 -4 4];
        start = [-2 -0.5];
        finish = start;
        finish(1) = 6;
        selectedFromToToPlot = [1 6];
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
        (plottingOptions), (figNum));
    
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


