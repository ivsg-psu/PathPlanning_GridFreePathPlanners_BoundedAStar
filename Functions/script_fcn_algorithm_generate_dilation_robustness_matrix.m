% script_fcn_algorithm_generate_dilation_robustness_matrix
% Tests: fcn_fcn_algorithm_generate_dilation_robustness_matrix

%
% REVISION HISTORY:
%
% 2024_02_01 by S. Harnett
% -- first write of script
% 2025_10_06 - S. Brennan
% -- removed addpath calls
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal

% addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
% addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_plot_slow = 0;

fig_num = 2;
line_width = 3;

for test_case_idx = 1:2
    if test_case_idx == 1
        %% test case 1
        clear polytopes
        polytopes(1).vertices = [0 0; 4,0; 4 2; 2 4; 0 0];
        polytopes(2).vertices = [0 -1; 4, -1; 5 -2; 3 -10; 0 -1];
        polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes);
        start = ones(1,2)*(-0.5);
        finish = start;
        finish(1) = 6;
    end
    if test_case_idx == 2
        %% test case 2
        clear polytopes
        polytopes(1).vertices = [0 0; 10 0; 10 1; 0 1; 0 0];
        polytopes(2).vertices = polytopes(1).vertices+[0,2];
        polytopes(3).vertices = polytopes(1).vertices+[0,5];
        polytopes(4).vertices = polytopes(1).vertices+[0,10];
        polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(polytopes);
        start = [0,9];
        finish = start + [10,0];
    end
    fig_num = fig_num + 1;
    fcn_MapGen_plotPolytopes(polytopes,fig_num,'b-',line_width);
    hold on; box on;
    xlabel('x [m]');
    ylabel('y [m]');
    title('polytope map')


    plot(start(1),start(2),'xg','MarkerSize',6);
    plot(finish(1),finish(2),'xr','MarkerSize',6);

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

    % label point ids for debugging
    plot(all_pts(:,1), all_pts(:,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
    nudge = 1.05;
    text(all_pts(:,1)*nudge,all_pts(:,2)*nudge,string(all_pts(:,3)));

    finishes = [all_pts; start; finish];
    starts = [all_pts; start; finish];
    [vgraph, visibility_results_all_pts] = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes, starts, finishes,1);
    % plot visibility graph edges
    if flag_do_plot_slow
        fig_num = fig_num + 1;
        fcn_MapGen_plotPolytopes(polytopes,fig_num,'b-',line_width);
        hold on; box on;
        xlabel('x [m]');
        ylabel('y [m]');
        title('visibility graph');
        for i = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(i,j) == 1
                    plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--g','LineWidth',2)
                end
                if vgraph(i,j) == 0
                    plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--r','LineWidth',2)
                end
            end
        end
    end
    mode = '2d';
    dilation_robustness_matrix = fcn_algorithm_generate_dilation_robustness_matrix(all_pts, start, finish, vgraph, mode, polytopes);

    for left_or_right = [1,2]
        % show the difference between measuring to the right and to the left
        dilation_robustness_values = dilation_robustness_matrix(:,:,left_or_right)';
        dilation_robustness_values = dilation_robustness_values(:)';
        max_dilation_robustness_excluding_inf = max(dilation_robustness_values(~isinf(dilation_robustness_values) & ~isinf(-dilation_robustness_values)));

        % plot corridor width approximation graph edges
        if flag_do_plot
            fig_num = fig_num + 1;
            fcn_MapGen_plotPolytopes(polytopes,fig_num,'g-',line_width);
            hold on; box on;
            xlabel('x [m]');
            ylabel('y [m]');
            title('dilation robustness');
            for i = 1:size(vgraph,1)
                for j = 1:size(vgraph,1)
                    if vgraph(i,j) == 1
                        alpha = dilation_robustness_matrix(i,j,left_or_right)/max_dilation_robustness_excluding_inf;
                        if alpha == inf %| alpha == -inf
                            continue % don't plot infinite values
                        end
                        plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--','Color',[alpha 0 1-alpha],'LineWidth',2)
                    end
                end
            end
            map = [(linspace(0,1,100))' zeros(100,1) (linspace(1,0,100))'];
            colormap(map)
            set(gca,'CLim',sort([0 1]*max_dilation_robustness_excluding_inf));
            c = colorbar;
            c.Label.String = 'dilation robustness';
        end

        % plot corridor width approximation values
        if flag_do_plot
            fig_num = fig_num + 1;
            figure(fig_num); hold on; box on;
            for j = 1:length(polytopes)
                 fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
            end
            hold on; box on;
            xlabel('x [m]');
            ylabel('y [m]');
            l_or_r_string = {'left','right'};
            title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
            vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
            for i = 1:size(vgraph,1)
                for j = 1:size(vgraph,1)
                    % plot only start and finish for assymetry checking
                    if ~(i == start(3) && j == finish(3)) && ~(i == 7 && j == 8) &&  ~(i == 15 && j == 16)
                        continue
                    end
                    if vgraph(i,j) == 1
                        % plot a nice gray line
                        quiver(starts(i,1),starts(i,2),starts(j,1)-starts(i,1),starts(j,2)-starts(i,2),'--','Color',0.4*ones(1,3),'LineWidth',2);
                        % label the dilation robustness
                        text((starts(i,1)+starts(j,1))/2 ,(starts(i,2)+starts(j,2))/2, string(dilation_robustness_matrix(i,j,left_or_right)));
                    end
                end % inner vgraph loop
            end % outer vgraph loop
        end % if do plot loop
        % plot corridor width approximation values
        if flag_do_plot
            fig_num = fig_num + 1;
            figure(fig_num); hold on; box on;
            for j = 1:length(polytopes)
                 fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
            end
            hold on; box on;
            xlabel('x [m]');
            ylabel('y [m]');
            l_or_r_string = {'left','right'};
            title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
            vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
            for i = 1:size(vgraph,1)
                for j = 1:size(vgraph,1)
                    % skip start and finish for plotting clarity
                    if (i == start(3) || j == finish(3) || i == finish(3) || j == start(3))
                        continue
                    end
                    if vgraph(i,j) == 1
                        % plot a nice gray line
                        quiver(starts(i,1),starts(i,2),starts(j,1)-starts(i,1),starts(j,2)-starts(i,2),'--','Color',0.4*ones(1,3),'LineWidth',2);
                        % label the dilation robustness
                        text((starts(i,1)+starts(j,1))/2 ,(starts(i,2)+starts(j,2))/2, string(dilation_robustness_matrix(i,j,left_or_right)));
                    end
                end % inner vgraph loop
            end % outer vgraph loop
        end % if do plot loop
    end % left or right loop
end % test case loop
