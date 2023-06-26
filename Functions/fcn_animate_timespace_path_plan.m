function fcn_animate_timespace_path_plan(start, finish, verts, route_dense, dt)
    close all;
    %% create an animation for moving line
    dense_times = route_dense(:,3);
    num_dense_times = length(dense_times);
    for i = 1:num_dense_times
        hold on; box on; title(sprintf('animation of routing around \n moving obstacles shown at 10x speed'))
        % define figure properties
        opts.width      = 8;
        opts.height     = 6;
        opts.fontType   = 'Times';
        opts.fontSize   = 9;
        fig = gcf;
        % scaling
        fig.Units               = 'centimeters';
        fig.Position(3)         = opts.width;
        fig.Position(4)         = opts.height;
        set(gcf,'color','white')
        % set text properties
        set(fig.Children, ...
            'FontName',     'Times', ...
            'FontSize',     9);

        % remove unnecessary white space
        set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
        xlabel('x [m]')
        ylabel('y [m]')
        ylim([-1 3])
        xlim([0 4])
        % for each poly
        % this polys verts
        % need to get x y and z coords at this time
        cur_time = dense_times(i);
        cur_time_locations = find(verts(:,3) == cur_time);
        cur_x = verts(cur_time_locations,1);
        cur_y = verts(cur_time_locations,2);

        cur_route_idx = find(route_dense(:,3) == cur_time);

        p_route = plot(route_dense(1:cur_route_idx,1),route_dense(1:cur_route_idx,2),'-k','LineWidth',2);
        p_pose = plot(route_dense(cur_route_idx,1),route_dense(cur_route_idx,2),'xk','MarkerSize',2);
        cur_time_locations_in_finish = find(finish(:,3) == cur_time);
        p_start = plot(start(:,1),start(:,2),'gx');
        p_finish = plot(finish(cur_time_locations_in_finish ,1),finish(cur_time_locations_in_finish,2),'rx');
        fill(cur_x,cur_y,'b','FaceAlpha',0.2);
        if i == 1
            gif('moving_wall_with_path.gif','LoopCount',1,'DelayTime',dt/10)
        else
            gif
        end
        delete(gca)
        delete(p_route)
        delete(p_pose)
        delete(p_start)
        delete(p_finish)
    end
        % https://www.mathworks.com/matlabcentral/fileexchange/33073-triangle-ray-intersection
    % https://en.wikipedia.org/wiki/Intersection_of_a_polyhedron_with_a_line
    % https://www.mathworks.com/help/matlab/visualize/multifaceted-patches.html
    % at each time t, calculate P
    % ir for all points on all bodies that are candidates for collision
    % select one point P
    % jr on body j and test if it is inside body i
    % cast an infinite ray in any direction from P
    % jr and compute intersections with all facets on body i
    % if there are an even number of intersections, P
    % jr is not inside body i
    % if there are an odd number of intersections, P
    % jr is inside body i
    % works for concavities, holes and self-crossing and is independent of CW versus CCW boundary
    % ray-facet intersection is similar to edge-facet intersection described below
    % point in polygon does not always work for thin bodies (may need edge intersection)
    %
    function INTERNAL_fcn_format_timespace_plot()
        % define figure properties
        opts.width      = 8;
        opts.height     = 6;
        opts.fontType   = 'Times';
        opts.fontSize   = 9;
        fig = gcf;
        % scaling
        fig.Units               = 'centimeters';
        fig.Position(3)         = opts.width;
        fig.Position(4)         = opts.height;

        % set text properties
        set(fig.Children, ...
            'FontName',     'Times', ...
            'FontSize',     9);

        % remove unnecessary white space
        set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
        xlabel('x [m]')
        ylabel('y [m]')
        zlabel('t [s]')
        view([36 30])
    end
end
