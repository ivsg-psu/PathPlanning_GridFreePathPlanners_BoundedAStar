function vgraph = fcn_visibility_graph_3d_global(verts, start, finish, all_surfels, speed_limit)

    all_pts = [verts; start; finish];
    num_pts = size(all_pts,1); % number of rows
    all_pts_idx = 1:1:num_pts; % array of all possible pt idx
    all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts
    all_pt_combos = nchoosek(all_pts_idx,2); % each row of this matrix is a combination of 2 point idxs

    all_ray_starts = all_pts(all_pt_combos(:,1),:); % take all cols of all_pts at the row provided by the first col of all combos
    all_ray_ends = all_pts(all_pt_combos(:,2),:); % take all cols of all_pts at the row provided by the second col of all combos
    all_ray_dirs = all_ray_ends - all_ray_starts; % TriangleRayIntersection takes a ray direction which is end minus beginning
    num_rays = size(all_ray_starts,1);

    figure; hold on; box on; title('all rays casted')
    INTERNAL_fcn_format_timespace_plot();
    for i = 1:1:num_rays
        plot3([all_ray_starts(i,1), all_ray_ends(i,1)],[all_ray_starts(i,2), all_ray_ends(i,2)],[all_ray_starts(i,3), all_ray_ends(i,3)],'LineWidth',2)
    end

    figure; hold on; box on; title('vgraph')
    INTERNAL_fcn_format_timespace_plot();
    for i = 1:1:num_rays
        plot3([all_ray_starts(i,1), all_ray_ends(i,1)],[all_ray_starts(i,2), all_ray_ends(i,2)],[all_ray_starts(i,3), all_ray_ends(i,3)],'Color',[0 1 0],'LineWidth',1)
    end

    num_surfels = size(all_surfels,1);
    all_ray_idx = 1:1:num_rays;
    all_surfel_idx = 1:1:num_surfels;
    all_surfel_ray_combos = table2array(combinations(all_ray_idx,all_surfel_idx));

    all_ray_starts_repeated = all_ray_starts(all_surfel_ray_combos(:,1),:);
    all_ray_ends_repeated = all_ray_ends(all_surfel_ray_combos(:,1),:);
    all_ray_dirs_repeated = all_ray_dirs(all_surfel_ray_combos(:,1),:);
    all_surfels_repeated = all_surfels(all_surfel_ray_combos(:,2),:);

    [intersects, ts, us, vs, xcoors] = TriangleRayIntersection (all_ray_starts_repeated(:,1:3), all_ray_dirs_repeated(:,1:3), all_surfels_repeated(:,1:3),all_surfels_repeated(:,4:6),all_surfels_repeated(:,7:9),'lineType','segment','border','normal');

    vgraph = ones(num_pts); % initialize vgraph as zero
    intersects_idx = find(intersects);
    for k = 1:1:length(intersects_idx)
        i = intersects_idx(k);
        % if the intersection occured at a node that implies that the end of the ray
        % touched a plane segment, rather than the ray passing through the plane
        intersect_x = xcoors(i,1);
        intersect_y = xcoors(i,2);
        intersect_t = xcoors(i,3);
        verts_x = all_pts(:,1);
        verts_y = all_pts(:,2);
        verts_t = all_pts(:,3);
        diff_intersect_and_verts = abs([verts_x - intersect_x, verts_y - intersect_y, verts_t - intersect_t]);
        total_diffs = sum(diff_intersect_and_verts,2);
        small_diffs_bool = total_diffs < 10e-14;
        if sum(small_diffs_bool) == 0
            plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'Color',[1 0 0],'LineWidth',1)
            plot3(rmmissing(xcoors(i,1)),rmmissing(xcoors(i,2)),rmmissing(xcoors(i,3)),'cx','MarkerSize',10)
            start_id = all_ray_starts_repeated(i,4);
            end_id = all_ray_ends_repeated(i,4);
            vgraph(start_id,end_id) = 0;
            vgraph(end_id,start_id) = 0;
        end
    end
    fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b','FaceAlpha',0.3);
    fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'b','FaceAlpha',0.3);

    %% discard rays that are too high in velocity
    % ray slope is rise over run
    % rise is delta t
    % all_delta_ts = (all_ray_ends_repeated(:,3) - all_ray_starts_repeated(:,3));
    % % run is change in total length regardless of x or y
    % all_delta_xs = all_ray_ends_repeated(:,1) - all_ray_starts_repeated(:,1);
    % all_delta_ys = all_ray_ends_repeated(:,2) - all_ray_starts_repeated(:,2);
    % all_delta_dist = (all_delta_xs.^2 + all_delta_ys.^2).^0.5;
    % all_slopes = all_delta_ts./all_delta_dist;
    %
    % speed_violation_idx = find(all_slopes <= 0);
    % for l = 1:1:length(speed_violation_idx)
    %     i = speed_violation_idx(l);
    %     plot3([all_ray_starts_repeated(i,1), all_ray_ends_repeated(i,1)],[all_ray_starts_repeated(i,2), all_ray_ends_repeated(i,2)],[all_ray_starts_repeated(i,3), all_ray_ends_repeated(i,3)],'k','LineWidth',2)
    %     start_id = all_ray_starts_repeated(i,4);
    %     end_id = all_ray_ends_repeated(i,4);
    %     vgraph(start_id,end_id) = 0;
    % end

    %% discard rays too high in velocity using all pts array
    all_delta_ts = (-all_pts(:,3) + (all_pts(:,3))');
    % run is change in total length regardless of x or y
    all_delta_xs = (all_pts(:,1) - (all_pts(:,1))');
    all_delta_ys = (all_pts(:,2) - (all_pts(:,2))');
    all_delta_dist = (all_delta_xs.^2 + all_delta_ys.^2).^0.5;
    all_slopes = all_delta_ts./all_delta_dist;

    speed_violation_idx = find(all_slopes <= 1/speed_limit );
    for l = 1:1:length(speed_violation_idx)
        i = speed_violation_idx(l);
        [beg,term] = ind2sub(size(all_slopes),i);
        % plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],'k','LineWidth',2)
        start_id = all_pts(beg,4);
        end_id = all_pts(term,4);
        vgraph(start_id,end_id) = 0;
    end
end

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
