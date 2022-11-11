function unique_deduped_points_struct = fcn_convert_polytope_struct_to_deduped_points(all_pts)
    deduped_points_struct = [];
    for i = 1:size(all_pts,1)
        deduped_points_struct(i).x = all_pts(i,1);
        deduped_points_struct(i).y = all_pts(i,2);
        same_x_idx = find(round(all_pts(:,1),5)==round(all_pts(i,1),5));
        same_y_idx = find(round(all_pts(:,2),5)==round(all_pts(i,2),5));
        % find ids that have both same x and y
        same_point_idx = intersect(same_x_idx,same_y_idx);
        % make a list of obstacles with the same point
        % find indecies of rows for same_point_idx and column 4
        idx = sub2ind(size(all_pts), same_point_idx, 4.*ones(size(same_point_idx,1),size(same_point_idx,2)));
        % go to these ids and store in obstalce list
        obs_on_cur_pt = all_pts(idx);
        deduped_points_struct(i).polys = obs_on_cur_pt;
    end
    x_y_pairs = [round(extractfield(deduped_points_struct,'x'),5);round(extractfield(deduped_points_struct,'y'),5)]';
    [C,unique_pair_idxs,ic] = unique(x_y_pairs,'rows');
    unique_pairs = x_y_pairs(unique_pair_idxs,:);
    unique_deduped_points_struct = deduped_points_struct(unique_pair_idxs);
end
