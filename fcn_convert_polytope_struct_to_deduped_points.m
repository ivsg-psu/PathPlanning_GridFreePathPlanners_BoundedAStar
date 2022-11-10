function deduped_points_struct = fcn_convert_polytope_struct_to_deduped_points(all_pts)
    times_recursed = 1;
    deduped_points_struct = [];
    deduped_points_struct = ...
        INTERNAL_fcn_convert_polytope_struct_to_deduped_pts(deduped_points_struct, all_pts,times_recursed);
end
function deduped_points_struct = ...
    INTERNAL_fcn_convert_polytope_struct_to_deduped_pts(deduped_points_struct, all_pts, i);

    deduped_points_struct(i).x = all_pts(i,1);
    deduped_points_struct(i).y = all_pts(i,2);
    same_x_idx = find(all_pts(:,1)==all_pts(i,1));
    same_y_idx = find(all_pts(:,1)==all_pts(i,1));
    % find ids that have both same x and y
    same_point_idx = intersect(same_x_idx,same_y_idx);
    % make a list of obstacles with the same point
    % find indecies of rows for same_point_idx and column 4
    idx = sub2ind(size(all_pts), same_point_idx, 4.*ones(size(same_point_idx,1),size(same_point_idx,2)));
    % go to these ids and store in obstalce list
    obs_on_cur_pt = all_pts(idx);
    deduped_points_struct(i).polys = obs_on_cur_pt;
    % remove all the points we checked from all_pts
    for j = 1:length(same_point_idx)
        all_pts(same_point_idx(j),:) = [];
    end
    if length(all_pts) ~= 0:
        times_recursed = i + 1;
        deduped_points_struct = ...
            INTERNAL_fcn_convert_polytope_struct_to_deduped_pts(deduped_points_struct, all_pts,times_recursed);
    else
        return
    end
end
