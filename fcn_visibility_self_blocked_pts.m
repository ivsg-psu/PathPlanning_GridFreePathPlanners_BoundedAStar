function [cur_obs_id, self_blocked_cost, pts_blocked_by_self] = ...
    fcn_visibility_self_blocked_pts(polytopes,cur_pt,all_pts)

    cur_obs_id = cur_pt(4);
    if cur_obs_id == -1
        % this isn't on any polytope
        pts_blocked_by_self = [];
        self_blocked_cost = 0;
        return
    end
    % find points with current point's obstacle id
    [row, col] = find(all_pts(:,4)==cur_obs_id);
    pts_on_cur_poly = all_pts(row(1):row(end),:);

    % find polytope that current point is on
    cur_poly = polytopes(cur_obs_id);

    % check that the cur_pt is indeed a vertex of cur_poly
    assert(ismember(cur_pt(1),cur_poly.xv))
    assert(ismember(cur_pt(2),cur_poly.yv))

    % TODO(@sjharnett) this needs tests, minimum of three (middle vertex is curPt, end is curPt, beginning is curPt)

    % vertices = cur_poly.vertices;

    % remove last vertex
    % vertices = vertices(1:end-1,:);

    % find cur_pt's position in vertices
    % cur_pt_xy = cur_pt(1:2);
    % [~,cur_pt_indx]=ismember(cur_pt_xy,vertices,'rows');
    [~,cur_pt_indx]=ismember(cur_pt,pts_on_cur_poly,'rows');

    % initialize array of self blocked pts
    pts_blocked_by_cur_poly = pts_on_cur_poly;

    % remove cur_pt
    pts_blocked_by_cur_poly(cur_pt_indx,:) = NaN(1,5);

    % remove neighbor before and neighbor after
    if cur_pt_indx-1 == 0
        % remove the end if cur_pt is the first point
        pts_blocked_by_cur_poly(end,:) = NaN(1,5);
    else
        % or just remove whatever is before cur_pt
        pts_blocked_by_cur_poly(cur_pt_indx-1,:) = NaN(1,5);
    end
    if cur_pt_indx  == size(pts_blocked_by_cur_poly,1)
        % remove the beginning if cur_pt is the last point
        pts_blocked_by_cur_poly(1,:) = [];
    else
        % or just remove whatever is after cur_pt
        pts_blocked_by_cur_poly(cur_pt_indx+1,:) = NaN(1,5);
    end
    pts_blocked_by_self = rmmissing(pts_blocked_by_cur_poly);
    self_blocked_cost = cur_poly.cost;
end
