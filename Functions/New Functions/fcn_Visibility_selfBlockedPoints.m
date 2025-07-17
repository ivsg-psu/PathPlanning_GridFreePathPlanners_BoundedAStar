function [cur_obs_id, self_blocked_cost, pts_blocked_by_self] = ...
    fcn_Visibility_selfBlockedPoints(polytopes,cur_pt,all_pts)
    % fcn_Visibility_selfBlockedPoints
    % determines the points blocked by the obstacle that the planner is currently
    % at a vertex of
    %
    % FORMAT:
    %
    % [cur_obs_id, self_blocked_cost, pts_blocked_by_self] = ...
    % fcn_Visibility_selfBlockedPoints(polytopes,cur_pt,all_pts)
    %
    % INPUTS:
    %
    % POLYTOPES: a 1-by-n seven field structure of shrunken polytopes,
    % where n <= number of polytopes with fields:
    %   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
    %     the number of the individual polytope vertices
    %   xv: a 1-by-m vector of vertice x-coordinates
    %   yv: a 1-by-m vector of vertice y-coordinates
    %   distances: a 1-by-m vector of perimeter distances from one point to the
    %     next point, distances(i) = distance from vertices(i) to vertices(i+1)
    %   mean: centroid xy coordinate of the polytope
    %   area: area of the polytope
    % cur_pt: the 1x5 array representing the current point, expected to be a vertex of a polytope
    % ALL_PTS: p-by-5 matrix of all the points except start and finish
    %
    % OUTPUTS:
    % cur_obs_id - obstacle ID of the polytope that cur_pt is a vertex of
    % self_blocked_cost - polytope traversal scaling of the polytope the cur_pt is a vertex of
    % pts_blocked_by_self - the other vertices on the polytope that cur_pt is a vertex of
    % that cannot be seen from cur_pt (i.e. neither neighboring vertex which would be visible
    % by looking down the side of a convex polytope)
    %
    % DEPENDENCIES:
    %
    % EXAMPLES:
    %
    % For additional examples, see implementation of this in the through planner in
    % fcn_algorithm_bound_Astar.m
    %
    % This function was written in 2022_05 by Steve Harentt
    % Questions or comments? sjh6473@psu.edu
    %

    % Revision History:
    % 2025_07_17 - K. Hayes, kxh1031@psu.edu
    % -- copied to new function from fcn_visibility_self_blocked_pts to
    %    follow library convention

    % TO DO
    % -- implement contingency for concave polytopes

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


    % find cur_pt's position in vertices
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
