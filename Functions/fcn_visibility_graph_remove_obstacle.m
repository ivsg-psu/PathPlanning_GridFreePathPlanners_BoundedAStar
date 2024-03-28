function [visibility_matrix_new, all_pts_new, start_new, finish_new, polytopes_after] = ...
    fcn_visibility_graph_remove_obstacle(...
    visibility_matrix, all_pts, start, finish, polytopes_before, idx_of_polytope_for_removal)
    % fcn_MapGen_increasePolytopeVertexCount
    % The function fcn_visibility_clear_and_blocked_points returns an intersection
    % matrix for a single start point, showing what was intersected between
    % that start point and numerous possible end points.
    % This function wraps that function to call it on every possible start and end
    % combination to provide global visibility truth tables rather than local
    % intersection truth tables.
    %
    %
    % FORMAT:
    % visibility_matrix = fcn_visibility_clear_and_blocked_points_global(polytopes,all_pts)
    %
    % INPUTS:
    %     polytopes - the polytope field
    %     ALL_PTS: p-by-5 matrix of all the possible start points
    %       the information in the 5 columns is as follows:
    %         x-coordinate
    %         y-coordinate
    %         point id number
    %         obstacle id number
    %         beginning/ending indication (1 if the point is a beginning or ending
    %         point and 0 otherwise)
    %         Ex: [x y point_id obs_id beg_end]
    %      gap_size: if zero, the special fully tiled case will be handled.
    %         This involves assuming that visibility is only down sides and through polytopes
    %     (optional inputs)
    %     is_concave: set a 1 to allow for concave (i.e. non-convex) obstacles.  If this is left
    %         blank or set to anyting other than 1, the function defaults to the convex behavior
    %         which is more conservative (i.e. setting the flag wrong incorrectly may result in
    %         suboptimal paths but not collisions). For background on what this flag does, see slides 9-14 here:
    %         https://pennstateoffice365.sharepoint.com/:p:/r/sites/IntelligentVehiclesandSystemsGroup-Active/Shared%20Documents/IVSG/Theses/2025_Harnett_PhD/Weekly%20Updates/HARNETT_WEEKLY_UPDATE_JAN08_2024.pptx?d=w4f5e75a3c5b343aab47b41d2b945075b&csf=1&web=1&e=5otpZ3
    %
    %
    % OUTPUTS:
    %
    %     visibility_matrix - nxn matrix, where n is the number of points in all_pts
    %       a 1 in column i and row j indicates that all_pts(i,:) is visible from
    %       all_pts(j,:).  This matrix is therefore symmetric
    %
    % DEPENDENCIES:
    %     fcn_visibility_clear_and_blocked_points
    %
    % EXAMPLES:
    %
    % See the script: script_fcn_visibility_clear_and_blocked_points_global.m
    % for a full test suite.
    %
    % Questions or comments? contact sjh6473@psu.edu

    % REVISION HISTORY:
    % 2021_10_28
    % -- first written by Steve Harnett
    % Questions? sjh6473@psu.edu
    outer = tic;
    visibility_matrix_new = visibility_matrix;
    all_pts_new = all_pts;
    start_new = start;
    finish_new = finish;
    all_pts_with_start_and_fin = [all_pts; start; finish];
    polytope_of_interest = polytopes_before(idx_of_polytope_for_removal);
    polytope_vertices = [[polytope_of_interest.xv]',[polytope_of_interest.yv]'];
    AABB = [min(polytope_of_interest.xv) min(polytope_of_interest.yv) max(polytope_of_interest.xv) max(polytope_of_interest.yv)];

    polytopes_after = polytopes_before;
    polytopes_after(idx_of_polytope_for_removal) = [];
    [~, xloc] = ismember(polytope_vertices(:,1), all_pts(:,1));
    [~, yloc] = ismember(polytope_vertices(:,2), all_pts(:,2));
    idx_of_points_on_polytope = union(xloc,yloc);
    % vgraph edges that start or end on this obstacle should be removed
    visibility_matrix_new(idx_of_points_on_polytope,:) = [];
    visibility_matrix_new(:,idx_of_points_on_polytope) = [];
    % remove points on that polytope from all_pts table
    all_pts_new(idx_of_points_on_polytope,:) = [];
    % size of vgraph has now changed so points need to be re-indexed
    % reindex all_pts, start, and finish
    num_pts_after_removal = size(all_pts_new,1);
    all_pts_new(:,3) = [1:num_pts_after_removal]';
    % if the user gave a start or finish, reindex it
    if ~isempty(start_new)
        start_new(3) = num_pts_after_removal+1;
    end
    if ~isempty(finish_new)
        finish_new(3) = num_pts_after_removal+2;
        assert(finish_new(3) == size(visibility_matrix_new,1))
    end
    % find which new points cross the AABB
    isInside = fcn_MapGen_isCrossingAABB(AABB, [all_pts_new; start; finish]);
    % only want possible edges that are not already edges as deleting the obstacle adds edges, it does
    % not remove existing edges
    [r,c] = find(isInside & ~visibility_matrix_new);
    toc(outer)
    inner = tic
    %% check only  specific edges method
    % TODO @sjharnett use global function and check from each start to all finishes for that start
    for i = 1:length(r)
        [~,~,D] = fcn_visibility_clear_and_blocked_points(polytopes_after, all_pts_new(r(i),:), all_pts_new(c(i),:));
        visibility_scalar = sum(D);
        assert(isequal(size(visibility_scalar),[1 1]))
        if ~visibility_scalar
            visibility_matrix_new(r(i),c(i)) = 1;
        end
    end
    toc(inner)
    sprintf('num checks was %i',length(r))
end
