function [visibility_matrix_new, all_pts_new, start_new, finish_new, polytopes_after] = ...
    fcn_visibility_graph_add_obstacle(...
    visibility_matrix, all_pts, start, finish, polytopes_before, polytope_to_add)
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
    old_point_count = size(visibility_matrix,2);
    start_new = start;
    finish_new = finish;
    polytopes_after = [polytopes_before, polytope_to_add];
    polytope_vertices = [[polytope_to_add.xv]',[polytope_to_add.yv]'];
    AABB = [min(polytope_to_add.xv) min(polytope_to_add.yv) max(polytope_to_add.xv) max(polytope_to_add.yv)];
    % remake all_pts table
    % TODO @sjharnett call all_pts function here again
    new_point_tot = length([polytope_to_add.xv]); % total number of vertices in the polytopes
    beg_end = zeros(1,new_point_tot); % is the point the start/end of an obstacle
    curpt = 0;
    polytope_to_add.obs_id = ones(1,new_point_tot)*(max(all_pts(:,4))+1); % obs_id is the same for every vertex on a single polytope
    beg_end([curpt+1,curpt+new_point_tot]) = 1; % the first and last vertices are marked with 1 and all others are 0
    curpt = curpt+new_point_tot;
    obs_id = [polytope_to_add.obs_id];
    all_pts_new = [[polytope_to_add.xv];[polytope_to_add.yv];max(all_pts(:,3))+1:max(all_pts(:,3))+new_point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
    all_pts_new = [all_pts; all_pts_new];
    % if the user gave a start or finish, reindex it
    start_and_finish_count = 0;
    if ~isempty(start_new)
        start_new = [start size(all_pts_new,1)+1 -1 1];
        start_and_finish_count = start_and_finish_count + 1;
    end
    if ~isempty(finish_new)
        finish_new = [finish size(all_pts_new,1)+2 -1 1];
        start_and_finish_count = start_and_finish_count + 1;
    end
    A = visibility_matrix(1:(old_point_count-start_and_finish_count),1:(old_point_count-start_and_finish_count));
    assert(isequal(size(A) ,  [old_point_count - start_and_finish_count, old_point_count - start_and_finish_count]))
    B = zeros(old_point_count-start_and_finish_count, new_point_tot);
    assert(isequal(size(B) ,  [old_point_count - start_and_finish_count, new_point_tot]))
    C = eye(new_point_tot, new_point_tot);
    assert(isequal(size(C) ,  [new_point_tot, new_point_tot]))
    if start_and_finish_count ~= 0
        D = visibility_matrix(1:end-start_and_finish_count,end-start_and_finish_count+1:end);
        assert(isequal(size(D) ,  [old_point_count - start_and_finish_count, start_and_finish_count]))
        E = zeros(new_point_tot,start_and_finish_count);
        assert(isequal(size(E) ,  [new_point_tot, start_and_finish_count]))
        F = visibility_matrix(end-start_and_finish_count+1:end,end-start_and_finish_count+1:end);
        assert(isequal(size(F) ,  [start_and_finish_count, start_and_finish_count]))
    else
        D = [];
        E = [];
        F = [];
    end
    visibility_matrix_new = [A, B, D; B', C, E; D', E', F];

    isInside = fcn_MapGen_isCrossingAABB(AABB, [all_pts_new; start; finish]);
    [r,c] = find(isInside & visibility_matrix_new);
    idx_of_new_obs = (old_point_count - start_and_finish_count + 1):(old_point_count - start_and_finish_count + new_point_tot);
    idx_of_all_pts = 1:size(visibility_matrix_new,1);
    combos_of_idx_of_new_obs = combinations(idx_of_new_obs,idx_of_all_pts);
    array_of_combos_of_idx_of_new_obs = table2array(combos_of_idx_of_new_obs);
    r = [r;array_of_combos_of_idx_of_new_obs(:,1)];
    c = [c;array_of_combos_of_idx_of_new_obs(:,2)];
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
        else
            visibility_matrix_new(r(i),c(i)) = 0;
        end
    end
    toc(inner)
    sprintf('num checks was %i',length(r))
end
