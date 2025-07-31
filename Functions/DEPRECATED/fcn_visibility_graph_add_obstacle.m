function [visibility_matrix_new, all_pts_new, start_new, finish_new, polytopes_after] = ...
    fcn_visibility_graph_add_obstacle(...
    visibility_matrix, all_pts, start, finish, polytopes_before, polytope_to_add)
    % fcn_visibility_graph_add_obstacle
    %   this function recalculates the visibility graph after adding a polytope without recalculating
    %   the entire visibility graph.  This is accomplished using an AABB check as a coarse check.
    %   This function also recalculates the all_pts, start, finish, and polytopes data structures
    %   as these are also affected by the addition of an obstacle.
    % see vgraph_modification section of Documentation/bounded_astar_documentation.pptx for pseudocode and algorithm description
    %
    %
    % FORMAT:
    % [visibility_matrix_new, all_pts_new, start_new, finish_new, polytopes_after] = ...
    %     fcn_visibility_graph_add_obstacle(...
    %     visibility_matrix, all_pts, start, finish, polytopes_before, polytope_to_add)
    %
    % INPUTS:
    %     visibility_matrix - nxn matrix, where n is the number of points in all_pts
    %       a 1 in column i and row j indicates that all_pts(i,:) is visible from
    %       all_pts(j,:).  This matrix is therefore symmetric
    %     ALL_PTS: p-by-5 matrix of all the possible start points
    %       the information in the 5 columns is as follows:
    %         x-coordinate
    %         y-coordinate
    %         point id number
    %         obstacle id number
    %         beginning/ending indication (1 if the point is a beginning or ending
    %         point and 0 otherwise)
    %         Ex: [x y point_id obs_id beg_end]
    %     START: 1-by-5 vector with the same info as route for the starting point
    %     FINISH: same as start for the finish point
    %     polytopes_before - the polytope struct array prior to modification EXCLUDING the polytope for additionk
    %     polytope_to_add - struct of the polytope to add
    %
    %
    % OUTPUTS:
    %     visibility_matrix_new: same as the visiblity_matrix input but modified so that the added
    %         polytope now affects the visibility.  Note this may have more points than
    %         the input matrix as points on the added polytope are now considered as nodes.
    %     all_pts_new: same as the all_pts input but with points on the added polytope.
    %         May be reindexed.
    %     start_new: same as start input but reindexed to account for added points
    %     finish_new: same as finish input but reindexed to account for added points
    %     polytopes_after:  the polytope struct array after modification now including
    %         the polytope for addition
    %
    % DEPENDENCIES:
    %     fcn_MapGen_isCrossingAABB from the MapGen repo
    %     fcn_visibility_clear_and_blocked_points
    %
    % EXAMPLES:
    %
    % See the script: script_visibility_graph_modification
    % for a full test suite.
    %
    % Questions or comments? contact sjh6473@psu.edu

    % REVISION HISTORY:
    % 2024_03
    % -- first written by Steve Harnett
    % Questions? sjh6473@psu.edu

    %% initialize the modified vgraph, pts, start, and finish to the old values
    visibility_matrix_new = visibility_matrix;
    old_point_count = size(visibility_matrix,2);
    start_new = start;
    finish_new = finish;

    %% add polytope
    polytopes_after = [polytopes_before, polytope_to_add]; % append new polytope to polytope struct array
    % form AABB for added polytope
    polytope_vertices = [[polytope_to_add.xv]',[polytope_to_add.yv]'];
    AABB = [min(polytope_to_add.xv) min(polytope_to_add.yv) max(polytope_to_add.xv) max(polytope_to_add.yv)];
    % remake all_pts table
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
    %% initialize new visibility matrix as block matrix
    % the following code inserts the new points on the new polytope into the vgraph such that
    % the start and finish are still the last two points in the matrix rather than simply appending
    % new points to the end of the vgraph resulting in the start and finish being somewhere in
    % the interior of the matrix
    % a diagram of what this does is shown below
    % old_vgraph =
    %   ___________________________________________________________________
    %   |  A                             | D                             |
    %   |  the original vgraph           | original vgraph               |
    %   |  block for points excluding    | block for start and finish    |
    %   |  the start and the finish      | relative to other pts         |
    %   ___________________________________________________________________
    %   | D'                             |  F                            |
    %   | original vgraph                |  original vgraph              |
    %   | block for start and finish     |  block for start and finish   |
    %   | relative to other pts          |  relative to themselves       |
    %   ___________________________________________________________________
    % new_vgraph =
    %   ___________________________________________________________________________________________________
    %   |  A                             | B                             | D                             |
    %   |  the original vgraph           | block of zeros for new points | original vgraph               |
    %   |  block for points excluding    | relative to other pts         | block for start and finish    |
    %   |  the start and the finish      | init. to zero until checked   | relative to other pts         |
    %   ___________________________________________________________________________________________________
    %   |  B'                            | C                             | E                             |
    %   |  block of zeros for new points | block for new points relative | block of zeros for new points |
    %   |  relative to other pts         | to themselves.  Init. to      | relative to start and finish  |
    %   |  init. to zero until checked   | identity until checked        | init. to zero until checked   |
    %   ___________________________________________________________________________________________________
    %   | D'                             | E'                            |  F                            |
    %   | original vgraph                | block of zeros for new points |  original vgraph              |
    %   | block for start and finish     | relative to start and finish  |  block for start and finish   |
    %   | relative to other pts          | init. to zero until checked   |  relative to themselves       |
    %   ___________________________________________________________________________________________________


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
    %% fill in values for new vgraph based on aabb coarse check first
    isInside = fcn_MapGen_isCrossingAABB(AABB, [all_pts_new; start; finish]);
    % need to note edges that are 1 in new vgraph (i.e. they are unblocked) and cross the AABB
    [r,c] = find(isInside & visibility_matrix_new);
    % also need to note edges that go to any point but either start or end on the new obstacle
    idx_of_new_obs = (old_point_count - start_and_finish_count + 1):(old_point_count - start_and_finish_count + new_point_tot); % ids of new points
    idx_of_all_pts = 1:size(visibility_matrix_new,1); % ids of all points
    combos_of_idx_of_new_obs = combinations(idx_of_new_obs,idx_of_all_pts); % every combination of a new point with some other points
    array_of_combos_of_idx_of_new_obs = table2array(combos_of_idx_of_new_obs); % convert from table to array
    % append the list of new edges to the list of edges possibly blocked by the new obstacle because they cross the AABB
    r = [r;array_of_combos_of_idx_of_new_obs(:,1)];
    c = [c;array_of_combos_of_idx_of_new_obs(:,2)];
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
    sprintf('num checks was %i',length(r))
end
