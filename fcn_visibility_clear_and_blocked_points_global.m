function visibility_matrix = fcn_visibility_clear_and_blocked_points_global(polytopes, all_pts, gap_size)
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

    % TODO(@sjharnett) could use the Lee algorithm to speed up if necessary
    % https://github.com/davetcoleman/visibility_graph/blob/master/Visibility_Graph_Algorithm.pdf
    % TODO(@sjharnett) could also discard sides based on direction of normal relative to scan direction
    % there is an issue: polytopes contain points, therefore points are represented multiple times
    % need the reverse mappping of points to polytopes
    % then each point is only represented once
    % visibility graph can then be reduced
    num_points = size(all_pts,1)
    % for non-zero gap size, we can repeatedly call the legacy visibility functions
    if gap_size ~= 0
        visibility_matrix = NaN(num_points)
        %% loop through all points
        for i = 1:num_points
            % legacy visibility function returns visibility vector for this point
            [clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ] = ...
                fcn_visibility_clear_and_blocked_points(polytopes,all_pts(i,:),all_pts);
            % D is finish points on the rows and polytope sides on the columns
            % transpose this so we have column for each point
            % sum each column into a row vector so each element is the sum of number
            % of sides hit
            % if sum>0, this implies there is no visibility
            visibility_matrix(i,:) = sum(D')==0;
            % sometimes the diagonal does not contain only 1's (it always should
            % since every point is visible to itself so we overwrite this)
            visibility_matrix(i,i) = 1;

            %% add self-blocked points
            % points across the polytope are also visible so find self blocked pts
            % find obs_id for cur_pt
            cur_obs_id = all_pts(i,4);
            % find pt ids of every point on this obstacle
            pt_idx = find(all_pts(:,4)==cur_obs_id);
            % at row i, and columns with values in pt_idx...
            idx = sub2ind(size(visibility_matrix), i.*ones(size(pt_idx,1),size(pt_idx,2)), pt_idx);
            % set a 1, indicating the self-blocked points are visible
            visibility_matrix(idx) = 1;
        end
    else
        % for the zero gap size case, we can do an optimization: all points on the same polytope
        % are visible, either along the side or across the polytope
        % other points are not visible since there are no gaps and angles of 180 deg
        % are not possible in a Voronoi diagram where all vertices have 3 Voronoi sides
        deduped_pts = fcn_convert_polytope_struct_to_deduped_pts(polytopes);

    end
end
