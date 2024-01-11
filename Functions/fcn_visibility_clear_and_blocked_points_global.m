function [visibility_matrix, visibility_results] = fcn_visibility_clear_and_blocked_points_global(polytopes, starts, finishes, varargin)
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
    %% check input arguments
    if nargin < 3 || nargin > 4
        error('Incorrect number of arguments');
    end
    % if there is no value in varargin...
    if nargin == 3
        % default is to assume convex obstacles as this is conservative
        is_concave = 0;
    end
    % if there is a value in varargin...
    if nargin == 4
        % check what it is
        if varargin{1} == 1
            % set concave flag if it was passed in
            is_concave = 1;
        elseif varargin{1} == 0
            is_concave = 0;
        else
            % throw error if it was passed in with an incorrect value
            error('optional argument is the is_concave flag and can either be 1 or 0')
        end
    end

    % TODO(@sjharnett) could use the Lee algorithm to speed up if necessary
    % https://github.com/davetcoleman/visibility_graph/blob/master/Visibility_Graph_Algorithm.pdf
    % TODO(@sjharnett) could also discard sides based on direction of normal relative to scan direction
    % there is an issue: polytopes contain points, therefore points are represented multiple times
    % need the reverse mappping of points to polytopes
    % then each point is only represented once
    % visibility graph can then be reduced
    num_points = size(starts,1);
    % for non-zero gap size, we can repeatedly call the legacy visibility functions

    % if gap_size ~= 0
        visibility_matrix = NaN(num_points);
        %% loop through all points
        for j = 1:num_points
            i = starts(j,3);
            % legacy visibility function returns visibility vector for this point
            [visibility_results(i).clear_pts,visibility_results(i).blocked_pts,visibility_results(i).D,visibility_results(i).di,visibility_results(i).dj,visibility_results(i).num_int,visibility_results(i).xiP,visibility_results(i).yiP,visibility_results(i).xiQ,visibility_results(i).yiQ,visibility_results(i).xjP,visibility_results(i).yjP,visibility_results(i).xjQ,visibility_results(i).yjQ] = ...
                fcn_visibility_clear_and_blocked_points(polytopes,starts(j,:),finishes,is_concave);
            % D is finish points on the rows and polytope sides on the columns
            % transpose this so we have column for each point
            % sum each column into a row vector so each element is the sum of number
            % of sides hit
            % if sum>0, this implies there is no visibility
            visibility_matrix(i,:) = sum(visibility_results(i).D')==0;
            % sometimes the diagonal does not contain only 1's (it always should
            % since every point is visible to itself so we overwrite this)
            visibility_matrix(i,i) = 1;

            % %% add self-blocked points
            % % points across the polytope are also visible so find self blocked pts
            % % find obs_id for cur_pt
            % cur_obs_id = all_pts(i,4);
            % % find pt ids of every point on this obstacle
            % pt_idx = find(all_pts(:,4)==cur_obs_id);
            % % at row i, and columns with values in pt_idx...
            % idx = sub2ind(size(visibility_matrix), i.*ones(size(pt_idx,1),size(pt_idx,2)), pt_idx);
            % % set a 1, indicating the self-blocked points are visible
            % visibility_matrix(idx) = 1;
        end
    % elseif gap_size == 0
    %     % for the zero gap size case, we can do an optimization: all points on the same polytope
    %     % are visible, either along the side or across the polytope
    %     % other points are not visible since there are no gaps and angles of 180 deg
    %     % are not possible in a Voronoi diagram where all vertices have 3 Voronoi sides
    %     deduped_pts = fcn_convert_polytope_struct_to_deduped_points(all_pts);
    %     num_unique_pts = length(deduped_pts);
    %     all_polys = NaN(num_unique_pts,3);
    %     for i = 1:num_unique_pts
    %         for j = 1:length(deduped_pts(i).polys)
    %             all_polys(i,j) = deduped_pts(i).polys(j);
    %         end
    %     end
    %     visibility_matrix = zeros(num_unique_pts);
    %     for i = 1:num_unique_pts
    %         for j = 1:3
    %             poly_of_interest = all_polys(i,j);
    %             if isnan(poly_of_interest)
    %                 continue
    %             end
    %             [r,c] = find(all_polys == poly_of_interest);
    %             for k = 1:length(r)
    %                 visibility_matrix(i,r(k)) = 1;
    %             end
    %         end
    %    end
    % end
    %% check for edges entirely contained by polytopes
    % don't need to check self visible points as this will not change
    visibility_matrix_without_self_visible = visibility_matrix - eye(size(visibility_matrix,1));
    % find indeces of every other '1' or allowed edge...
    linear_idx = find(visibility_matrix_without_self_visible); % find 1s in visibility_matrix
    [rows_of_1s, cols_of_1s] = ind2sub(size(visibility_matrix_without_self_visible),linear_idx); % convert linear idx to r,c
    num_1s = length(rows_of_1s);
    for e = 1:num_1s
        start_pt = starts(rows_of_1s(e),1:3);
        end_pt = finishes(cols_of_1s(e),1:3);
        % parametric equation for line in 3D: https://math.stackexchange.com/questions/404440/what-is-the-equation-for-a-3d-line
        % [x y z]' = [a b c]'*t + [x0 y0 z0]'
        % parametric equation for line in 2D: https://math.libretexts.org/Bookshelves/Calculus/CLP-3_Multivariable_Calculus_(Feldman_Rechnitzer_and_Yeager)/01%3A_Vectors_and_Geometry_in_Two_and_Three_Dimensions/1.03%3A_Equations_of_Lines_in_2d
        % [x y]' = d'*t + [x0 y0]'
        d_vec = end_pt - start_pt;
        mid_pt = start_pt + 0.5*d_vec; % find the middle of the edge
        % for each polytope...
        num_polys = length(polytopes);
        p = 1;
        while p <= num_polys
            verts = polytopes(p).vertices;
            % get xmin and xmax also ymin and ymax
            xmax = max(verts(:,1));
            xmin = min(verts(:,1));
            ymax = max(verts(:,2));
            ymin = min(verts(:,2));
            % check axis aligned bounding box before checking for polytope containment of the midpoint
            in_AABB = (mid_pt(1) < xmax && mid_pt(1) > xmin) && (mid_pt(2) < ymax && mid_pt(2) > ymin);
            % is point between xmin xmax and ymin max? if not continue
            if ~in_AABB
                p = p+1;
                continue
            end
            % if point is in AABB make polyshape from these verts
            polyshape_p = polyshape(verts);
            % is point in but not on polyshape?
            [is_in,is_on] = isinterior(polyshape_p,mid_pt(1:2));
            % if so, remove the edge, and stop trying polytopes
            if is_in && ~ is_on
                visibility_matrix(rows_of_1s(e),cols_of_1s(e)) = 0;
                % if it is in one polytope, we needn't check any others
                p = num_polys+1;
            end
            % if not, continue to check the next polytope
            p = p + 1;
        end
    end
end
