function all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes)
% fcn_make_triangular_surfels_from_facets
%
% Decomposes timespace polytopes with facets into triangular surface elements (surfels) which are necessary for
% performing intersection checking between possible visibity graph edges and polytope obstacles.
%
%
%
% FORMAT:
% all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes)
%
% INPUTS:
%
%     time_space_polytopes: this is the time space polytope struct array with
%     both vertices and facets, of the form output by fcn_make_facets_from_verts
%     i.e. a struct array with the following fields:
%       vertices: field holding the vertices of each polytope
%           vertices consists of 4 columns: x position, y position, time (z-axis position) and vertex id
%           the vertex ID is necessary for correctly mapping a vertex at one time to its position at the next time
%       flats: field associating vertices into flat facets (i.e. facets that lie flat in a single time plane)
%       sides: field associating vertices into side wall facets (i.e. facets that can span several time values)
%       both the flats and sides fields have the same format: a matrix where each row is a facet and every 4 columns
%         is one vertex consisting of (x, y, t, and ID) as stated above
%         e.g. two flats with three vertices each would be represented as the following 2x12 matrix:
%         flat 1, vert 1 x, flat 1 vert 1 y, flat 1 vert 1 t, flat 1 vert 1 ID, ..., flat 1, vert 3 x, flat 1 vert 3 y, flat 1 vert 3 t, flat 1 vert 3 ID
%         flat 2, vert 1 x, flat 2 vert 1 y, flat 2 vert 1 t, flat 2 vert 1 ID, ..., flat 2, vert 3 x, flat 2 vert 3 y, flat 2 vert 3 t, flat 2 vert 3 ID
%
%
% OUTPUTS:
%     all_surfels: a matrix with all triangular surface elements (surfels) from all timespace polytopes
%     there is one row for each surfel
%     each row has 9 columns representing the x,y,t coordinates of each point of the triangle ordered
%     x1 y1 t1 x2 y2 t2 x3 y3 t3
%
% DEPENDENCIES:
% generally, fcn_make_facets_from_verts can be run before this function to create facets for this function to triangulate
% see scritp_test_3d_polytope_multiple or the readme for an example of the typical call order
% but there are no dependencies in the source code of this function
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a full test suite.
%
% This function was written on summer 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, summer by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.

    all_surfels = [];
    % loop through each polytope
    for i = 1:length(time_space_polytopes)
        flats = time_space_polytopes(i).flats; % extract flats and sides from polytope
        sides = time_space_polytopes(i).sides;
        % loop through each flat, representing a 2D polytope at one time (see fcn_make_facets_from_verts)
        for j = 1:size(flats,1)
            my_flat = flats(j,:); % store flat of interest
            my_flat_matrix = (reshape(my_flat,[4 length(my_flat)/4]))'; % unwrap flat from a vector into a matrix with columns for (x y t id)
            flat_centroid = [mean(my_flat_matrix(:,1)) mean(my_flat_matrix(:,2)) mean(my_flat_matrix(:,3))]; % find centroid of the flat
            my_flat_polyshape = polyshape(my_flat_matrix(:,1),my_flat_matrix(:,2)); % convert flat polytope to a polyshape
            my_flat_triangulated = triangulation(my_flat_polyshape); % perform Delaunay triangulation on the polyshape
            tris_this_flat = [];
            % each iteration of the following loop takes the outputs of MATLAB's Delaunay triangulation
            % function and reformats them as a matrix of vertices representing the triangles
            for k=1:size(my_flat_triangulated.ConnectivityList,1)
                x1 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,1),1);
                y1 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,1),2);
                x2 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,2),1);
                y2 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,2),2);
                x3 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,3),1);
                y3 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,3),2);
                tris_this_flat = [tris_this_flat; x1 y1 my_flat_matrix(1,3) x2 y2 my_flat_matrix(2,3) x3 y3 my_flat_matrix(3,3)];
            end
            all_surfels = [all_surfels; tris_this_flat]; % add triangles from this flat to all surfel array
            % fill3(my_flat_matrix(:,1),my_flat_matrix(:,2),my_flat_matrix(:,3),rand(1,3),'FaceAlpha',0.3);
        end
        % loop through each side wall of the 3D timespace obstacle
        % triangulating this is simple as each side wall is a parallelogram (see fcn_make_facets_from_verts)
        % thus vertices can just be grouped, there is no need to use Delaunay
        for j = 1:size(sides,1)
            my_side = sides(j,:); % store side of interest
            my_side_matrix = (reshape(my_side,[4 length(my_side)/4]))'; % unwrap side from a vector into a matrix with columns for (x y t id)
            tris_this_side = [my_side_matrix(1,1:3) my_side_matrix(2,1:3) my_side_matrix(3,1:3);...
                              my_side_matrix(1,1:3) my_side_matrix(3,1:3) my_side_matrix(4,1:3)]; % 1 2 3, 1 3 4
            all_surfels = [all_surfels; tris_this_side];
            % fill3(my_side_matrix(:,1),my_side_matrix(:,2),my_side_matrix(:,3),rand(1,3),'FaceAlpha',0.3);
        end
    end
end
