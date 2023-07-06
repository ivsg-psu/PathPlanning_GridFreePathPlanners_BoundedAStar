function all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes)

    all_surfels = [];
    for i = 1:length(time_space_polytopes)
        flats = time_space_polytopes(i).flats;
        sides = time_space_polytopes(i).sides;
        for j = 1:size(flats,1)
            my_flat = flats(j,:);
            my_flat_matrix = (reshape(my_flat,[4 length(my_flat)/4]))';
            flat_centroid = [mean(my_flat_matrix(:,1)) mean(my_flat_matrix(:,2)) mean(my_flat_matrix(:,3))];
            my_flat_polyshape = polyshape(my_flat_matrix(:,1),my_flat_matrix(:,2));
            my_flat_triangulated = triangulation(my_flat_polyshape);
            tris_this_flat = [];
            for k=1:size(my_flat_triangulated.ConnectivityList,1)
                x1 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,1),1);
                y1 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,1),2);
                x2 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,2),1);
                y2 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,2),2);
                x3 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,3),1);
                y3 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,3),2);
                tris_this_flat = [tris_this_flat; x1 y1 my_flat_matrix(1,3) x2 y2 my_flat_matrix(2,3) x3 y3 my_flat_matrix(3,3)];
            end
            all_surfels = [all_surfels; tris_this_flat];
            % fill3(my_flat_matrix(:,1),my_flat_matrix(:,2),my_flat_matrix(:,3),rand(1,3),'FaceAlpha',0.3);
        end
        for j = 1:size(sides,1)
            my_side = sides(j,:);
            my_side_matrix = (reshape(my_side,[4 length(my_side)/4]))';
            tris_this_side = [my_side_matrix(1,1:3) my_side_matrix(2,1:3) my_side_matrix(3,1:3);...
                              my_side_matrix(1,1:3) my_side_matrix(3,1:3) my_side_matrix(4,1:3)] % 1 2 3, 1 3 4
            all_surfels = [all_surfels; tris_this_side];
            % fill3(my_side_matrix(:,1),my_side_matrix(:,2),my_side_matrix(:,3),rand(1,3),'FaceAlpha',0.3);
        end
    end
end
