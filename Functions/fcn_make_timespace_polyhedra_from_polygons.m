function time_space_polytopes = fcn_make_timespace_polyhedra_from_polygons(shrunk_polytopes, max_translation_distance, final_time)

    for i = 1:length(shrunk_polytopes)
        % rand gives a value between 0 and 1 thus rand-0.5 gives a value between -0.5 and 0.5 thus 2*(rand-0.5)
        % gives a value between -1 and 1 thus we randomize magnitude and direction of the max_translation_distance
        vel_this_poly = 2*[(rand-0.5)*max_translation_distance (rand-0.5)*max_translation_distance];
        num_verts = length(shrunk_polytopes(i).xv);
        vert_ids = (1:1:num_verts)'; % each vertex must be identified as itself as each time
        verts = [shrunk_polytopes(i).xv' shrunk_polytopes(i).yv' 0*ones(num_verts,1) vert_ids;
                 shrunk_polytopes(i).xv'+vel_this_poly(1) shrunk_polytopes(i).yv'+vel_this_poly(2) final_time*ones(num_verts,1) vert_ids];
        time_space_polytopes(i).vertices = verts;
    end
end
