function time_space_polytopes_with_facets = fcn_make_facets_from_verts(time_space_polytopes)
    for i = 1:length(time_space_polytopes)
        verts = time_space_polytopes(i).vertices;
        % verts = [1 1 0 1; 1.5 2 0 2; 2 1 0 3; 2 2 20 1; 2.5 3 20 2; 3 2 20 3;]; % a line that translates its length in x over the course of 20 seconds
        unique_times = unique(verts(:,3));
        num_unique_times = length(unique_times);

        unique_verts = unique(verts(:,4));
        num_unique_verts = length(unique_verts);
        
        % both flats and sides are the number of facets x (number of pts*4)
        % this is because each point is 4-dimensional (x,y,t,id)
        % initialize flats as num times (one flat per time) by num verts*4
        % (each vertex is required to define a flat polygon)
        flats = nan(num_unique_times,num_unique_verts*4);

        % there is a side for ever pair of verts and every pair of times
        % these always have 4 points so the sides is initialized as (num
        % verts * num times - 1) x 4*4 (for qty 4 4-dimensional points)
        sides = nan(num_unique_verts*(num_unique_times-1), 4*4);
        sides_recorded = 1;
        
        % put first time into flats array
        this_time_rows = find(verts(:,3) == unique_times(1));
        flats(1,:) = reshape(verts(this_time_rows,:).',1,[]);
        
        for j = 2:num_unique_times
            % put this time into flats array
            prev_time = unique_times(j-1);
            this_time = unique_times(j);
            this_time_rows = find(verts(:,3) == this_time);
            prev_time_rows = find(verts(:,3) == prev_time);
            flats(j,:) = reshape(verts(this_time_rows,:).',1,[]);
            for k = 1:num_unique_verts
            % for each vertex, go this vert to the next...
                this_vert = unique_verts(k);
                if k == num_unique_verts
                    next_vert = unique_verts(1);
                else
                    next_vert = unique_verts(k+1);
                end
                this_vert_rows = find(verts(:,4) == this_vert);
                next_vert_rows = find(verts(:,4) == next_vert);
                % ...at this time to the previous
                side_wall_idx = [(intersect(this_vert_rows, prev_time_rows)),...
                                 (intersect(next_vert_rows, prev_time_rows)),...
                                 (intersect(next_vert_rows, this_time_rows)),...
                                 (intersect(this_vert_rows, this_time_rows))];

                % and put that in the array sidewall
                sides(sides_recorded,:) = reshape(verts(side_wall_idx,:).',1,[]);
                sides_recorded = sides_recorded + 1;
            end
        end
        time_space_polytopes(i).flats = flats;
        time_space_polytopes(i).sides = sides;
    end
    time_space_polytopes_with_facets = time_space_polytopes;
end
