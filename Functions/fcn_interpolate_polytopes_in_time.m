function verts = fcn_interpolate_polytopes_in_time(verts,dt)

    %% interpolation code for a shape
    % TODO @sjharnett when functionalizing this, add the vertex ID column but then remove it
    % for each shape
    % get shape.verts
    % columns of verts are x,y,t,id
    % for number of unique time values in verts...
    unique_times = unique(verts(:,3));
    num_unique_times = length(unique(verts(:,3)));

    unique_verts = unique(verts(:,4));
    num_unique_verts = length(unique(verts(:,4)));

    dense_times = [];
    for i = 2:1:num_unique_times
        new_times = unique_times(i-1):dt:unique_times(i);
        dense_times = [dense_times; new_times'];
    end
    dense_times = unique(dense_times);
    num_dense_times = length(dense_times);

    for i = 1:1:num_unique_verts
        this_vert_id = unique_verts(i);
        this_vert_rows = find(verts(:,4)==this_vert_id);
        this_vert_x = verts(this_vert_rows,1);
        this_vert_y = verts(this_vert_rows,2);
        this_vert_t = verts(this_vert_rows,3);

        this_vert_dense_x = interp1(this_vert_t,this_vert_x,dense_times);
        this_vert_dense_y = interp1(this_vert_t,this_vert_y,dense_times);
        this_vert_id_repeated = ones(num_dense_times,1);
        verts = [verts; this_vert_dense_x this_vert_dense_y dense_times this_vert_id_repeated];
    end

    % will need to remove duplicate rows

    % end for each shape
end
