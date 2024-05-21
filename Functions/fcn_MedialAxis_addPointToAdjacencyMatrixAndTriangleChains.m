function [adjacency_matrix, triangle_chains, nodes, point_closest_tri, point_closest_node] = fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains(point_xy, adjacency_matrix, triangle_chains, nodes, xcc, ycc, max_side_lengths_per_tri)
    % pick a start and finish
    % start = all_pts(24,:);
    % finish = all_pts(104,:);
    % find the xcc,ycc pair closest to start
    % start_xy = [1031.5 -4715.4];
    tris_in_graph = unique([triangle_chains{:,3}]');
    point_delta_from_all_tris = point_xy - [xcc(tris_in_graph), ycc(tris_in_graph)];
    point_dist_from_all_tris = (point_delta_from_all_tris(:,1).^2 + point_delta_from_all_tris(:,2).^2).^0.5;
    [~, point_closest_tri_loc] = min(point_dist_from_all_tris);
    point_closest_tri = tris_in_graph(point_closest_tri_loc);
    % find which chains the closest tri is in (should really only be 2)
    idx_chains_containing_point = [];
    for i = 1:(size(triangle_chains,1))
        % pop off a triangle chain
        chain_of_note = triangle_chains{i,3};
        if ismember(point_closest_tri, chain_of_note)
            idx_chains_containing_point = [idx_chains_containing_point i];
        end
    end
    % make point closest tri a node
    nodes = [nodes; point_closest_tri];
    point_closest_node = find(nodes == point_closest_tri);
    % make a new adjacency matrix row and column for the point triangle
    adjacency_matrix = [adjacency_matrix, zeros(size(adjacency_matrix,2),1); zeros(1,size(adjacency_matrix,1)+1)];
    for i = 1:length(idx_chains_containing_point)
        % pop off the triangle chain containing the point triangle
        first_node = triangle_chains{idx_chains_containing_point(i),1};
        last_node = triangle_chains{idx_chains_containing_point(i),2};
        chain_of_note = triangle_chains{idx_chains_containing_point(i),3};
        % find where the point triangle is in the chain
        point_tri_location = find(chain_of_note == point_closest_tri);
        % make two new chains from beginning to point tri and point tri to end
        first_chain = chain_of_note(1:point_tri_location);
        last_chain = chain_of_note(point_tri_location:end);
        triangle_chains{end+1,1} = first_node;
        triangle_chains{end,2} = point_closest_node;
        triangle_chains{end,3} = first_chain;
        triangle_chains{end,4} = min(max_side_lengths_per_tri(first_chain));
        % the cumulative length of all the distance between circumcenters is the triangle chain length
        delta_x_and_y = diff([xcc(first_chain) ycc(first_chain)]);
        triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
        triangle_chains{end,5} = triangle_chain_length;
        triangle_chains{end+1,1} = point_closest_node;
        triangle_chains{end,2} = last_node;
        triangle_chains{end,3} = last_chain;
        triangle_chains{end,4} = min(max_side_lengths_per_tri(last_chain));
        % the cumulative length of all the distance between circumcenters is the triangle chain length
        delta_x_and_y = diff([xcc(last_chain) ycc(last_chain)]);
        triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
        triangle_chains{end,5} = triangle_chain_length;
        % add the new chains to adjacency
        adjacency_matrix(point_closest_node,point_closest_node) = 1;
        adjacency_matrix(first_node,point_closest_node) = 1;
        adjacency_matrix(point_closest_node,last_node) = 1;
    end
end
