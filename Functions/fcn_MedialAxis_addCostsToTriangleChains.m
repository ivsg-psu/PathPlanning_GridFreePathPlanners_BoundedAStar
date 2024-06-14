function [triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, varargin)
% fcn_MedialAxis_addCostsToTriangleChains
%
% This function appends triangle_chains data structure of the the medial axis graph
% (composed of an adjacency matrix and the triangle_chains structure describing
% the edges) so that the length and approximate corridor width of each edge (i.e., triangle chain)
% are included as a new 4th and 5th columns in the triangle_chains cell array
%
% FORMAT:
%
% [triangle_chains, max_side_lengths_per_tri] = fcn_MedialAxis_addCostsToTriangleChains(triangle_chains, nodes, xcc, ycc, tr, shrunk_polytopes, varargin)
%
%
% INPUTS:
%
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    triangle_chains: an Mx3 cell array with a row for each edge in the medial axis graph.  The first
%      column contains an int for the node ID for the start of the chain.  The second is the end node.
%      The third column is a 1xP_M array of integers representing IDs of the triangles whose circumcenters
%      form the "chain of triangles" connecting the two nodes. P_M can be different for each row, M.
%
%    nodes: a Nx1 array of integers.  The integers are the IDs of the triangles that are 3-connected,
%      i.e., their circumcenters are nodes in the medial axis graph.  The position in the nodes array
%      is the node ID and the value is the triangle ID.  E.g., if nodes(10)=146, then the 10th node
%      in the adjacency_matrix and triangle_chains struct is the 146th triangle in
%      the Delaunay triangulation.
%
%    xcc: Qx1 array of doubles.  The x positions of the circumcenters of the triangles.
%
%    ycc: Qx1 array of doubles.  The y positions of the circumcenters of the triangles.
%
%    tr: Qx3 triangulation.  See: https://www.mathworks.com/help/matlab/ref/triangulation.html#d126e1402901
%
%    shrunk_polytopes: the polytope struct array
%
%   (optional arguments)
%   flag_do_plot: a 1 or 0 for flagging plotting on or off.  If ommitted, it is assumed to be 0.
%
% OUTPUTS:
%    Useful variables for outputs: N - number of nodes, M - number of edges, P_M - number of triangles
%       in the Mth edge, Q - number of triangles in the triangulation.
%
%    triangle_chains: an Mx5 cell array with a row for each edge in the medial axis graph.  The first
%      column contains an int for the node ID for the start of the chain.  The second is the end node.
%      The third column is a 1xP_M array of integers representing IDs of the triangles whose circumcenters
%      form the "chain of triangles" connecting the two nodes. P_M can be different for each row, M.
%      The 4th column contains the estimated corridor width (the minimum lateral free space a vehicle
%      would have when routing down the edge) and the 5th column contains the length of the edge.
%
%    max_side_lengths_per_tri: a Qx1 array of doubles.  The double is the length of the shortest
%      side of each triangle.
%
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
% See the script: script_test_voronoi_planning* for examples of the script in use.
% See ../Documentation/medial_axis_planning.pptx for a flow chart of the medial axis/voronoi planning stack
%
% This function was written Spring 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024, Spring by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.
    %% check input arguments
    if nargin < 6 || nargin > 7
        error('Incorrect number of arguments');
    end
    % if there is no value in varargin...
    if nargin == 6
        % default is to assume convex obstacles as this is conservative
        flag_do_plot = 0;
    end
    % if there is a value in varargin...
    if nargin == 7
        % check what it is
        if varargin{1} == 1
            % set concave flag if it was passed in
            flag_do_plot = 1;
        elseif varargin{1} == 0
            flag_do_plot = 0;
        else
            % throw error if it was passed in with an incorrect value
            error('optional argument is the plotting flag and can either be 1 or 0')
        end
    end
    % to get corridor width:
    % (1) need to get triangle side lengths
    % (2) need to store the max side length of each triangle
    % (3) need to store the min of these max side lengths for each triangle chain
    %% get max length side for each triangle
    max_side_lengths_per_tri = nan(size(tr,1),1); % initialize array to store max side of each tri
    for my_tri = 1:size(tr,1) % loop over each triangle in the triangulation
        my_connectivity = tr.ConnectivityList(my_tri,:); % find the 3 point IDs forming the tri
        my_points = tr.Points(my_connectivity,:); % extract the points forming the tri
        my_verts = [my_points; my_points(1,:)]; % duplicate first point to close the triangle
        side_deltas = diff([my_verts(:,1) my_verts(:,2)]); % find x and y difference between each vertex
        side_lengths = sqrt(side_deltas(:,1).^2 + side_deltas(:,2).^2); % find length from deltaX and deltaY
        max_side_lengths_per_tri(my_tri) = max(side_lengths); % only need to store max length
    end
    %% store route segment length and corridor width for each triangle chain
    for i = 1:(size(triangle_chains,1)) % for each triangle chain
        % pop off a triangle chain
        chain_of_note = triangle_chains{i,3};
        if isempty(chain_of_note) % if the chain was deleted during pruning we skip it
            triangle_chains{i,4} = nan; % just store nan in the corridor width column
            triangle_chains{i,5} = nan; % just store nan in the chain length column
            continue
        end
        % the min of the triangle "sizes" should be the tightest choke in that triangle chain
        triangle_chains{i,4} = min(max_side_lengths_per_tri(chain_of_note));
        % the cumulative length of all the distance between circumcenters is the triangle chain length
        delta_x_and_y = diff([xcc(chain_of_note) ycc(chain_of_note)]);
        triangle_chain_length = sum(sqrt(sum(delta_x_and_y.*delta_x_and_y,2)));
        triangle_chains{i,5} = triangle_chain_length;
    end
    if flag_do_plot
        figure; hold on; box on; title('medial axis graph with corridor width expressed')
        xlabel('x [km]')
        ylabel('y [km]')
        corridor_widths = [triangle_chains{:,4}]';
        corridor_widths(isnan(corridor_widths)) = [];
        max_corridor_width = 0.6*max(corridor_widths);
        min_corridor_width = min(corridor_widths);
        my_colormap = colormap(turbo);
        num_colors = size(my_colormap,1);
        for i = 1:(size(triangle_chains,1))
            % pop off a triangle chain
            chain_of_note = triangle_chains{i,3};
            if isempty(chain_of_note)
                continue
            end
            width_of_note = triangle_chains{i,4};
            if width_of_note > max_corridor_width
                width_of_note = max_corridor_width;
            end
            width_portion = (width_of_note-min_corridor_width)/(max_corridor_width-min_corridor_width);
            if width_portion > 1
                width_portion = 1;
            end
            width_portion_color_idx = round(width_portion*num_colors,0); % convert width_portion to an index in colormap
            if width_portion_color_idx == 0
                width_portion_color_idx = 1; % matlab is 1 indexed
            end
            width_color = my_colormap(width_portion_color_idx,:);
            % plot the medial axis path between them (this is the curved path from the triangle chain)
            plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',width_color)
        end
        set(gca,'CLim',[min_corridor_width max_corridor_width]);
        c = colorbar;
        xlabel('x [km]')
        ylabel('y [km]')
        ylabel(c,'corridor with [km]')
        plot(xcc(nodes(~isnan(nodes))), ycc(nodes(~isnan(nodes))), '.k','MarkerSize',20) % plot 3 connected triangle circumcenters
        for j = 2:length(shrunk_polytopes)
            fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
        end

        figure; hold on; box on; title('medial axis graph with route segment length expressed')
        lengths = [triangle_chains{:,5}]';
        lengths(isnan(lengths)) = [];
        max_length = max(lengths);
        min_length = min(lengths);
        my_colormap = colormap(turbo);
        num_colors = size(my_colormap,1);
        for i = 1:(size(triangle_chains,1))
            % pop off a triangle chain
            chain_of_note = triangle_chains{i,3};
            if isempty(chain_of_note)
                continue
            end
            length_of_note = triangle_chains{i,5};
            if width_of_note > max_corridor_width
                width_of_note = max_corridor_width;
            end
            length_portion = (length_of_note-min_length)/(max_length-min_length);
            if length_portion > 1
                length_portion = 1;
            end
            length_portion_color_idx = round(length_portion*num_colors,0); % convert length_portion to an index in colormap
            if length_portion_color_idx == 0
                length_portion_color_idx = 1; % matlab is 1 indexed
            end
            length_color = my_colormap(length_portion_color_idx,:);
            % plot the medial axis path between them (this is the curved path from the triangle chain)
            plot(xcc(chain_of_note), ycc(chain_of_note), '--','LineWidth',2,'Color',length_color)
        end
        set(gca,'CLim',[min_length max_length]);
        c = colorbar;
        xlabel('x [km]')
        ylabel('y [km]')
        plot(xcc(nodes(~isnan(nodes))), ycc(nodes(~isnan(nodes))), '.k','MarkerSize',20) % plot 3 connected triangle circumcenters
        ylabel(c,'path segment length [km]')
        for j = 2:length(shrunk_polytopes)
            fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
        end
    end % end flag_do_plot
end
