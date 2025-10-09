function [dilation_robustness_matrix] = fcn_BoundedAStar_generateDilationRobustnessMatrix(...
    all_pts, start, finish, vgraph, mode, polytopes, varargin)
% fcn_BoundedAStar_generateDilationRobustnessMatrix
% estimates edge clearances around each edge in a visibility matrix
%
% This function operates on a visibility graph formed from a polytope map
% to estimate the distance, for each vgraph edge, that the polytopes would
% have to be dilated to block that edge. This is similar to corridor width
% except (1) it is only an estimate, the actual corridor around the vgraph
% edge is not measured/calculated and (2) the distance is measured to each
% side independently meaning it is more accurate to think of it as the
% lateral distance from the vgraph edge to the nearest polytope, rather
% than thinking of it as the width of the corridors between polytopes.  For
% a better approximate of corridor width please see the medial axis graph
% structure. see dilation_robustness section of
% Documentation/bounded_astar_documentation.pptx for pseudocode and
% algorithm description
%
% FORMAT:
%
%      dilation_robustness_matrix = ...
%      fcn_BoundedAStar_generateDilationRobustnessMatrix(...
%      all_pts, start, finish, vgraph, mode, polytopes,...
%      (selectedFromToToPlot), (plottingOptions), (figNum)) 
%
% INPUTS:
%
%     start: the start point vector
%         Format:
%         2D: (x,y,point_id, -1, 1)
%         3D: (x,y,z,point_id, -1, 1)
%   
%     finish: the finish point matrix of all valid finishes where each row is a single finish point
%       vector.
%         Format:
%         2D: (x,y,point_id, -1, 1)
%         3D: (x,y,z,point_id, -1, 1)
%   
%     all_pts: the point matrix of all point that can be in the route, except the start and finish where
%         each row is a single point vector. 
%         Format:
%         2D: (x,y,point_id, poly_id, 1 if point is start/end on poly, 0 otherwise)
%         3D: (x,y,z,point_id, poly_id, 1 if point is start/end on poly, 0 otherwise)
%   
%     vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%         A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%   
%     mode: a string for what dimension the inputs are in. The mode argument must be a string with
%       one of the following values:
%         - "3D" - this implies xyz or xyt space
%         - "2D" - this implies xy spatial only dimensions only
%   
%     polytopes: polytope struct array
%        polytopes: an array of structures containing polytope information.
%            
%      (OPTIONAL INPUTS)
%
%      selectedFromToToPlot: an integer denoting the "from" index to
%      highlight. Useful for explaining the code
%
%      plottingOptions: allows user to set plotting options particular to
%      this function. These include:
%          plottingOptions.axis
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      dilation_robustness_matrix - nxnx2 matrix where n is the number of
%      points (nodes) in the map. The value of element i,j,k is the
%      estimated coridor width surrounding the line segment from point i to
%      point j.  The third index, k, is 1 if the free space is measured to
%      the left hand side when facing the j from i, or k = 2 if measured to
%      the right hand side .
%
% DEPENDENCIES:
%
%      (none)
%
% EXAMPLES:
%
% See the script: script_fcn_BoundedAStar_generateDilationRobustnessMatrix
% for a full test suite.
%
% This function was written in January 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu


% REVISION HISTORY:
% As: fcn_algorithm_generate_dilation_robustness_matrix
% January 2024 by Steve Harnett
% -- first write of function
% February 2024 by Steve Harnett
% -- function updated to make a right left distinction using cross products
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% -- function copied to new script from
%    fcn_algorithm_generate_dilation_robustness_matrix.m to follow library
%    conventions
% 2025_10_07 by Sean Brennan
% -- fixed heading strings and function formatting

% TO DO:
%
% -- fill in to-do items here.



%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 9; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %   %   Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %   %   Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS");
    MATLABFLAG_MAPGEN_FLAG_DO_DEBUG = getenv("MATLABFLAG_MAPGEN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS);
    end
end

flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; 
end


%% check input arguments?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (0==flag_max_speed)
    if 1 == flag_check_inputs

        % Are there the right number of inputs?
        narginchk(1,MAX_NARGIN);

        % Check the polytopes input
        % fcn_DebugTools_checkInputsToFunctions(polytopes, 'polytopes');

    end
end

% Does user want to specify selectedFromToToPlot?
selectedFromToToPlot = []; % initialize default values
if 7 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        selectedFromToToPlot = temp;
    end
end

% Does user want to specify plottingOptions?
% initialize default values
plottingOptions = struct;
plottingOptions.axis = []; 
if 8 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        plottingOptions = temp;
    end
end

% Does user want to show the plots?
flag_do_plots = 1; % Default is to show plots
figNum = []; % Empty by default
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        flag_do_plots = 1;
    end
end

addNudge = 0.25;


%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

filename = 'dilationAnimation.gif'; % Specify the output file name
delayTime = 0.5; % Delay between frames in seconds
loopCount = Inf; % Loop indefinitely (0 for no loop)


%%  get the primary edge
% get the physical location of the edge start and end
if strcmp(mode, "2d") || strcmp(mode,"2D")
    highestDimension = 2;
elseif strcmp(mode, "3d") || strcmp(mode,"3D")
    highestDimension = 3;
else
    error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
end


vgraph = vgraph - eye(size(vgraph)); % we don't want to consider self interactions here
all_pts = [all_pts; start; finish];
% initialize to zero value for each edge (zero implying the edge has no corridor width, i.e. is blocked)
dilation_robustness_matrix = zeros(size(vgraph,1),size(vgraph,2),2);
% only need to perform this operation for 1's in vgraph, 0's are blocked edges and have 0 corridor width
idx_of_valid_edges = find(vgraph==1);
num_edges = length(idx_of_valid_edges);
% recall vgraph edge ij is the line segment from point i to point j
[edge_start_idx, edge_end_idx] = ind2sub(size(vgraph),idx_of_valid_edges);

% Extract points of correct dimension out of all_pts
allPoints = all_pts(:,1:highestDimension);
edgeStartPoints = allPoints(edge_start_idx,:);
edgeEndPoints   = allPoints(edge_end_idx,:);
edgePolysStart  = all_pts(edge_start_idx,highestDimension+2);
edgePolysEnd    = all_pts(edge_end_idx,highestDimension+2);
edgeVectors     = edgeEndPoints - edgeStartPoints;
edgeVectors_magnitude = sum(edgeVectors.^2,2).^0.5;
unitEdgeVectors = edgeVectors./edgeVectors_magnitude;
unitEdgeNormalVectors = unitEdgeVectors*[0 1; -1 0];

iterationForPlotting = find(edge_start_idx==1,1);

% loop through all primary edges
for ith_edge = iterationForPlotting:iterationForPlotting
% for ith_edge = 1:num_edges
    thisEdgeStartIndex = edge_start_idx(ith_edge);
    thisEdgeEndIndex   = edge_end_idx(ith_edge);
    thisStartPoint     = edgeStartPoints(ith_edge,:);
    thisEndPoint       = edgeEndPoints(ith_edge,:);

    % find edge direction vectors
    thisEdgeVector = edgeVectors(ith_edge,:);
    thisEdgeVector_magnitude = edgeVectors_magnitude(ith_edge,:);
    unitThisEdgeVector = unitEdgeVectors(ith_edge,:);
    unitThisNormalVector = unitEdgeNormalVectors(ith_edge,:);

    if 1==flag_do_debug
        figure(debug_fig_num); 
        clf;

        % Plot the polytopes
        fcn_INTERNAL_plotPolytopes(polytopes, debug_fig_num)

        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('polytope map')

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);


        % Plot the start and end points
        plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
        plot(finish(1),finish(2),'r.','MarkerSize',20);
        text(start(:,1),start(:,2)+addNudge,'Start');
        text(finish(:,1),finish(:,2)+addNudge,'Finish');

        % label point ids for debugging. The last two points are start and
        % finish, so do not need to be plotted and labeled.
        plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
        text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));
        title('polytope map with numbered vertices')

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);


        % Plot the visibiliity graph for all from and to lines
        fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-');
        title('visibility graph');

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);

        % Plot the visibiliity graph for this from
        h_plotThisEdgeStartIndex = fcn_Visibility_plotVGraph(vgraph, all_pts, '-',thisEdgeStartIndex);
        set(h_plotThisEdgeStartIndex,'Color',[0 0.5 0],'LineWidth',3);

        % Plot this edge 
        h_plotThisEdge = fcn_Visibility_plotVGraph(vgraph, all_pts, '-',[thisEdgeStartIndex thisEdgeEndIndex]);
        set(h_plotThisEdge,'Color',[0.5 0 0],'LineWidth',3);

        % Plot the unit vector and unit normal
        quiver(thisStartPoint(1,1),thisStartPoint(1,2),unitThisEdgeVector(1,1),unitThisEdgeVector(1,2), 0, '-','Color',0.7*ones(1,3),'LineWidth',2);
        quiver(thisStartPoint(1,1),thisStartPoint(1,2),unitThisNormalVector(1,1),unitThisNormalVector(1,2), 0, '-','Color',0.7*ones(1,3),'LineWidth',2);

        title(sprintf('visibility graph, only from node %.0f',thisEdgeStartIndex));

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);


    end

    %% get the unit normal of the primary edge
    % calculate direction normal to edge
    if abs(thisEdgeVector_magnitude) == 0
        warning("visibility graph edge has zero length")
        corridor_width = inf; % for an edge of zero length, routing down the edge
        % is equivalent to staying still which should have no corridor width restriction as
        % staying still is "free" kinematically
        dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex,:) = corridor_width; % set this to both the left and right
        continue % skip to next edge if this edge had zero length
    end

    %% get the secondary edges
    % find all visibility graph edges with same origin (these are the secondary edges)
    secondary_edge_ends_idx = find(edge_start_idx==thisEdgeStartIndex);
    % remove the primary edge from the list of secondary edges
    % i.e., we don't want to use the same edge as the primary and secondary edge because there is
    % no corridor width (i.e., lateral spacing) between an edge and itself
    secondary_edge_ends_idx_not_this_edge = secondary_edge_ends_idx(secondary_edge_ends_idx~=ith_edge);
    
    % find the secondary edge direction vectors
    secondary_edge_vectors = edgeVectors(secondary_edge_ends_idx_not_this_edge,:);
    Nsecondary = length(secondary_edge_ends_idx_not_this_edge(:,1));
    if 1==flag_do_debug
        figure(debug_fig_num); 
        h_quiverAllSecondary = ...
            quiver(thisStartPoint(1,1)*ones(Nsecondary,1),thisStartPoint(1,2)*ones(Nsecondary,1),...
            secondary_edge_vectors(:,1),secondary_edge_vectors(:,2),...
            0, '-','Color',0.4*[1 0 0],'LineWidth',1);

        title('Secondary edges');

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);

    end


    %% discard irrelevant secondary edges
    % we want to discard edges that either have no component in the direction of the original vector
    % or that end at a point too far away to cut off the original vector


    % dot the unit vector in the direction of the primary edge with the secondary edge    
    % to get the component of the secondary vector in the direction of the primary vector    
    secondary_component_in_dir_of_primary = sum(ones(Nsecondary,1)*unitThisEdgeVector.*secondary_edge_vectors,2);

    % if the result is negative, the secondary edge ends "behind" the primary edge
    % if the result is longer than the primary edge it ends "after" the primary edge
    % both cases are discarded
    badSecondaryEdges = (secondary_component_in_dir_of_primary <= 0) | ...
        (secondary_component_in_dir_of_primary > thisEdgeVector_magnitude);
    goodSecondaryEdge_idx = find(~badSecondaryEdges);
    goodSecondaryEdgeVectors = secondary_edge_vectors(goodSecondaryEdge_idx,:);
    NgoodSecondaryEdgeVectors = length(goodSecondaryEdgeVectors(:,1));

    if 1==flag_do_debug
        figure(debug_fig_num); 
        badIndices = find(badSecondaryEdges);
        Nbad = length(badIndices);
        h_quiverBadEdges = quiver(thisStartPoint(1,1)*ones(Nbad,1),thisStartPoint(1,2)*ones(Nbad,1),...
            secondary_edge_vectors(badIndices,1),secondary_edge_vectors(badIndices,2),...
            0, '-','Color',1*[1 0 0],'LineWidth',3);

        title('Bad secondary edges');

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);


        % Shut off visibility of bad data
        set(h_plotThisEdgeStartIndex,'Visible','off')
        set(h_quiverAllSecondary,'Visible','off');
        set(h_quiverBadEdges,'Visible','off')

        % Plot good secondary edges
        h_quiverGoodSecondary = ...
            quiver(thisStartPoint(1,1)*ones(NgoodSecondaryEdgeVectors,1),thisStartPoint(1,2)*ones(NgoodSecondaryEdgeVectors,1),...
            goodSecondaryEdgeVectors(:,1),goodSecondaryEdgeVectors(:,2),...
            0, '-','Color',0.5*[0 1 0],'LineWidth',3);

        title('Good secondary edges');

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);

    end

    % Remove secondary edges that are on this same poly
    startingPoly = edgePolysStart();
    goodSecondaryEdge_idx

    %% find if dot product is right or left
    % need to repeat primary edge to vectorize cross product
    primary_edge_dir_repeated = repmat(thisEdgeVector,NgoodSecondaryEdgeVectors,1); 
    cross_primary_with_secondary = ...
        cross([primary_edge_dir_repeated,zeros(NgoodSecondaryEdgeVectors,1)], ...
        [goodSecondaryEdgeVectors,zeros(NgoodSecondaryEdgeVectors,1)], 2);
    cross_primary_with_secondary_direction = cross_primary_with_secondary(:,3);
    flagIsLeft = cross_primary_with_secondary_direction>0;
    flagIsRight = cross_primary_with_secondary_direction<0;

    if 1==flag_do_debug
        figure(debug_fig_num); 
        Nleft = length(find(flagIsLeft));
        h_quiverLeft = quiver(thisStartPoint(1,1)*ones(Nleft,1),thisStartPoint(1,2)*ones(Nleft,1),...
            goodSecondaryEdgeVectors(flagIsLeft,1),goodSecondaryEdgeVectors(flagIsLeft,2),...
            0, '-','Color',1*[0 0 1],'LineWidth',3);

        Nright = length(find(flagIsLeft));
        h_quiverRight = quiver(thisStartPoint(1,1)*ones(Nright,1),thisStartPoint(1,2)*ones(Nright,1),...
            goodSecondaryEdgeVectors(flagIsLeft,1),goodSecondaryEdgeVectors(flagIsLeft,2),...
            0, '-','Color',1*[0 0 1],'LineWidth',3);

        title('Left and Right secondary edges');

        drawnow;
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);

    end

    URHERE

    % dot the other edges with the unit normal to find the corridor width defined by each vgraph edge
    % basically the projection of the secondary edge in the direction normal to the primary edge is what we want
    dot_secondary_with_unit_normal = ...
        sum(ones(NgoodSecondaryEdgeVectors,1)*unitThisNormalVector.*goodSecondaryEdgeVectors,2);

    dot_secondary_with_unit_normal_left = dot_secondary_with_unit_normal(flagIsLeft);
    dot_secondary_with_unit_normal_right = dot_secondary_with_unit_normal(flagIsRight);


    % the min of projection from all secondary edges is the "closest" secondary edge to the primary, defining
    % the point that would cut off the primary edge
    corridor_width_left = min(abs(dot_secondary_with_unit_normal_left));
    corridor_width_right = min(abs(dot_secondary_with_unit_normal_right)); % these are negative so switch to positive before taking min

    % check for polytope walls
    [is_in_left, is_in_right] = fcn_INTERNAL_checkPolytopes(...
    polytopes, thisStartPoint, thisEdgeVector, unitThisNormalVector);

    if isempty(corridor_width_left)
        if is_in_left
            corridor_width_left = 0;
        else
            corridor_width_left = inf;
        end
    end
    if isempty(corridor_width_right)
        if is_in_right
            corridor_width_right = 0;
        else
            corridor_width_right = inf;
        end
    end
    dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex, 1) = corridor_width_left;
    dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex, 2) = corridor_width_right;

    if 1==flag_do_debug
        figure(debug_fig_num);
        thisEdgeDilationRobustnessMatrix =     dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex, :);
        
        % Find the maximum value, not including infinity
        max_dilation_robustness_excluding_inf = max(thisEdgeDilationRobustnessMatrix(~isinf(thisEdgeDilationRobustnessMatrix)),[],"all");        
        normalizedDilationRobustnessMatrix = thisEdgeDilationRobustnessMatrix./max_dilation_robustness_excluding_inf;


        for left_or_right = [1,2]

            % plot corridor width approximation graph edges
            % Plot the polytopes
            % figNum = figNum + 1;
            % fcn_INTERNAL_plotPolytopes(polytopes, figNum)
            if left_or_right==1
                title('dilation robustness, left');
            else
                title('dilation robustness, right');
            end

            % Plot this result
            for j = 1:size(vgraph,1)
                if vgraph(ith_edge,j) == 1
                    % alpha = dilation_robustness_matrix(i,j,left_or_right)/max_dilation_robustness_excluding_inf;
                    alpha = normalizedDilationRobustnessMatrix(ith_edge,j,left_or_right);
                    if alpha == inf %| alpha == -inf
                        continue % don't plot infinite values
                    end
                    plot([starts(ith_edge,1),starts(j,1)],[starts(ith_edge,2),starts(j,2)],'-','Color',[alpha 0 1-alpha],'LineWidth',3)
                end
            end

            map = [(linspace(0,1,100))' zeros(100,1) (linspace(1,0,100))'];
            colormap(map)
            set(gca,'CLim',sort([0 1]*max_dilation_robustness_excluding_inf));
            c = colorbar;
            c.Label.String = 'dilation robustness';

            % plot corridor width approximation values
            if flag_do_plot
                % figNum = figNum + 1;
                figure(figNum); hold on; box on;
                for j = 1:length(polytopes)
                    fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
                end
                hold on; box on;
                xlabel('x [m]');
                ylabel('y [m]');
                l_or_r_string = {'left','right'};
                title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
                vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
                for ith_edge = 1:size(vgraph,1)
                    for j = 1:size(vgraph,1)
                        % plot only start and finish for assymetry checking
                        if ~(ith_edge == start(3) && j == finish(3)) && ~(ith_edge == 7 && j == 8) &&  ~(ith_edge == 15 && j == 16)
                            continue
                        end
                        if vgraph(ith_edge,j) == 1
                            % plot a nice gray line
                            quiver(starts(ith_edge,1),starts(ith_edge,2),starts(j,1)-starts(ith_edge,1),starts(j,2)-starts(ith_edge,2), 0, '-','Color',0.4*ones(1,3),'LineWidth',2);
                            % label the dilation robustness
                            text((starts(ith_edge,1)+starts(j,1))/2 ,(starts(ith_edge,2)+starts(j,2))/2, string(dilation_robustness_matrix(ith_edge,j,left_or_right)));
                        end
                    end % inner vgraph loop
                end % outer vgraph loop
            end % if do plot loop

            % plot corridor width approximation values
            if flag_do_plot
                % figNum = figNum + 1;
                figure(figNum); hold on; box on;
                for j = 1:length(polytopes)
                    fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
                end
                hold on; box on;
                xlabel('x [m]');
                ylabel('y [m]');
                l_or_r_string = {'left','right'};
                title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
                vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
                for ith_edge = 1:size(vgraph,1)
                    for j = 1:size(vgraph,1)
                        % skip start and finish for plotting clarity
                        if (ith_edge == start(3) || j == finish(3) || ith_edge == finish(3) || j == start(3))
                            continue
                        end
                        if vgraph(ith_edge,j) == 1
                            % plot a nice gray line
                            quiver(starts(ith_edge,1),starts(ith_edge,2),starts(j,1)-starts(ith_edge,1),starts(j,2)-starts(ith_edge,2),0,'-','Color',0.4*ones(1,3),'LineWidth',2);
                            % label the dilation robustness
                            text((starts(ith_edge,1)+starts(j,1))/2 ,(starts(ith_edge,2)+starts(j,2))/2, string(dilation_robustness_matrix(ith_edge,j,left_or_right)));
                        end
                    end % inner vgraph loop
                end % outer vgraph loop
            end % if do plot loop
        end % left or right loop
    end % Ends if statement on debug plotting

end % Ends looping through edges


%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _                 
%  |  __ \     | |                
%  | |  | | ___| |__  _   _  __ _ 
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_plots
    % check whether the figure already has data
    temp_h = figure(figNum);
    flag_rescale_axis = 0; 
    if isempty(get(temp_h,'Children'))
        flag_rescale_axis = 0; % Set to 1 to force rescaling
    end        

    % Plot the polytopes
    % figNum = figNum + 1;
    fcn_INTERNAL_plotPolytopes(polytopes, figNum)
    if ~isempty(plottingOptions.axis)
        axis(plottingOptions.axis);
    end
    title('polytope map')

    % Plot the start and end points
    plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
    plot(finish(1),finish(2),'r.','MarkerSize',20);
    text(start(:,1),start(:,2)+addNudge,'Start');
    text(finish(:,1),finish(:,2)+addNudge,'Finish');

    % label point ids for debugging. The last two points are start and
    % finish, so do not need to be plotted and labeled.
    plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
    text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));
    title('polytope map with numbered vertices')


    % Plot the visibiliity graph
    fcn_Visibility_plotVGraph(vgraph, all_pts, 'g-');
    title('visibility graph');


    if ~isempty(selectedFromToToPlot)
        % Plot portion of visibility graph for selectedFromToToPlot
        for ith_edge = selectedFromToToPlot
            for j = 1:size(vgraph,1)
                if vgraph(ith_edge,j) == 1
                    plot([starts(ith_edge,1),starts(j,1)],[starts(ith_edge,2),starts(j,2)],'-','Color',[0 0.5 0],'LineWidth',3)
                end
                if vgraph(ith_edge,j) == 0
                    % plot([starts(i,1),starts(j,1)],[starts(i,2),starts(j,2)],'--r','LineWidth',2)
                end
            end
        end
        title(sprintf('visibility graph, only from node %.0f',selectedFromToToPlot));
    end


    % Find the maximum value, not including infinity
    % Select left or right maxima?
    if 1==0
        dilation_robustness_values = dilation_robustness_matrix(:,:,left_or_right)';
        dilation_robustness_values = dilation_robustness_values(:)';
        max_dilation_robustness_excluding_inf = max(dilation_robustness_values(~isinf(dilation_robustness_values) & ~isinf(-dilation_robustness_values)));
    else
        max_dilation_robustness_excluding_inf = max(dilation_robustness_matrix(~isinf(dilation_robustness_matrix)),[],"all");
    end
    normalizedDilationRobustnessMatrix = dilation_robustness_matrix./max_dilation_robustness_excluding_inf;


    for left_or_right = [1,2]

        % plot corridor width approximation graph edges
        % Plot the polytopes
        % figNum = figNum + 1;
        % fcn_INTERNAL_plotPolytopes(polytopes, figNum)
        if left_or_right==1
            title('dilation robustness, left');
        else
            title('dilation robustness, right');
        end

        for ith_edge = 1:size(vgraph,1)
            for j = 1:size(vgraph,1)
                if vgraph(ith_edge,j) == 1
                    % alpha = dilation_robustness_matrix(i,j,left_or_right)/max_dilation_robustness_excluding_inf;
                    alpha = normalizedDilationRobustnessMatrix(ith_edge,j,left_or_right);
                    if alpha == inf %| alpha == -inf
                        continue % don't plot infinite values
                    end
                    plot([starts(ith_edge,1),starts(j,1)],[starts(ith_edge,2),starts(j,2)],'-','Color',[alpha 0 1-alpha],'LineWidth',3)
                end
            end
        end
        map = [(linspace(0,1,100))' zeros(100,1) (linspace(1,0,100))'];
        colormap(map)
        set(gca,'CLim',sort([0 1]*max_dilation_robustness_excluding_inf));
        c = colorbar;
        c.Label.String = 'dilation robustness';

        % plot corridor width approximation values
        if flag_do_plot
            % figNum = figNum + 1;
            figure(figNum); hold on; box on;
            for j = 1:length(polytopes)
                fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
            end
            hold on; box on;
            xlabel('x [m]');
            ylabel('y [m]');
            l_or_r_string = {'left','right'};
            title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
            vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
            for ith_edge = 1:size(vgraph,1)
                for j = 1:size(vgraph,1)
                    % plot only start and finish for assymetry checking
                    if ~(ith_edge == start(3) && j == finish(3)) && ~(ith_edge == 7 && j == 8) &&  ~(ith_edge == 15 && j == 16)
                        continue
                    end
                    if vgraph(ith_edge,j) == 1
                        % plot a nice gray line
                        quiver(starts(ith_edge,1),starts(ith_edge,2),starts(j,1)-starts(ith_edge,1),starts(j,2)-starts(ith_edge,2), 0, '-','Color',0.4*ones(1,3),'LineWidth',2);
                        % label the dilation robustness
                        text((starts(ith_edge,1)+starts(j,1))/2 ,(starts(ith_edge,2)+starts(j,2))/2, string(dilation_robustness_matrix(ith_edge,j,left_or_right)));
                    end
                end % inner vgraph loop
            end % outer vgraph loop
        end % if do plot loop

        % plot corridor width approximation values
        if flag_do_plot
            % figNum = figNum + 1;
            figure(figNum); hold on; box on;
            for j = 1:length(polytopes)
                fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
            end
            hold on; box on;
            xlabel('x [m]');
            ylabel('y [m]');
            l_or_r_string = {'left','right'};
            title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
            vgraph = triu(vgraph); % only want to plot upper triangle so bidirectional edges don't plot over each other.
            for ith_edge = 1:size(vgraph,1)
                for j = 1:size(vgraph,1)
                    % skip start and finish for plotting clarity
                    if (ith_edge == start(3) || j == finish(3) || ith_edge == finish(3) || j == start(3))
                        continue
                    end
                    if vgraph(ith_edge,j) == 1
                        % plot a nice gray line
                        quiver(starts(ith_edge,1),starts(ith_edge,2),starts(j,1)-starts(ith_edge,1),starts(j,2)-starts(ith_edge,2),0,'-','Color',0.4*ones(1,3),'LineWidth',2);
                        % label the dilation robustness
                        text((starts(ith_edge,1)+starts(j,1))/2 ,(starts(ith_edge,2)+starts(j,2))/2, string(dilation_robustness_matrix(ith_edge,j,left_or_right)));
                    end
                end % inner vgraph loop
            end % outer vgraph loop
        end % if do plot loop
    end % left or right loop


    % 
    % % make plots
    % if formatting_type==1
    %     finalPlotFormat = fcn_DebugTools_extractPlotFormatFromString(plotFormat, (-1));
    % elseif formatting_type==2
    %     finalPlotFormat.Color = plotFormat;
    % elseif formatting_type==3        
    %     finalPlotFormat = plotFormat;
    % else
    %     warning('on','backtrace');
    %     warning('An unkown input format is detected in the main code - throwing an error.')
    %     error('Unknown plot type')
    % end
    % 
    % % If plotting only one point, make sure point style is filled
    % NplotPoints = length(polytope_plot_data_x(:,1));
    % if NplotPoints==1
    %     if ~isfield(plotFormat,'Marker') || strcmp(plotFormat.Marker,'none')
    %         finalPlotFormat.Marker = '.';
    %         finalPlotFormat.LineStyle = 'none';
    %     end
    % end
    % 
    % 
    % % Do plotting
    % % Plot polytope as filled object, using 'fill'
    % if fillFormat(1) == 1
    %     for polys = 1:size(polytopes,2)
    %         filler = fill(polytopes(polys).vertices(:,1)',polytopes(polys).vertices(:,2)',fillFormat(2:4));
    %         filler.FaceAlpha = polytopes(polys).cost;
    %     end
    % end
    % 
    % 
    % % Plot polytope edges depending on line style
    % h_plot = plot(polytope_plot_data_x,polytope_plot_data_y);    
    % list_fieldNames = fieldnames(finalPlotFormat);
    % for ith_field = 1:length(list_fieldNames)
    %     thisField = list_fieldNames{ith_field};
    %     h_plot.(thisField) = finalPlotFormat.(thisField);
    % end
    % 
    % % Make axis slightly larger?
    % if flag_rescale_axis
    %     temp = axis;
    %     %     temp = [min(points(:,1)) max(points(:,1)) min(points(:,2)) max(points(:,2))];
    %     axis_range_x = temp(2)-temp(1);
    %     axis_range_y = temp(4)-temp(3);
    %     percent_larger = 0.3;
    %     axis([temp(1)-percent_larger*axis_range_x, temp(2)+percent_larger*axis_range_x,  temp(3)-percent_larger*axis_range_y, temp(4)+percent_larger*axis_range_y]);
    % end
    % 
    % axis equal
    % axis tight

end % Ends check if plotting

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends main function

%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%% fcn_INTERNAL_plotPolytopes
function fcn_INTERNAL_plotPolytopes(polytopes, figNum)
% A wrapper function for plotPolytopes, to plot the polytopes with same
% format

% axes_limits = [0 1 0 1]; % x and y axes limits
% axis_style = 'square'; % plot axes style
plotFormat.Color = 'Blue'; % edge line plotting
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 2; % linewidth of the edge
fillFormat = [1 0 0 1 0.4];
% FORMAT: fcn_MapGen_plotPolytopes(polytopes,fig_num,line_spec,line_width,axes_limits,axis_style);
fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(figNum));
hold on
box on
% axis([-0.1 1.1 -0.1 1.1]);
xlabel('x [m]');
ylabel('y [m]');
end % Ends fcn_INTERNAL_plotPolytopes

%% fcn_INTERNAL_checkPolytopes
function [is_in_left, is_in_right, is_on_left, is_on_right] = ...
    fcn_INTERNAL_checkPolytopes(...
    polytopes, thisStartPoint, thisEdgeVector, unitThisNormalVector)
% this check is based on the definition of an interior point:
% https://wiki.math.ntnu.no/linearmethods/basicspaces/openandclosed
% i.e. that a point is interior if all points within an epsilon sized
% ball centered at that point is also in the set. This may break due to
% numerical precision issues but it is more robust than the other way
% of checking for interior points: assuming adjascent vertices in the
% vertex list define a polytope side.  This has two issues:
%   1) we can't tell if the polytope is defined CCW or CW so the wall
%   can be on the left or right 2) if the polytope is not formed by
%   drawing lines between vertecies and instead by taking the convex
%   hull
%      of the vertices, then adjascent points in the vertex list may
%      not be a polytope wall
% if there are no dot products, there's either nothing to that side, so
% the space is infinte or there's a polytope to that side so the width
% is zero

% find a point to the left and a point to the right of the primary edge
% first want to find if unit vector points left or right of primary vector
primary_edge_mid_point = thisStartPoint + 0.5*thisEdgeVector; % find the middle of the edge
cross_primary_with_normal = cross([thisEdgeVector 0],[unitThisNormalVector 0]);

my_eps = 1e-7;
if cross_primary_with_normal(3) > 0 % if cross is negative, unit vector points left
    % thus a point to the left, starting from the midpoint of the primary edge is eps in the normal direction
    point_to_left = primary_edge_mid_point + my_eps*unitThisNormalVector;
    point_to_right = primary_edge_mid_point - my_eps*unitThisNormalVector;
elseif cross_primary_with_normal(3) < 0
    point_to_right = primary_edge_mid_point + my_eps*unitThisNormalVector;
    point_to_left = primary_edge_mid_point - my_eps*unitThisNormalVector;
else
    error('primary edge crossed with unit normal is 0')
    % just make the midpoint the point in this case
end

% now that we have a point on either side of the primary edge, check these points to see if they are in polytopes
num_polys = length(polytopes);
p = 1;
is_in_left = 0; % initialize to false
is_in_right = 0;
% loop over all polys
while p <= num_polys
    verts = polytopes(p).vertices;
    % get xmin and xmax also ymin and ymax
    xmax = max(verts(:,1));
    xmin = min(verts(:,1));
    ymax = max(verts(:,2));
    ymin = min(verts(:,2));
    % check axis aligned bounding box before checking for polytope containment of the midpoint
    in_AABB = (primary_edge_mid_point(1) <= xmax && primary_edge_mid_point(1) >= xmin) && (primary_edge_mid_point(2) <= ymax && primary_edge_mid_point(2) >= ymin);
    % is point between xmin xmax and ymin max? if not continue
    if ~in_AABB
        p = p+1;
        continue
    end

    % if point is in AABB make polyshape from these verts
    polyshape_p = polyshape(verts);
    % is point in but not on polyshape?
    [is_in,is_on] = isinterior(polyshape_p,primary_edge_mid_point);
    % if the midpoint is on a polytope, the edge boarders a polytope so we need to check
    % where the polytope is relative to the edge by testing the right and left points
    if is_on
        [is_in_left,is_on_left] = isinterior(polyshape_p,point_to_left);
        [is_in_right,is_on_right] = isinterior(polyshape_p,point_to_right);
        if is_in_left && is_in_right
            error('the point is somehow left and right of the polytope')
        end
        % if it is in one polytope, we needn't check any others
        p = num_polys+1; % this will cause the loop to 
    elseif is_in
        % Do nothing, but can add code here later to handle this case
    end
    % if not, continue to check the next polytope
    p = p + 1;
end

end % Ends fcn_INTERNAL_checkPolytopes
