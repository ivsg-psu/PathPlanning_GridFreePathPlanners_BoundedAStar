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
%      (plottingOptions), (figNum))
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
%      plottingOptions: allows user to set plotting options particular to
%      this function. These include:
%          plottingOptions.axis
%          plottingOptions.selectedFromToToPlot: the [from to] edge to plot
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
MAX_NARGIN = 8; % The largest Number of argument inputs to the function
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

% flag_do_debug = 1;

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


% Does user want to specify plottingOptions?
% initialize default values
plottingOptions = struct;
plottingOptions.axis = [];
plottingOptions.selectedFromToToPlot = [];
plottingOptions.filename = [];
if 7 <= nargin
    temp = varargin{1};
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

if ~isempty(plottingOptions.filename)
    delayTime = 0.5; % Delay between frames in seconds
    loopCount = Inf; % Loop indefinitely (0 for no loop)
end

figure(383838)
colorMapMatrixOrString = colormap('turbo');
close(383838);
greenToRedColormap = colorMapMatrixOrString(100:end,:);
Ncolors = 32;
reducedGreenToRedColorMap = fcn_plotRoad_reduceColorMap(greenToRedColormap, Ncolors, -1);
reducedRedToGreenColorMap = flipud(reducedGreenToRedColorMap);
%%%% Uncomment the following two lines to see the colormap
% colormap(reducedRedToGreenColorMap);
% colorbar;

%%  get the primary edge
% get the physical location of the edge start and end
if strcmp(mode, "2d") || strcmp(mode,"2D")
    highestDimension = 2;
elseif strcmp(mode, "3d") || strcmp(mode,"3D")
    highestDimension = 3;
else
    error(strcat("Mode argument must be a string containing '2d' or '3d' (case insensitive) mode was instead given as: ",mode))
end


vgraphNoSelfInteractions = vgraph - eye(size(vgraph)); % we don't want to consider self interactions here
all_pts = [all_pts; start; finish];
Npoints = length(all_pts(:,1));

% initialize to zero value for each edge (zero implying the edge has no corridor width, i.e. is blocked)
dilation_robustness_matrix = zeros(size(vgraphNoSelfInteractions,1),size(vgraphNoSelfInteractions,2),2);
dilation_robustness_matrix_min = nan(Npoints,Npoints);
allEdgeNumbersProcessed = nan(Npoints,Npoints);

% only need to perform this operation for 1's in vgraph, 0's are blocked edges and have 0 corridor width
idx_of_valid_edges = find(vgraphNoSelfInteractions==1);
num_edges = length(idx_of_valid_edges);
% recall vgraph edge ij is the line segment from point i to point j
[edge_start_idx, edge_end_idx] = ind2sub(size(vgraphNoSelfInteractions),idx_of_valid_edges);

% Extract points of correct dimension out of all_pts
allPoints = all_pts(:,1:highestDimension);
edgeStartPoints = allPoints(edge_start_idx,:);
edgeEndPoints   = allPoints(edge_end_idx,:);
edgePolysStart  = all_pts(edge_start_idx,highestDimension+2); % Which polytope the start point is on
edgePolysEnd    = all_pts(edge_end_idx,highestDimension+2); % Which polytope the end point is on
[edgeNextInPolySequence, edgePriorInPolySequence] = fcn_INTERNAL_findAdjacentInSequence(all_pts,highestDimension);
polyOrientations = fcn_INTERNAL_findPolytopeOrientation(all_pts, highestDimension);
edgeVectors     = edgeEndPoints - edgeStartPoints;
edgeVectors_magnitude = sum(edgeVectors.^2,2).^0.5;
unitEdgeVectors = edgeVectors./edgeVectors_magnitude;
unitEdgeNormalVectors = unitEdgeVectors*[0 1; -1 0];
edgeFinalMinWidths = nan(num_edges,1);

% Did user give a specific to/from combo to plot? If so, find the edge
% number that is associated with that combo.
if ~isempty(plottingOptions.selectedFromToToPlot)
    iterationForPlotting = intersect(...
        find(edge_start_idx==plottingOptions.selectedFromToToPlot(1)),...
        find(edge_end_idx==plottingOptions.selectedFromToToPlot(2)));
    loopingRange = iterationForPlotting:iterationForPlotting;
else
    loopingRange = 1:num_edges;
end

% loop through all primary edges
for ith_edge = loopingRange

    thisEdgeStartIndex = edge_start_idx(ith_edge);
    thisEdgeEndIndex   = edge_end_idx(ith_edge);
    thisStartPoint     = edgeStartPoints(ith_edge,:);
    % thisEndPoint       = edgeEndPoints(ith_edge,:);
    thisPolyStart      = edgePolysStart(ith_edge,1);
    thisPolyEnd        = edgePolysEnd(ith_edge,1);

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

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
        end

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

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end

        % Plot the visibiliity graph for all from and to lines
        fcn_Visibility_plotVGraph(vgraphNoSelfInteractions, all_pts, 'g-');
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end

        title('visibility graph');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end

        % Plot the visibiliity graph for this from
        h_plotThisEdgeStartIndex = fcn_Visibility_plotVGraph(vgraphNoSelfInteractions, all_pts, '-',thisEdgeStartIndex);
        set(h_plotThisEdgeStartIndex,'Color',[0 0.5 0],'LineWidth',3);

        % Plot this edge
        h_plotThisEdge = fcn_Visibility_plotVGraph(vgraphNoSelfInteractions, all_pts, '-',[thisEdgeStartIndex thisEdgeEndIndex]);
        set(h_plotThisEdge,'Color',[0.5 0 0],'LineWidth',3);

        % Plot the unit vector and unit normal
        h_xaxis = quiver(thisStartPoint(1,1),thisStartPoint(1,2),unitThisEdgeVector(1,1),unitThisEdgeVector(1,2), 0, '-','Color',0.7*ones(1,3),'LineWidth',2);
        h_yaxis = quiver(thisStartPoint(1,1),thisStartPoint(1,2),unitThisNormalVector(1,1),unitThisNormalVector(1,2), 0, '-','Color',0.7*ones(1,3),'LineWidth',2);
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end

        title(sprintf('visibility graph, only from node %.0f',thisEdgeStartIndex));

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end

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
            edgeVectors(secondary_edge_ends_idx_not_this_edge,1),edgeVectors(secondary_edge_ends_idx_not_this_edge,2),...
            0, '-','Color',0.4*[1 0 0],'LineWidth',1);
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end

        title('Secondary edges');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end

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
    stickOutSecondaryEdges = (secondary_component_in_dir_of_primary <= 0) | ...
        (secondary_component_in_dir_of_primary > thisEdgeVector_magnitude);

    notStickOutSecondaryEdge_idx = secondary_edge_ends_idx_not_this_edge(~stickOutSecondaryEdges);

    notStickOutSecondaryEdgeVectors = edgeVectors(notStickOutSecondaryEdge_idx,:);
    NnotStickOutSecondaryEdgeVectors = length(notStickOutSecondaryEdgeVectors(:,1));

    if 1==flag_do_debug
        figure(debug_fig_num);
        stickOut_idx = secondary_edge_ends_idx_not_this_edge(stickOutSecondaryEdges);
        NstickOut = length(stickOut_idx);
        h_quiverBadEdges = quiver(thisStartPoint(1,1)*ones(NstickOut,1),thisStartPoint(1,2)*ones(NstickOut,1),...
            edgeVectors(stickOut_idx,1),edgeVectors(stickOut_idx,2),...
            0, '-','Color',1*[1 0 0],'LineWidth',3);
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('Bad secondary edges that stick out beyond test edge');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end

        % Shut off visibility of bad data
        set(h_plotThisEdgeStartIndex,'Visible','off')
        set(h_quiverAllSecondary,'Visible','off');
        set(h_quiverBadEdges,'Visible','off')

        % Plot good secondary edges
        h_quiverNotStickOut = ...
            quiver(thisStartPoint(1,1)*ones(NnotStickOutSecondaryEdgeVectors,1),thisStartPoint(1,2)*ones(NnotStickOutSecondaryEdgeVectors,1),...
            edgeVectors(notStickOutSecondaryEdge_idx,1),edgeVectors(notStickOutSecondaryEdge_idx,2),...
            0, '-','Color',0.5*[0 1 0],'LineWidth',3);
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('Secondary edges with stick-outs removed');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end
    end

    % If edge is entirely on same poly, remove secondary edges that are on
    % this same poly
    thisStartingPoly = edgePolysStart(ith_edge);
    thisEndingPoly = edgePolysEnd(ith_edge);
    if thisStartingPoly==thisEndingPoly
        endingPolys  = edgePolysEnd(notStickOutSecondaryEdge_idx);
        flag_notSamePoly_idx = endingPolys~=thisStartingPoly;
        goodSecondaryEdges_idx = notStickOutSecondaryEdge_idx(flag_notSamePoly_idx);
        badSecondaryEdges_idx  = notStickOutSecondaryEdge_idx(~flag_notSamePoly_idx);
    else
        goodSecondaryEdges_idx = notStickOutSecondaryEdge_idx;
        badSecondaryEdges_idx  = [];
    end

    badSecondaryEdgeVectors = edgeVectors(badSecondaryEdges_idx,:);
    goodSecondaryEdgeVectors = edgeVectors(goodSecondaryEdges_idx,:);

    if 1==flag_do_debug
        figure(debug_fig_num);
        NbadSecondary = length(badSecondaryEdgeVectors(:,1));
        h_quiverBadEdges = quiver(thisStartPoint(1,1)*ones(NbadSecondary,1),thisStartPoint(1,2)*ones(NbadSecondary,1),...
            edgeVectors(badSecondaryEdges_idx,1),edgeVectors(badSecondaryEdges_idx,2),...
            0, '-','Color',1*[1 0 0],'LineWidth',3);
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('Bad secondary edges that are on the same poly as start');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end

        % Shut off visibility of old plots
        set(h_quiverNotStickOut,'Visible','off')
        set(h_quiverBadEdges,'Visible','off');

        % Plot good secondary edges
        NgoodSecondary = length(goodSecondaryEdgeVectors(:,1));

        h_quiverGoodSecondary = ...
            quiver(thisStartPoint(1,1)*ones(NgoodSecondary,1),thisStartPoint(1,2)*ones(NgoodSecondary,1),...
            edgeVectors(goodSecondaryEdges_idx,1),edgeVectors(goodSecondaryEdges_idx,2),...
            0, '-','Color',0.5*[0 1 0],'LineWidth',3);
        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('Secondary edges to calculate distances');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end
    end

    %% find if dot product is right or left
    % need to repeat primary edge to vectorize cross product
    NgoodSecondaryVectors = length(goodSecondaryEdges_idx);
    primary_edge_dir_repeated = repmat(thisEdgeVector,NgoodSecondaryVectors,1);
    cross_primary_with_secondary = ...
        cross([primary_edge_dir_repeated,zeros(NgoodSecondaryVectors,1)], ...
        [goodSecondaryEdgeVectors,zeros(NgoodSecondaryVectors,1)], 2);
    cross_primary_with_secondary_direction = cross_primary_with_secondary(:,3);
    flagIsLeft = cross_primary_with_secondary_direction>0;
    flagIsRight = cross_primary_with_secondary_direction<0;

    leftSecondaryEdges_idx = goodSecondaryEdges_idx(flagIsLeft);
    rightSecondaryEdges_idx = goodSecondaryEdges_idx(flagIsRight);

    if 1==flag_do_debug
        figure(debug_fig_num);

        % Shut off visibility of old data
        set(h_quiverGoodSecondary,'Visible','off')

        % Plot left edges in BLUE
        Nleft = length(leftSecondaryEdges_idx);
        h_quiverLeft = ...
            quiver(thisStartPoint(1,1)*ones(Nleft,1),thisStartPoint(1,2)*ones(Nleft,1),...
            edgeVectors(leftSecondaryEdges_idx,1),edgeVectors(leftSecondaryEdges_idx,2),...
            0, '-','Color', [0 0 1],'LineWidth',3);

        % Plot left edges in RED
        Nright = length(rightSecondaryEdges_idx);
        h_quiverRight = ...
            quiver(thisStartPoint(1,1)*ones(Nright,1),thisStartPoint(1,2)*ones(Nright,1),...
            edgeVectors(rightSecondaryEdges_idx,1),edgeVectors(rightSecondaryEdges_idx,2),...
            0, '-','Color', [1 0 0],'LineWidth',3);


        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('Left (blue) and Right (red) Secondary Edges');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
        end
    end

    %% Find corridor widths to left and right
    % dot the other edges with the unit normal to find the corridor width defined by each vgraph edge
    % basically the projection of the secondary edge in the direction normal to the primary edge is what we want
    NgoodSecondary = length(goodSecondaryEdges_idx);
    dot_secondary_with_unit_normal = ...
        sum(ones(NgoodSecondary,1)*unitThisNormalVector.*goodSecondaryEdgeVectors,2);

    dot_secondary_with_unit_normal_left = dot_secondary_with_unit_normal(flagIsLeft);
    dot_secondary_with_unit_normal_right = dot_secondary_with_unit_normal(flagIsRight);


    % the min of projection from all secondary edges is the "closest" secondary edge to the primary, defining
    % the point that would cut off the primary edge
    [corridor_width_left, edgeMinLeft_idx] = min(abs(dot_secondary_with_unit_normal_left));
    edgeMinLeft_idx = leftSecondaryEdges_idx(edgeMinLeft_idx);

    [corridor_width_right,edgeMinRight_idx] = min(abs(dot_secondary_with_unit_normal_right)); % these are negative so switch to positive before taking min
    edgeMinRight_idx = rightSecondaryEdges_idx(edgeMinRight_idx);

    % check for polytope walls
    % OLD (VERY slow)
    % [is_in_left, is_in_right] = fcn_INTERNAL_checkPolytopes(...
    % polytopes, thisStartPoint, thisEdgeVector, unitThisNormalVector);

    % Check if this edge is a polytope wall. This will be the case if the
    % start and end of the test edge is on the same polytope AND the end of
    % the edge is the next in sequence on the polytope
    is_in_left = 0;
    is_in_right = 0;
    flag_isPolyEdge = 0;
    if (thisPolyStart == thisPolyEnd)        
        % If enter here, edge starts and ends on same polytope. The edge
        % may be a side, but it might also connect two vertices of a
        % non-convex polytope. Need to check if the start/end is in a
        % sequence on the polytope, either going up or down.

        % Check if the edge end is in sequence
        if ~isnan(thisEdgeStartIndex) && ~isnan(thisEdgeEndIndex)
            if (thisEdgeEndIndex == edgeNextInPolySequence(thisEdgeStartIndex)) || (thisEdgeEndIndex == edgePriorInPolySequence(thisEdgeStartIndex))
                % If enter here, the edge is in a sequence, which means it is a
                % side of the polytope
                flag_isPolyEdge = 1; % This is a polytope edge

                if polyOrientations(thisEdgeStartIndex,1)>0
                    if (thisEdgeEndIndex == edgeNextInPolySequence(thisEdgeStartIndex))
                        is_in_left = 1;
                    else
                        is_in_right = 1;
                    end
                elseif polyOrientations(thisEdgeStartIndex,1)<0
                    if (thisEdgeEndIndex == edgeNextInPolySequence(thisEdgeStartIndex))
                        is_in_right = 1;
                    else
                        is_in_left = 1;
                    end
                else
                    error('Unknown case entry - polytope found that is neither CCW or CW?')
                end
            end
        end
    end



    if is_in_left
        corridor_width_left = 0;
        % The current edge is its own closest one
        edgeMinLeft_idx = ith_edge;
    elseif isempty(corridor_width_left)
        corridor_width_left = inf;
        % The current edge is its own closest one
        edgeMinLeft_idx = ith_edge;
    end
    
    if is_in_right
        corridor_width_right = 0;
        % The current edge is its own closest one
        edgeMinRight_idx = ith_edge;
    elseif isempty(corridor_width_right)
        corridor_width_right = inf;
        % The current edge is its own closest one
        edgeMinRight_idx = ith_edge;
    end
    
    % thisCorridorWidth = min(corridor_width_left, corridor_width_right);
    thisCorridorWidth = max(corridor_width_left, corridor_width_right);

    if 1==flag_isPolyEdge
        if corridor_width_left == 0
            thisCorridorWidth = corridor_width_right;
        elseif corridor_width_right == 0
            thisCorridorWidth = corridor_width_right;
        else
            error('Entered a situation where edge is on polytope but neither side has zero length. Unexpected situation - unable to continue.');
        end
    end

    dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex, 1) = corridor_width_left;
    dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex, 2) = corridor_width_right;
    dilation_robustness_matrix_min(thisEdgeStartIndex, thisEdgeEndIndex) = thisCorridorWidth;


    allEdgeNumbersProcessed(thisEdgeStartIndex, thisEdgeEndIndex) = ith_edge;
    edgeFinalMinWidths(ith_edge,1) = thisCorridorWidth;

    if 1==flag_do_debug
        figure(debug_fig_num);

        % Shut off visibility of old data
        set(h_quiverLeft,'Visible','off')
        set(h_quiverRight,'Visible','off')

        % Plot left edges in BLUE
        h_quiverLeftLeast = ...
            quiver(thisStartPoint(1,1),thisStartPoint(1,2),...
            edgeVectors(edgeMinLeft_idx,1),edgeVectors(edgeMinLeft_idx,2),...
            0, '-','Color', [0 0 1],'LineWidth',5); %#ok<NASGU>

        % Put dotted line
        leftmostVector = edgeVectors(edgeMinLeft_idx,:);
        thisMagnitude = sum(unitThisEdgeVector.*leftmostVector);
        locationOnThis = thisStartPoint + unitThisEdgeVector*thisMagnitude;
        segmentPoints = [locationOnThis; edgeEndPoints(edgeMinLeft_idx,:)];
        plot(segmentPoints(:,1),segmentPoints(:,2),'--','Color',0.7*[0 0 1]);
        midpoint = mean(segmentPoints,1,'omitmissing');
        text(midpoint(1,1)+addNudge*0.5,midpoint(1,2),sprintf('%.2f',corridor_width_left),'Color',[0 0 1]);

        % Plot left edges in RED
        h_quiverRightLeast = ...
            quiver(thisStartPoint(1,1),thisStartPoint(1,2),...
            edgeVectors(edgeMinRight_idx,1),edgeVectors(edgeMinRight_idx,2),...
            0, '-','Color', [1 0 0],'LineWidth',5); %#ok<NASGU>

        % Put dotted line
        rightmostVector = edgeVectors(edgeMinRight_idx,:);
        thisMagnitude = sum(unitThisEdgeVector.*rightmostVector);
        locationOnThis = thisStartPoint + unitThisEdgeVector*thisMagnitude;
        segmentPoints = [locationOnThis; edgeEndPoints(edgeMinRight_idx,:)];
        plot(segmentPoints(:,1),segmentPoints(:,2),'--','Color',0.7*[1 0 0]);
        midpoint = mean(segmentPoints,1,'omitmissing');
        text(midpoint(1,1)+addNudge*0.5,midpoint(1,2),sprintf('%.2f',corridor_width_right),'Color',[1 0 0]);

        % Make this edge the top one
        uistack(h_plotThisEdge,'top');
        uistack(h_xaxis,'top');
        uistack(h_yaxis,'top');

        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('Left (blue) and Right (red) closest distances');

        drawnow;

        if ~isempty(plottingOptions.filename)
            % Capture the current frame
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
            imwrite(imind, cm, plottingOptions.filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
        end
    end


    if 1==flag_do_debug
        figure(debug_fig_num);

        % Grab all the edges processed so far
        allEdgesProcessedSoFar = allEdgeNumbersProcessed(~isnan(allEdgeNumbersProcessed));
        edgeFinalMinWidthsNotInf = edgeFinalMinWidths(~isinf(edgeFinalMinWidths));
        maxedgeFinalMinWidthsNotInf = max(edgeFinalMinWidthsNotInf);
        normalizedEdgeFinalMinWidths = min(edgeFinalMinWidths/maxedgeFinalMinWidthsNotInf,1);

        NtoPlot = length(allEdgesProcessedSoFar);
        dataToPlot = [];
        for ith_plot = 1:NtoPlot
            edgeToPlot = allEdgesProcessedSoFar(ith_plot,1);
            dataToPlot = [...
                dataToPlot; ...
                edgeStartPoints(edgeToPlot,:) normalizedEdgeFinalMinWidths(edgeToPlot,1); ...
                edgeEndPoints(edgeToPlot,:) normalizedEdgeFinalMinWidths(edgeToPlot,1); ...
                nan(1,highestDimension+1)]; %#ok<AGROW>
        end

        % % Grab the data for this "from"
        % allEdgeNumbersRow = allEdgeNumbersProcessed(thisEdgeStartIndex,:);
        % edgeIndicesToPlot = allEdgeNumbersRow(~isnan(allEdgeNumbersRow));
        % endIndicesToPlot  = edge_end_idx(edgeIndicesToPlot);
        %
        % widthsToRaw = dilation_robustness_matrix_min(thisEdgeStartIndex,endIndicesToPlot);
        % widthsToPlotNotInf = widthsToRaw(~isinf(widthsToRaw));
        % widthsToPlot = widthsToRaw./max(widthsToPlotNotInf);
        %
        % NtoPlot = length(endIndicesToPlot);
        % edgeStartsToPlot  = edgeStartPoints(edgeIndicesToPlot,:);
        % edgeEndsToPlot    = edgeEndPoints(edgeIndicesToPlot,:);
        % dataToPlot = [];
        % for ith_plot = 1:NtoPlot
        %     dataToPlot = [...
        %         dataToPlot; ...
        %         edgeStartsToPlot(ith_plot,:) widthsToPlot(ith_plot,1); ...
        %         edgeEndsToPlot(ith_plot,:) widthsToPlot(ith_plot,1); ...
        %         nan(1,highestDimension+1)]; %#ok<AGROW>
        % end

        clear plotFormat
        plotFormat.LineStyle = '-';
        plotFormat.LineWidth = 5;
        plotFormat.Marker = '.';
        plotFormat.MarkerSize = 10;

        colormap(gca,colorMapMatrixOrString);
        fcn_plotRoad_plotXYI([dataToPlot(:,1) dataToPlot(:,2) dataToPlot(:,3)], (plotFormat), (reducedRedToGreenColorMap), (debug_fig_num));
        title('');

        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end

        % h_colorbar = colorbar;
        % h_colorbar.Ticks = linspace(0, 1, Ncolors) ; %Create ticks from zero to 1
        % % There are 2.23694 mph in 1 m/s
        % colorbarValues   = round(2.23694 * linspace(min(velocities), max(velocities), Ncolors));
        % h_colorbar.TickLabels = num2cell(colorbarValues) ;    %Replace the labels of these 8 ticks with the numbers 1 to 8
        % h_colorbar.Label.String = 'Speed (mph)';

        % title('Left (blue) and Right (red) closest distances');
        % 
        % drawnow;
        % 
        % if ~isempty(plottingOptions.filename)
        %     % Capture the current frame
        %     frame = getframe(gcf);
        %     im = frame2im(frame);
        %     [imind, cm] = rgb2ind(im, 256); % Convert to indexed image
        %     imwrite(imind, cm, plottingOptions.filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
        % end

    end



    % if 1==flag_do_debug
    %     figure(debug_fig_num);
    %     thisEdgeDilationRobustnessMatrix =     dilation_robustness_matrix(thisEdgeStartIndex, thisEdgeEndIndex, :);
    %
    %     % Find the maximum value, not including infinity
    %     max_dilation_robustness_excluding_inf = max(thisEdgeDilationRobustnessMatrix(~isinf(thisEdgeDilationRobustnessMatrix)),[],"all");
    %     normalizedDilationRobustnessMatrix = thisEdgeDilationRobustnessMatrix./max_dilation_robustness_excluding_inf;
    %
    %
    %     for left_or_right = [1,2]
    %
    %         % plot corridor width approximation graph edges
    %         % Plot the polytopes
    %         % figNum = figNum + 1;
    %         % fcn_INTERNAL_plotPolytopes(polytopes, figNum)
    %         if left_or_right==1
    %             title('dilation robustness, left');
    %         else
    %             title('dilation robustness, right');
    %         end
    %
    %         % Plot this result
    %         for j = 1:size(vgraphNoSelfInteractions,1)
    %             if vgraphNoSelfInteractions(ith_edge,j) == 1
    %                 % alpha = dilation_robustness_matrix(i,j,left_or_right)/max_dilation_robustness_excluding_inf;
    %                 alpha = normalizedDilationRobustnessMatrix(ith_edge,j,left_or_right);
    %                 if alpha == inf %| alpha == -inf
    %                     continue % don't plot infinite values
    %                 end
    %                 plot([starts(ith_edge,1),starts(j,1)],[starts(ith_edge,2),starts(j,2)],'-','Color',[alpha 0 1-alpha],'LineWidth',3)
    %             end
    %         end
    %
    %         map = [(linspace(0,1,100))' zeros(100,1) (linspace(1,0,100))'];
    %         colormap(map)
    %         set(gca,'CLim',sort([0 1]*max_dilation_robustness_excluding_inf));
    %         c = colorbar;
    %         c.Label.String = 'dilation robustness';
    %
    %         % plot corridor width approximation values
    %         if flag_do_plot
    %             % figNum = figNum + 1;
    %             figure(figNum); hold on; box on;
    %             for j = 1:length(polytopes)
    %                 fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    %             end
    %             hold on; box on;
    %             xlabel('x [m]');
    %             ylabel('y [m]');
    %             l_or_r_string = {'left','right'};
    %             title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
    %             vgraphNoSelfInteractions = triu(vgraphNoSelfInteractions); % only want to plot upper triangle so bidirectional edges don't plot over each other.
    %             for ith_edge = 1:size(vgraphNoSelfInteractions,1)
    %                 for j = 1:size(vgraphNoSelfInteractions,1)
    %                     % plot only start and finish for assymetry checking
    %                     if ~(ith_edge == start(3) && j == finish(3)) && ~(ith_edge == 7 && j == 8) &&  ~(ith_edge == 15 && j == 16)
    %                         continue
    %                     end
    %                     if vgraphNoSelfInteractions(ith_edge,j) == 1
    %                         % plot a nice gray line
    %                         quiver(starts(ith_edge,1),starts(ith_edge,2),starts(j,1)-starts(ith_edge,1),starts(j,2)-starts(ith_edge,2), 0, '-','Color',0.4*ones(1,3),'LineWidth',2);
    %                         % label the dilation robustness
    %                         text((starts(ith_edge,1)+starts(j,1))/2 ,(starts(ith_edge,2)+starts(j,2))/2, string(dilation_robustness_matrix(ith_edge,j,left_or_right)));
    %                     end
    %                 end % inner vgraph loop
    %             end % outer vgraph loop
    %         end % if do plot loop
    %
    %         % plot corridor width approximation values
    %         if flag_do_plot
    %             % figNum = figNum + 1;
    %             figure(figNum); hold on; box on;
    %             for j = 1:length(polytopes)
    %                 fill(polytopes(j).vertices(:,1)',polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    %             end
    %             hold on; box on;
    %             xlabel('x [m]');
    %             ylabel('y [m]');
    %             l_or_r_string = {'left','right'};
    %             title(strcat('dilation robustness: ',l_or_r_string{left_or_right}));
    %             vgraphNoSelfInteractions = triu(vgraphNoSelfInteractions); % only want to plot upper triangle so bidirectional edges don't plot over each other.
    %             for ith_edge = 1:size(vgraphNoSelfInteractions,1)
    %                 for j = 1:size(vgraphNoSelfInteractions,1)
    %                     % skip start and finish for plotting clarity
    %                     if (ith_edge == start(3) || j == finish(3) || ith_edge == finish(3) || j == start(3))
    %                         continue
    %                     end
    %                     if vgraphNoSelfInteractions(ith_edge,j) == 1
    %                         % plot a nice gray line
    %                         quiver(starts(ith_edge,1),starts(ith_edge,2),starts(j,1)-starts(ith_edge,1),starts(j,2)-starts(ith_edge,2),0,'-','Color',0.4*ones(1,3),'LineWidth',2);
    %                         % label the dilation robustness
    %                         text((starts(ith_edge,1)+starts(j,1))/2 ,(starts(ith_edge,2)+starts(j,2))/2, string(dilation_robustness_matrix(ith_edge,j,left_or_right)));
    %                     end
    %                 end % inner vgraph loop
    %             end % outer vgraph loop
    %         end % if do plot loop
    %     end % left or right loop
    % end % Ends if statement on debug plotting

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

    if 1==flag_rescale_axis
        temp = axis;
        axis([temp(1,1)-1 temp(1,2)+1 temp(1,3)-1 temp(1,4)+1]);
    end

    % Plot the polytopes
    fcn_INTERNAL_plotPolytopes(polytopes, figNum)

    % Plot the start and end points
    plot(start(1),start(2),'.','Color',[0 0.5 0],'MarkerSize',20);
    plot(finish(1),finish(2),'r.','MarkerSize',20);
    text(start(:,1),start(:,2)+addNudge,sprintf('%.0f, Start', Npoints-1));
    text(finish(:,1),finish(:,2)+addNudge,sprintf('%.0f, Finish',Npoints));

    % label point ids for debugging. The last two points are start and
    % finish, so do not need to be plotted and labeled.
    plot(all_pts(1:end-2,1), all_pts(1:end-2,2),'LineStyle','none','Marker','o','MarkerFaceColor',[255,165,0]./255);
    text(all_pts(1:end-2,1)+addNudge,all_pts(1:end-2,2)+addNudge,string(all_pts(1:end-2,3)));

    % Plot the visibiliity graph for all from and to lines
    fcn_Visibility_plotVGraph(vgraphNoSelfInteractions, all_pts, 'g-');
    % Set the axis
    if ~isempty(plottingOptions.axis)
        axis(plottingOptions.axis);
    end

    if ~isempty(plottingOptions.selectedFromToToPlot)
        % Plot the visibiliity graph for this from
        h_plotThisEdgeStartIndex = fcn_Visibility_plotVGraph(vgraphNoSelfInteractions, all_pts, '-',thisEdgeStartIndex);
        set(h_plotThisEdgeStartIndex,'Color',[0 0.5 0],'LineWidth',3);

        % Plot this edge
        h_plotThisEdge = fcn_Visibility_plotVGraph(vgraphNoSelfInteractions, all_pts, '-',[thisEdgeStartIndex thisEdgeEndIndex]);
        set(h_plotThisEdge,'Color',[0.5 0 0],'LineWidth',3);

        % Plot left edges in BLUE
        h_quiverLeftLeast = ...
            quiver(thisStartPoint(1,1),thisStartPoint(1,2),...
            edgeVectors(edgeMinLeft_idx,1),edgeVectors(edgeMinLeft_idx,2),...
            0, '-','Color', [0 0 1],'LineWidth',5); %#ok<NASGU>

        % Put dotted line
        leftmostVector = edgeVectors(edgeMinLeft_idx,:);
        thisMagnitude = sum(unitThisEdgeVector.*leftmostVector);
        locationOnThis = thisStartPoint + unitThisEdgeVector*thisMagnitude;
        segmentPoints = [locationOnThis; edgeEndPoints(edgeMinLeft_idx,:)];
        plot(segmentPoints(:,1),segmentPoints(:,2),'--','Color',0.7*[0 0 1]);
        midpoint = mean(segmentPoints,1,'omitmissing');
        text(midpoint(1,1)+addNudge*0.5,midpoint(1,2),sprintf('%.2f',corridor_width_left),'Color',[0 0 1]);

        % Plot left edges in RED
        h_quiverRightLeast = ...
            quiver(thisStartPoint(1,1),thisStartPoint(1,2),...
            edgeVectors(edgeMinRight_idx,1),edgeVectors(edgeMinRight_idx,2),...
            0, '-','Color', [1 0 0],'LineWidth',5); %#ok<NASGU>

        % Put dotted line
        rightmostVector = edgeVectors(edgeMinRight_idx,:);
        thisMagnitude = sum(unitThisEdgeVector.*rightmostVector);
        locationOnThis = thisStartPoint + unitThisEdgeVector*thisMagnitude;
        segmentPoints = [locationOnThis; edgeEndPoints(edgeMinRight_idx,:)];
        plot(segmentPoints(:,1),segmentPoints(:,2),'--','Color',0.7*[1 0 0]);
        midpoint = mean(segmentPoints,1,'omitmissing');
        text(midpoint(1,1)+addNudge*0.5,midpoint(1,2),sprintf('%.2f',corridor_width_right),'Color',[1 0 0]);


        h_xaxis = quiver(thisStartPoint(1,1),thisStartPoint(1,2),unitThisEdgeVector(1,1),unitThisEdgeVector(1,2), 0, '-','Color',0.7*ones(1,3),'LineWidth',2);
        h_yaxis = quiver(thisStartPoint(1,1),thisStartPoint(1,2),unitThisNormalVector(1,1),unitThisNormalVector(1,2), 0, '-','Color',0.7*ones(1,3),'LineWidth',2);


        % Make this edge the top one
        uistack(h_plotThisEdge,'top');
        uistack(h_xaxis,'top');
        uistack(h_yaxis,'top');

        % Set the axis
        if ~isempty(plottingOptions.axis)
            axis(plottingOptions.axis);
        end
        title('');

        drawnow;
    else
        % Grab all the edges processed so far
        allEdgesProcessedSoFar = allEdgeNumbersProcessed(~isnan(allEdgeNumbersProcessed));
        flagInfiniteEdges = isinf(edgeFinalMinWidths);
        edgeFinalMinWidthsNotInf = edgeFinalMinWidths(~isinf(edgeFinalMinWidths));
        maxedgeFinalMinWidthsNotInf = max(edgeFinalMinWidthsNotInf);
        adjustmentRatio = 1.0;
        adjustedMax = maxedgeFinalMinWidthsNotInf*adjustmentRatio;

        % Create an adjusted minimum width matrix. This is the original
        % width matrix, but with the infinite values converted into the
        % non-infinite max values, plus 10%
        edgeFinalMinWidthsAdjusted = edgeFinalMinWidths;
        edgeFinalMinWidthsAdjusted(flagInfiniteEdges) = adjustedMax;

        % disp([edgeFinalMinWidthsNotInf edge_start_idx edge_end_idx])

        % normalizedEdgeFinalMinWidths = min(edgeFinalMinWidths/maxedgeFinalMinWidthsNotInf,1);
        normalizedEdgeFinalMinWidths = edgeFinalMinWidthsAdjusted/adjustedMax;

        NtoPlot = length(allEdgesProcessedSoFar);
        dataToPlot = [];
        for ith_plot = 1:NtoPlot
            edgeToPlot = allEdgesProcessedSoFar(ith_plot,1);
            dataToPlot = [...
                dataToPlot; ...
                edgeStartPoints(edgeToPlot,:) normalizedEdgeFinalMinWidths(edgeToPlot,1); ...
                edgeEndPoints(edgeToPlot,:) normalizedEdgeFinalMinWidths(edgeToPlot,1); ...
                nan(1,highestDimension+1)]; %#ok<AGROW>
        end

        % % Grab the data for this "from"
        % allEdgeNumbersRow = allEdgeNumbersProcessed(thisEdgeStartIndex,:);
        % edgeIndicesToPlot = allEdgeNumbersRow(~isnan(allEdgeNumbersRow));
        % endIndicesToPlot  = edge_end_idx(edgeIndicesToPlot);
        %
        % widthsToRaw = dilation_robustness_matrix_min(thisEdgeStartIndex,endIndicesToPlot);
        % widthsToPlotNotInf = widthsToRaw(~isinf(widthsToRaw));
        % widthsToPlot = widthsToRaw./max(widthsToPlotNotInf);
        %
        % NtoPlot = length(endIndicesToPlot);
        % edgeStartsToPlot  = edgeStartPoints(edgeIndicesToPlot,:);
        % edgeEndsToPlot    = edgeEndPoints(edgeIndicesToPlot,:);
        % dataToPlot = [];
        % for ith_plot = 1:NtoPlot
        %     dataToPlot = [...
        %         dataToPlot; ...
        %         edgeStartsToPlot(ith_plot,:) widthsToPlot(ith_plot,1); ...
        %         edgeEndsToPlot(ith_plot,:) widthsToPlot(ith_plot,1); ...
        %         nan(1,highestDimension+1)]; %#ok<AGROW>
        % end

        clear plotFormat
        plotFormat.LineStyle = '-';
        plotFormat.LineWidth = 5;
        plotFormat.Marker = '.';
        plotFormat.MarkerSize = 10;

        colormap(gca,colorMapMatrixOrString);
        fcn_plotRoad_plotXYI([dataToPlot(:,1) dataToPlot(:,2) dataToPlot(:,3)], (plotFormat), (reducedRedToGreenColorMap), (figNum));
        title('');

    end

    % Set the axis
    if ~isempty(plottingOptions.axis)
        axis(plottingOptions.axis);
    end


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
    polytopes, thisStartPoint, thisEdgeVector, unitThisNormalVector) %#ok<DEFNU>
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

%% fcn_INTERNAL_findAdjacentInSequence
function [edgeNextInPolySequence, edgePriorInPolySequence] = fcn_INTERNAL_findAdjacentInSequence(all_pts, dimension)
% For each vertex in the polytope, finds the vertex that is next in
% sequence
Npoints = length(all_pts(:,1));
edgeNextInPolySequence = nan(Npoints,1);
edgePriorInPolySequence = nan(Npoints,1);
polysThisPoint = all_pts(:,2+dimension);
for ith_point = 1:Npoints
    thisPoly = polysThisPoint(ith_point,1);
    if thisPoly>0
        % Find all indices in this poly
        thisPolyIndices = find(polysThisPoint == thisPoly);
        if length(thisPolyIndices)<=1
            error('A polytope with 1 index found - unable to continue.');
        end

        % Find the next one
        largerIndices = thisPolyIndices(thisPolyIndices>ith_point);
        if isempty(largerIndices)
            % If empty, then use the first one
            nextIndex = thisPolyIndices(1);
        else
            nextIndex = largerIndices(1);
        end
        edgeNextInPolySequence(ith_point,1)=nextIndex;

        % Find the prior one
        smallerIndices = thisPolyIndices(thisPolyIndices<ith_point);
        if isempty(smallerIndices)
            % If empty, then use the last one
            priorIndex = thisPolyIndices(end);
        else
            priorIndex = smallerIndices(end);
        end
        edgePriorInPolySequence(ith_point,1)=priorIndex;

    end
end


end % ends fcn_INTERNAL_findAdjacentInSequence


%% fcn_INTERNAL_findPolytopeOrientations
function orientation = fcn_INTERNAL_findPolytopeOrientation(all_pts, dimension)
% Finds if a polytope is clockwise or counterClockwise
Npoints = length(all_pts(:,1));
orientation = nan(Npoints,1);
polysThisPoint = all_pts(:,2+dimension);
polysToCheck = unique(polysThisPoint,'legacy');

for ith_poly = 1:length(polysToCheck)
    thisPoly = polysToCheck(ith_poly,1);
    if thisPoly>0
        % Find all indices in this poly
        thisPolyIndices = find(polysThisPoint == thisPoly);
        if length(thisPolyIndices)<=1
            error('A polytope with 1 index found - unable to continue.');
        end

        % Starting at first point, circle until reach last point
        edgePoints = all_pts(thisPolyIndices,1:dimension);
        fullCircleEdgePoints = [edgePoints; edgePoints(1,:)];

        % Take row differences
        edgeVectors = diff(fullCircleEdgePoints,1,1);
        fullCircleEdgeVectors = [edgeVectors; edgeVectors(1,:)];
        magFullCircleEdgeVectors = sum(fullCircleEdgeVectors.^2,2).^0.5;
        unitFullCircleEdgeVectors = fullCircleEdgeVectors./magFullCircleEdgeVectors;

        % Take dot products
        dotProducts = sum(unitFullCircleEdgeVectors(1:end-1,:).*unitFullCircleEdgeVectors(2:end,:),2);
        Ncross = length(unitFullCircleEdgeVectors(1:end-1,:));
        crossProducts = cross([unitFullCircleEdgeVectors(1:end-1,:) zeros(Ncross,1)], [unitFullCircleEdgeVectors(2:end,:) zeros(Ncross,1)]);
        crossSigns = sign(crossProducts(:,3));

        angles = real(acos(dotProducts))*180/pi.*crossSigns;

        angleSum = sum(angles);

        encirclements = round(angleSum)/360;
        if abs(encirclements)~=1
            error('Non-unity encirclement found for a polytope. Unable to determine polytope direction of vertices as CW or CCW!');
        end

        orientation(thisPolyIndices,1) = encirclements;


    end
end


end % ends fcn_INTERNAL_findNextInSequence
