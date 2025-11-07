function [costGraph, visibilityGraph] = fcn_VGraph_findEdgeWeights(polytopes, pointsWithData, gapSize, varargin)
% fcn_VGraph_findEdgeWeights
%
% Finds costs associated with edges taking into account to/from pairs that
% are on the same polytope, thus allowing "through" costs rather than
% perimeter "go around" costs. The goal is to allow path planners to cut
% through obstacles, at least from vertex to vertex, if cutting through is
% less costly than going around. NOTE: this function is not yet done.
%
% FORMAT:
% [costGraph, visibilityGraph] = fcn_VGraph_findEdgeWeights...
% (polytopes, pointsWithData, gapSize, (figNum))
%
% INPUTS:
%
%   polytopes:
%
%   pointsWithData: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector with data
%           x-coordinate
%           y-coordinate
%           point id
%           obstacle id (-1 if none)
%           is beginning/end of polytope (1 if yes, 0 if no)
%
%   gapSize: size of gaps between polytopes
%
%   (optional inputs)
%
%   figNum: a figure number to plot results. If set to -1, skips any
%   input checking or debugging, no figures will be generated, and sets
%   up code to maximize speed. As well, if given, this forces the
%   variable types to be displayed as output and as well makes the input
%   check process verbose
%
% OUTPUTS:
%
%    costGraph: the cost graph matrix. A cost matrix is an nxn matrix where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element i-j is the cost of routing from i to j.
%
%    visibilityGraph: the original visibility matrix
%    corresponding to pointsWithData and the polytopes passed into the fcn
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
% fcn_Visibility_clearAndBlockedPointsGlobal
% fcn_Visibility_convertPolytopetoDedupedPoints
% fcn_MapGen_plotPolytopes
%
% EXAMPLES:
%
% See the script: script_test_fcn_VGraph_findEdgeWeights
% for a full test suite.
%
% This function was written on December 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

% REVISION HISTORY:
% As: fcn_algorithm_generate_cost_graph
% December 2023 by Steve Harnett
% -- first write of function
%
% As: fcn_BoundedAStar_findEdgeWeights
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% -- copied function to new file from fcn_algorithm_generate_cost_graph to
%    follow library conventions
% 2025_08_07 - K. Hayes
% -- updated fcn header and formatting
% -- moved plotting into fcn debug section
% 
% As: fcn_VGraph_findEdgeWeights
% 2025_11_06 - S. Brennan
% -- Renamed function
%    % * from fcn_BoundedAStar_findEdgeWeights
%    % * to fcn_VGraph_findEdgeWeights
% -- Cleaned up variable naming:
%    % * From fig_num to figNum
%    % * From all_pts to pointsWithData
%    % * From gap_size to gapSize
%    % * From cost_matrix to costGraph
%    % * From visibility_matrix_original to visibiltyGraph
%    % * From visibility_matrix_original to visibiltyGraph
% -- Deprecated fcn_BoundedAStar_convertPolytopetoDedupedPoints
%    % * Changed to fcn_Visibility_convertPolytopetoDedupedPoints
% -- Fixed global variables: _MAPGEN_ --> _VGRAPH_
% 2025_11_07 - S. Brennan
% -- Cleared extra figure command out of Inputs section

% TO DO:
% 2025_11_97 - S. Brennan, sbrennan@psu.ed
% -- need to finish function. It's half done

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 4; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS");
    MATLABFLAG_VGRAPH_FLAG_DO_DEBUG = getenv("MATLABFLAG_VGRAPH_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_VGRAPH_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_VGRAPH_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_VGRAPH_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_figNum = 999978; %#ok<NASGU>
else
    debug_figNum = []; %#ok<NASGU>
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

if 0==flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(3,MAX_NARGIN);

        % Check the polytopes input, make sure it is a structure
        assert(isstruct(polytopes));

        % Check the pointsWithData input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            pointsWithData, '5column_of_numbers');

    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin)
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        flag_do_plots = 1;
    end
end


%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%See: http://patorjk.com/software/taag/#p=display&f=Big&t=Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

% WARNING WORK IN PROGRESS FUNCTION
% TODO if the visibility matrix is reduced, this should be modified to
% find the min cost, of the two polytopes that are both members of pt 1
% and pt 2
% pointsWithData = [xv' yv' [1:length(xv)]' obs_id beg_end];
% pts : obs_id
% want to form a matrix of 1 and 0 for visibility
visibility_matrix = fcn_Visibility_clearAndBlockedPointsGlobal(polytopes,pointsWithData,pointsWithData,-1);
visibilityGraph = visibility_matrix;

% for each 1 in the visibility matrix...
[fromIndex, toIndex] = find(visibility_matrix==1);

% find the point pairs that are visible to each other
fromPoints = pointsWithData(fromIndex,:);
toPoints = pointsWithData(toIndex,:);

% We only want to keep costs polys if BOTH points are on the poly, not one.
% if only one point in the pair belongs to a certain polytope, this means
% the edge does not go through or along that polytope thus that edge cost
% cannot be used same_poly_edges = first_pts(:,4)==second_pts(:,4);
first_pts_redux = fromPoints; %(first_pts(:,4)==second_pts(:,4),:);
second_pts_redux = toPoints; %(first_pts(:,4)==second_pts(:,4),:);

% find the corresponding polytopes
first_polys = polytopes(first_pts_redux(:,4));
second_polys = polytopes(second_pts_redux(:,4));

% find the corresponding costs
first_traversal_costs = extractfield(first_polys,'cost');
second_traversal_costs = extractfield(second_polys,'cost');
% edges weights will be the minimum of the obstacles it spans
min_traversal_costs = min(first_traversal_costs,second_traversal_costs);
% explanation of the following line: https://www.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html
idx = sub2ind(size(visibility_matrix), fromIndex, toIndex);
visibility_matrix(idx) = min_traversal_costs;
costGraph = visibility_matrix;

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
    figure(figNum)
    hold on

    % Plot polytopes
    plotFormat.Color = 'Blue';
    plotFormat.LineWidth = 2;
    fcn_MapGen_plotPolytopes(polytopes, plotFormat, [1 0 0 0 1], figNum);

    % Plot vgraph edges
    deduped_pts = fcn_Visibility_convertPolytopetoDedupedPoints(pointsWithData);
    vgraph = visibilityGraph;
    if gapSize ==0
        for ith_from = 1:size(vgraph,1)
            for jth_to = 1:size(vgraph,1)
                if vgraph(ith_from,jth_to) == 1
                    plot([deduped_pts(ith_from).x,deduped_pts(jth_to).x],[deduped_pts(ith_from).y,deduped_pts(jth_to).y],'g-','LineWidth',1)
                end
            end
        end
    end
    if gapSize ~=0
        for ith_from = 1:size(vgraph,1)
            dataToPlot = [];
            for jth_to = 1:size(vgraph,1)
                if vgraph(ith_from,jth_to) == 1
                    dataToPlot = [dataToPlot; pointsWithData(ith_from,1:2); pointsWithData(jth_to,1:2); nan(1,2)]; %#ok<AGROW>                    
                end
            end
            plot(dataToPlot(:,1), dataToPlot(:,2),'g-','LineWidth',2)
        end
    end

    % Plot cgraph edges
    % for symmetric we only ned upper triangular part
    cgraph = costGraph;
    cgraph_upper_tri = triu(cgraph,1);
    [fromIndex, toIndex] = find(cgraph_upper_tri>0);
    for ith_from = 1:size(fromIndex,1)
        hold on;
        txt = sprintf('%.2f',round(cgraph(fromIndex(ith_from),toIndex(ith_from)),2));
        x1 = pointsWithData(fromIndex(ith_from),1);
        x2 = pointsWithData(toIndex(ith_from),1);
        y1 = pointsWithData(fromIndex(ith_from),2);
        y2 = pointsWithData(toIndex(ith_from),2);
        xbar = mean([x1, x2]);
        ybar = mean([y1, y2]);
        text(xbar, ybar, txt, 'clipping', 'off', 'Color', 'b');
    end
end


end % end function

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




