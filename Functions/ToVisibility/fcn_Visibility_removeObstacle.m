function [newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
fcn_Visibility_removeObstacle(...
visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, indexOfPolytopeForRemoval, varargin)
% fcn_Visibility_removeObstacle
%   this function recalculates the visibility graph after deleting a
%   polytope without recalculating the entire visibility graph.  This is
%   accomplished using an AABB check as a coarse check. This function also
%   recalculates the pointsWithData, startPointData, finishPointData, and polytopes data structures
%   as these are also affected by the removal of an obstacle.
% 
%   See vgraph_modification section of
%   Documentation/bounded_astar_documentation.pptx for pseudocode and
%   algorithm description
%
% FORMAT:
%
% [newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
%     fcn_Visibility_removeObstacle(...
%     visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, indexOfPolytopeForRemoval, (figNum))
%
% INPUTS:
%
%     visibilityMatrix: nxn matrix, where n is the number of points in pointsWithData
%       a 1 in column i and row j indicates that pointsWithData(i,:) is visible from
%       pointsWithData(j,:).  This matrix is therefore symmetric
%     
%     pointsWithData: p-by-5 matrix of all the polytope points
%       the information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id number
%         obstacle id number
%         beginning/ending indication (1 if the point is a beginning or ending
%         point and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%     
%     startPointData: 1-by-5 vector with the same info as route for the starting point
%     
%     finishPointData: same as startPointData for the finishPointData point
%
%     polytopes: the polytope struct array prior to modification INCLUDING the polytope for removal
%     
%     indexOfPolytopeForRemoval: the index of the polytope to be removed in the polytopes struct array
%
%     (optional inputs)
%
%     figNum: a figure number to plot results. If set to -1, skips any
%         input checking or debugging, no figures will be generated, and sets
%         up code to maximize speed. As well, if given, this forces the
%         variable types to be displayed as output and as well makes the input
%         check process verbose
%
% OUTPUTS:
%     newVisibilityMatrix: same as the visiblity_matrix input but modified so that the removed
%         removed polytope no longer affects the visibility.  Note this may have fewer points than
%         the input matrix as points on the removed polytope are deleted.
%
%     newPointsWithData: same as the pointsWithData input but with points on the removed polytope deleted.
%         May be reindexed
%
%     newStartPointData: same as startPointData input but reindexed to account for removed points
%     
%     newFinishPointData: same as finishPointData input but reindexed to account for removed points
%     
%     newPolytopes:  the polytope struct array after modification no longer including
%         the polytope for removal
%
% DEPENDENCIES:
%     fcn_MapGen_isCrossingAABB from the MapGen repo
%     fcn_Visibility_clearAndBlockedPoints
%
% EXAMPLES:
%
% See the scripts: 
%
%     script_test_fcn_Visibility_removeObstacle and 
%     script_demo_visibilityGraphAddRemoveObstacles
%
% for a full test suite.
%
% Questions or comments? contact sjh6473@psu.edu

% REVISION HISTORY
% As: fcn_visibility_graph_remove_obstacle
% 2024_03
% -- first written by Steve Harnett
%
% As: fcn_Visibility_removeObstacle
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_visibility_graph_remove_obstacle
%    to follow library convention
% 2025_07_31 - K. Hayes
% -- updated function formatting and header
% -- added input and debug checks
% 2025_08_04 - K. Hayes
% -- added debug plotting
% 2025_11_03 - S. Brennan
% -- updated variable naming:
%    % * fig_num to figNum
%    % * visibility_matrix to visibilityMatrix
%    % * all_pts to pointsWithData
%    % * start_new to newStartPointData
%    % * finish_new to newFinishPointData
%    % * newPolytopes to newPolytopes
%    % * visibilityMatrix_new to newVisibilityMatrix
%    % * pointsWithData_new to newPointsWithData
%    % * polytopes to polytopes
%    % * start to startPointData
%    % * finish to finishPointData
%    % * idx_of_polytope_for_removal to indexOfPolytopeForRemoval
% - staged function to move into Visibility library
%    % * _MAPGEN_ changed to _VGRAPH_

% TO DO:
% - uncomment the AABB test and get this to work without using deprecated
% function. It significantly speeds up this function

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 7; % The largest Number of argument inputs to the function
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
        narginchk(6,MAX_NARGIN);

        % % Check the startPointData input, make sure it has 5 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     startPointData, '5column_of_numbers');
        % 
        % % Check the finishPointData input, make sure it has 5 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     finishPointData, '5column_of_numbers');

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


%% initialize the modified vgraph, pts, startPointData, and finishPointData to the old values
newVisibilityMatrix = visibilityMatrix;
newPointsWithData = pointsWithData;
newStartPointData = startPointData;
newFinishPointData = finishPointData;

NtoDelete = length(find(pointsWithData(:,4)==indexOfPolytopeForRemoval));

%% remove polytope from vgraph and pointsWithData
polytope_of_interest = polytopes(indexOfPolytopeForRemoval); % get polytope for removal
% get AABB of polytope for removal
polytope_vertices = [[polytope_of_interest.xv]',[polytope_of_interest.yv]'];
% AABB = [min(polytope_of_interest.xv) min(polytope_of_interest.yv) max(polytope_of_interest.xv) max(polytope_of_interest.yv)];

% delete polytope
newPolytopes = polytopes;
newPolytopes(indexOfPolytopeForRemoval) = [];
% find the verts of that polytope in pointsWithData table
[~, xloc] = ismember(polytope_vertices(:,1), pointsWithData(:,1));
[~, yloc] = ismember(polytope_vertices(:,2), pointsWithData(:,2));
idx_of_points_on_polytope = union(xloc,yloc);
% vgraph edges that start or end on this obstacle should be removed
newVisibilityMatrix(idx_of_points_on_polytope,:) = [];
newVisibilityMatrix(:,idx_of_points_on_polytope) = [];
% remove points on that polytope from pointsWithData table
newPointsWithData(idx_of_points_on_polytope,:) = [];
% size of vgraph has now changed so points need to be re-indexed
% reindex pointsWithData, start, and finish
num_pts_after_removal = size(newPointsWithData,1);
newPointsWithData(:,3) = (1:num_pts_after_removal)';

% if the user gave a startPointData or finishPointData, reindex it
if ~isempty(startPointData)
    newStartPointData(3) = startPointData(3)-NtoDelete;
end
if ~isempty(finishPointData)
    newFinishPointData(3) = finishPointData(3)-NtoDelete;
end

% find which new points cross the AABB
% NOTE: commented this out because function is deprecated.
% isInside = fcn_MapGen_isCrossingAABB(AABB, [newPointsWithData(1:end-2,:); startPointData; finishPointData]);
isInside = true(length(newPointsWithData(:,1)),1);

% only want possible edges that are not already edges as deleting the obstacle adds edges, it does
% not remove existing edges
[r,c] = find(isInside & ~newVisibilityMatrix);
%% check only specific edges method
for i = 1:length(r)
    [~,~,D] = fcn_Visibility_clearAndBlockedPoints(newPolytopes, newPointsWithData(r(i),:), newPointsWithData(c(i),:));
    visibility_scalar = sum(D);
    assert(isequal(size(visibility_scalar),[1 1]))
    if ~visibility_scalar
        newVisibilityMatrix(r(i),c(i)) = 1;
    end
end

% sprintf('num checks was %i',length(r))

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

    % Plot old map
    subplot(1,2,1)
    title('Previous map')
    fcn_MapGen_plotPolytopes(polytopes,'b',[1 0 0 0 0.5],figNum);
    box on
    xlabel('x [km]')
    ylabel('y [km]')
     % Plot new vgraph
    for i = 1:size(visibilityMatrix,1)
        for j = 1:size(visibilityMatrix,1)
            if visibilityMatrix(i,j) == 1
                plot([pointsWithData(i,1),pointsWithData(j,1)],[pointsWithData(i,2),pointsWithData(j,2)],'-g')
            end
        end
    end
  
    % Plot revised map
    subplot(1,2,2)
    title('Updated map')
    fcn_MapGen_plotPolytopes(newPolytopes,'b',[1 0 0 0 0.5],figNum);
    box on
    xlabel('x [km]')
    ylabel('y [km]')
       
    % Plot new vgraph
    for i = 1:size(newVisibilityMatrix,1)
        for j = 1:size(newVisibilityMatrix,1)
            if newVisibilityMatrix(i,j) == 1
                plot([newPointsWithData(i,1),newPointsWithData(j,1)],[newPointsWithData(i,2),newPointsWithData(j,2)],'-g')
            end
        end
    end
end

end



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

