function [newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
fcn_VGraph_addObstacle(...
visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, varargin)
% fcn_VGraph_addObstacle
%
%   Recalculates the visibility graph after adding a polytope without
%   recalculating the entire visibility graph.  This is accomplished using
%   an AABB check as a coarse check. This function also recalculates the
%   pointsWithData, startPointData, finishPointData, and polytopes data
%   structures as these are also affected by the addition of an obstacle.
%   See vgraph_modification section of
%   Documentation/bounded_astar_documentation.pptx for pseudocode and
%   algorithm description
%
% FORMAT:
%     [newVisibilityMatrix, newPointsWithData, newStartPointData, newFinishPointData, newPolytopes] = ...
%     fcn_VGraph_addObstacle(...
%     visibilityMatrix, pointsWithData, startPointData, finishPointData, polytopes, polytopeToAdd, (figNum))
%
% INPUTS:
%
%     visibilityMatrix - nxn matrix, where n is the number of points in pointsWithData
%       a 1 in column i and row j indicates that pointsWithData(i,:) is visible from
%       pointsWithData(j,:).  This matrix is therefore symmetric
%
%     pointsWithData: p-by-5 matrix of all the possible start points
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
%     finishPointData: same as startPointData for the finish point
%     
%     polytopes - the polytope struct array prior to modification EXCLUDING the polytope for additionk
%     
%     polytopeToAdd - struct of the polytope to add
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
%
%     newVisibilityMatrix: same as the visiblity_matrix input but modified so that the added
%         polytope now affects the visibility.  Note this may have more points than
%         the input matrix as points on the added polytope are now considered as nodes.
%
%     newPointsWithData: same as the pointsWithData input but with points on the added polytope.
%         May be reindexed.
%     
%     newStartPointData: same as startPointData input but reindexed to account for added points
%     
%     newFinishPointData: same as finishPointData input but reindexed to account for added points
%     
%     newPolytopes:  the polytope struct array after modification now including
%         the polytope for addition
%
% DEPENDENCIES:
%
%     fcn_MapGen_isCrossingAABB from the MapGen repo
%     fcn_VGraph_clearAndBlockedPoints
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the scripts: 
%
%     script_test_fcn_VGraph_addObstacle
%     script_demo_visibilityGraphAddRemoveObstacles
%
% for a full test suite.
%
% Questions or comments? contact sjh6473@psu.edu

% REVISION HISTORY:
% As: fcn_visibility_graph_add_obstacle
% 2024_03
% -- first written by Steve Harnett
% 
% As: fcn_Visibility_addObstacle 
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_visibility_graph_add_obstacle to
%    follow library convention
% 2025_07_31 - K. Hayes
% -- fixed formatting and header details
% -- added input and debugging capabilities
% 2025_08_01 - K. Hayes
% -- moved plotting into function debug section
% 2025_11_03 - S. Brennan
% -- updated variable naming:
%    % * fig_num to figNum
%    % * visibility_matrix to visibilityMatrix
%    % * all_pts to pointsWithData
%    % * start_new to newStartPointData
%    % * finish_new to newFinishPointData
%    % * polytopes_after to newPolytopes
%    % * visibilityMatrix_new to newVisibilityMatrix
%    % * pointsWithData_new to newPointsWithData
%    % * polytopes_before to polytopes
%    % * start to startPointData
%    % * finish to finishPointData
%    % * polytope_to_add to polytopeToAdd
% - staged function to move into Visibility library
%    % * _MAPGEN_ changed to _VGRAPH_
% - fixed bug where start and finish data points not filled correctly
%
% As: fcn_VGraph_addObstacle
% 2025_11_07 - S. Brennan
% -- Renamed fcn_Visibility_addObstacle to fcn_VGraph_addObstacle
% -- Cleared extra figure command out of Inputs section


% TO DO:
% -- make sure input checking is working correctly

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
        % 
        % % Check the pointsWithData input, make sure it has 5 columns
        % fcn_DebugTools_checkInputsToFunctions(...
        %     pointsWithData, '5column_of_numbers');
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
old_point_count = size(visibilityMatrix,2);
newStartPointData = startPointData;
newFinishPointData = finishPointData;

%% add polytope
newPolytopes = [polytopes, polytopeToAdd]; % append new polytope to polytope struct array
% form AABB for added polytope
% polytope_vertices = [[polytopeToAdd.xv]',[polytopeToAdd.yv]'];
% AABB = [min(polytopeToAdd.xv) min(polytopeToAdd.yv) max(polytopeToAdd.xv) max(polytopeToAdd.yv)];


% remake pointsWithData table
new_point_tot = length([polytopeToAdd.xv]); % total number of vertices in the polytopes
beg_end = zeros(1,new_point_tot); % is the point the start/end of an obstacle
curpt = 0;
polytopeToAdd.obs_id = ones(1,new_point_tot)*(max(pointsWithData(:,4))+1); % obs_id is the same for every vertex on a single polytope
beg_end([curpt+1,curpt+new_point_tot]) = 1; % the first and last vertices are marked with 1 and all others are 0
% curpt = curpt+new_point_tot;

obs_id = [polytopeToAdd.obs_id];
newPointsWithData = [[polytopeToAdd.xv];[polytopeToAdd.yv];max(pointsWithData(:,3))+1:max(pointsWithData(:,3))+new_point_tot;obs_id;beg_end]'; % all points [x y point_id obs_id beg_end]
newPointsWithData = [pointsWithData; newPointsWithData];
% if the user gave a startPointData or finish, reindex it
start_and_finish_count = 0;
if ~isempty(newStartPointData)
    newStartPointData = [startPointData(1:2) size(newPointsWithData,1)+1 -1 1];
    start_and_finish_count = start_and_finish_count + 1;
end
if ~isempty(newFinishPointData)
    newFinishPointData = [finishPointData(1:2) size(newPointsWithData,1)+2 -1 2];
    start_and_finish_count = start_and_finish_count + 1;
end
%% initialize new visibility matrix as block matrix
% the following code inserts the new points on the new polytope into the vgraph such that
% the start and finish are still the last two points in the matrix rather than simply appending
% new points to the end of the vgraph resulting in the start and finish being somewhere in
% the interior of the matrix
% a diagram of what this does is shown below
% old_vgraph =
%   ___________________________________________________________________
%   |  A                             | D                             |
%   |  the original vgraph           | original vgraph               |
%   |  block for points excluding    | block for start and finish    |
%   |  the start and the finish      | relative to other pts         |
%   ___________________________________________________________________
%   | D'                             |  F                            |
%   | original vgraph                |  original vgraph              |
%   | block for start and finish     |  block for start and finish   |
%   | relative to other pts          |  relative to themselves       |
%   ___________________________________________________________________
% new_vgraph =
%   ___________________________________________________________________________________________________
%   |  A                             | B                             | D                             |
%   |  the original vgraph           | block of zeros for new points | original vgraph               |
%   |  block for points excluding    | relative to other pts         | block for start and finish    |
%   |  the start and the finish      | init. to zero until checked   | relative to other pts         |
%   ___________________________________________________________________________________________________
%   |  B'                            | C                             | E                             |
%   |  block of zeros for new points | block for new points relative | block of zeros for new points |
%   |  relative to other pts         | to themselves.  Init. to      | relative to start and finish  |
%   |  init. to zero until checked   | identity until checked        | init. to zero until checked   |
%   ___________________________________________________________________________________________________
%   | D'                             | E'                            |  F                            |
%   | original vgraph                | block of zeros for new points |  original vgraph              |
%   | block for start and finish     | relative to start and finish  |  block for start and finish   |
%   | relative to other pts          | init. to zero until checked   |  relative to themselves       |
%   ___________________________________________________________________________________________________


A = visibilityMatrix(1:(old_point_count-start_and_finish_count),1:(old_point_count-start_and_finish_count));
assert(isequal(size(A) ,  [old_point_count - start_and_finish_count, old_point_count - start_and_finish_count]))
B = zeros(old_point_count-start_and_finish_count, new_point_tot);
assert(isequal(size(B) ,  [old_point_count - start_and_finish_count, new_point_tot]))
C = eye(new_point_tot, new_point_tot);
assert(isequal(size(C) ,  [new_point_tot, new_point_tot]))
if start_and_finish_count ~= 0
    D = visibilityMatrix(1:end-start_and_finish_count,end-start_and_finish_count+1:end);
    assert(isequal(size(D) ,  [old_point_count - start_and_finish_count, start_and_finish_count]))
    E = zeros(new_point_tot,start_and_finish_count);
    assert(isequal(size(E) ,  [new_point_tot, start_and_finish_count]))
    F = visibilityMatrix(end-start_and_finish_count+1:end,end-start_and_finish_count+1:end);
    assert(isequal(size(F) ,  [start_and_finish_count, start_and_finish_count]))
else
    D = [];
    E = [];
    F = [];
end
newVisibilityMatrix = [A, B, D; B', C, E; D', E', F];
%% fill in values for new vgraph based on aabb coarse check first
% find which new points cross the AABB
% NOTE: commented this out because function is deprecated.
% isInside = fcn_MapGen_isCrossingAABB(AABB, [newPointsWithData; startPointData; finishPointData]);
isInside = true(length(newPointsWithData(:,1)),1);


% need to note edges that are 1 in new vgraph (i.e. they are unblocked) and cross the AABB
[r,c] = find(isInside & newVisibilityMatrix);
% also need to note edges that go to any point but either start or end on the new obstacle
idx_of_new_obs = (old_point_count - start_and_finish_count + 1):(old_point_count - start_and_finish_count + new_point_tot); % ids of new points
idx_of_pointsWithData = 1:size(newVisibilityMatrix,1); % ids of all points
combos_of_idx_of_new_obs = combinations(idx_of_new_obs,idx_of_pointsWithData); % every combination of a new point with some other points
array_of_combos_of_idx_of_new_obs = table2array(combos_of_idx_of_new_obs); % convert from table to array
% append the list of new edges to the list of edges possibly blocked by the new obstacle because they cross the AABB
r = [r;array_of_combos_of_idx_of_new_obs(:,1)];
c = [c;array_of_combos_of_idx_of_new_obs(:,2)];
%% check only  specific edges method
% TODO @sjharnett use global function and check from each start to all finishes for that start
for i = 1:length(r)
    [~,~,D] = fcn_VGraph_clearAndBlockedPoints(newPolytopes, newPointsWithData(r(i),:), newPointsWithData(c(i),:));
    visibility_scalar = sum(D);
    assert(isequal(size(visibility_scalar),[1 1]))
    if ~visibility_scalar
        newVisibilityMatrix(r(i),c(i)) = 1;
    else
        newVisibilityMatrix(r(i),c(i)) = 0;
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
    fcn_MapGen_plotPolytopes(polytopeToAdd,'y',[1 0 0 0 0.5],figNum);
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
