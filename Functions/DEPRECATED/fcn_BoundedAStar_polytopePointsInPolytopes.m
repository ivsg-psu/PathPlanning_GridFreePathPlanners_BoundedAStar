function [flagsAtLeastOnePointIsInPoly, startPolys, finishPolys, flagsStartIsInPoly, flagsFinishIsInPoly] = ...
    fcn_BoundedAStar_polytopePointsInPolytopes(startXY, finishXY, polytopes, varargin)

warning('on','backtrace');
warning(['fcn_BoundedAStar_polytopePointsInPolytopes is being deprecated. ' ...
    'Use fcn_VGraph_polytopePointsInPolytopes in the VGraph repo instead.']);


% fcn_BoundedAStar_polytopePointsInPolytopes checks if startXY or endXy is in
% one of the polytopes
%
% FORMAT:
%   
%   [flagsAtLeastOnePointIsInPoly, startPolys, finishPolys] = ...
%   fcn_BoundedAStar_polytopePointsInPolytopes(...
%      startXY, finishXY, polytopes, ...
%      (flagThrowError), (flagEdgeCheck), (figNum))
%   
% INPUTS:
%   
%   startXY: [Nx2] vector of x and y coordinates of starting points 
%   
%   finishXY: [Nx2] vector of x and y coordinates of ending points
%   
%   polytopes: a 1-by-n seven field structure of combined polytopes, where 
%     p = number of polytopes, with fields:
%         vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%         the number of the individual polytope vertices
%         xv: a 1-by-m vector of vertice x-coordinates
%         yv: a 1-by-m vector of vertice y-coordinates
%         distances: a 1-by-m vector of perimeter distances from one point to the
%         next point, distances(i) = distance from vertices(i) to vertices(i+1)
%         mean: average xy coordinate of the polytope
%         area: area of the polytope
%         max_radius: distance from the mean to the furthest vertex
%
% (OPTIONAL INPUTS)
%   
%   flagThrowError: flag determining whether an error should be thrown (1) for
%   points inside any polytope or no error and value assigned to ERR (0) 
%   
%   flagEdgeCheck: a flag that determines whether the polytope edges should be
%   checked for points. If set to 1, a point on the edge is considered
%   inside the polytope. Otherwise, a point on the edge is outside the
%   polytope.
%   
%   figNum: a figure number to plot results. If set to -1, skips any
%   input checking or debugging, no figures will be generated, and sets
%   up code to maximize speed. As well, if given, this forces the
%   variable types to be displayed as output and as well makes the input
%   check process verbose
%
% OUTPUTS:
%   
%   flagsAtLeastOnePointIsInPoly: [Nx1] logical vector containing, for each
%   ith start/end pair, a value of 0 if neither startXY or finishXY is in/on
%   any polytopes, 1 otherwise 
%
%   startPolys: [Nx1] vector of indicies of first polytope startXY is on.
%   Returns NaN if none.
%   
%   finishPolys: [Nx1] vector of indicies of first polytope finishXY is on.
%   Returns NaN if none.
%
%   flagsStartIsInPoly: [Nx1] vector of true if startXY is in a polytope
%
%   flagsFinishIsInPoly: [Nx1] vector of true if finishXY is in a polytope
%
% DEPENDENCIES:
%   
%   fcn_DebugTools_checkInputsToFunctions
%   inpolygon
%   fcn_MapGen_plotPolytopes
%
% EXAMPLES:
%
%     See the script: script_test_fcn_BoundedAStar_polytopePointsInPolytopes
%     for a full test suite.
%
% This function was written on 2018_12_18 by Seth Tau, maintained by Sean
% Brennan.
% Questions or comments? sbrennan@psu.edu 

% Revision History:
% As: fcn_polytope_calculation_points_in_polytopes
% 2018_12_18 by Seth Tau
% - First write of function
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% - Replaced fcn_general_calculation_euclidean_point_to_point_distance
%   with vector sum method in function usage examples
%
% As: fcn_BoundedAStar_polytopePointsInPolytopes
% 2025_07_17 - K. Hayes
% - copied to new function from
%   fcn_polytope_calculation_points_in_polytopes to follow library
%   convention
% 2025_08_14 - K. Hayes
% - updated fcn header and formatting
% - added debug plotting to fcn
% 2025_11_13 - S. Brennan, sbrennan@psu.edu
% (in fcn_BoundedAStar_polytopePointsInPolytopes)
% - Refactored code to change from structure inputs to vector inputs
% - Refactored code to vectorize outputs
% - Deleted extra figure call in input section
% - Fixed bad function formatting at end
% - Changed variable inputs to names that are more clear or that fit style
%   % standards
%   % from A (structure) to startXY (1x2 vector)
%   % from B (structure) to finishXY (1x2 vector)
%   % fig_num to figNum
%   % throw_error to flagThrowError
%   % edge_check to flagEdgeCheck
%   % Apoly to startPolys
%   % Bpoly to finishPolys
%   % err to flagsAtLeastOnePointIsInPoly

% TO-DO:
% - (nothing)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 6; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS");
    MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG = getenv("MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS);
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

        % Check the startXY input, make sure it is [Nx2]
        fcn_DebugTools_checkInputsToFunctions(startXY, '2column_of_numbers');

        % Check the startXY input, make sure it is [Nx2]
        fcn_DebugTools_checkInputsToFunctions(finishXY, '2column_of_numbers');

        % Check the polytopes input, make sure it is numeric
        assert(isstruct(polytopes));
        
    end
end

% Does user want to throw errors?
flagThrowError = 0; % Default is to NOT throw errors
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
       flagThrowError = temp;
    end
end

% Does user want to check edges?
flagEdgeCheck = 0;
if 5 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        flagEdgeCheck = temp;
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

Npoints = length(startXY(:,1));
Nfinishes = length(finishXY(:,1));
if Npoints~=Nfinishes
    error('The length of startXY must match finishXY');
end

Npolys = size(polytopes,2);

% Initialize outputs
flagsAtLeastOnePointIsInPoly = false(Npoints,1);
startIsInPoly  = false(Npolys,Npoints);
startIsOnPoly  = false(Npolys,Npoints);
finishIsInPoly = false(Npolys,Npoints);
finishIsOnPoly = false(Npolys,Npoints);

startPolys = nan(Npoints,1);
finishPolys = nan(Npoints,1);

% Check each polytope
for ith_polytope = 1:Npolys
    [startIsInPoly(ith_polytope,:),startIsOnPoly(ith_polytope,:)] = inpolygon(startXY(:,1),startXY(:,2),polytopes(ith_polytope).vertices(:,1),polytopes(ith_polytope).vertices(:,2));
    [finishIsInPoly(ith_polytope,:),finishIsOnPoly(ith_polytope,:)] = inpolygon(finishXY(:,1),finishXY(:,2),polytopes(ith_polytope).vertices(:,1),polytopes(ith_polytope).vertices(:,2));
end

% Does user want to count edges as "in", or "out"?
if flagEdgeCheck == 1
    % Edges and insides are both "in"
    isinsideStart = startIsInPoly | startIsOnPoly;
    isinsideFinish = finishIsInPoly | finishIsOnPoly; 
else
    % Only insides are "in"
    isinsideStart = startIsInPoly;
    isinsideFinish = finishIsInPoly; 
end

% Does user want to throw errors?
if flagThrowError == 1
    % is startXY in strictly in polygon?
    if any(isinsideStart,'all') 
        error('At least one startXY is within an obstacle')
    end
    if any(isinsideFinish,'all')
        error('At least one finishXY is within obstacle')
    end
else
    eitherStartOrFinishInside = isinsideStart | isinsideFinish;
    flagsAtLeastOnePointIsInPoly = any(eitherStartOrFinishInside,1)';
    flagsStartIsInPoly = any(isinsideStart,1)';
    flagsFinishIsInPoly = any(isinsideFinish,1)';


    % Loop through the points, checking if any polys are tagged as inside
    for ith_point = 1:Npoints
        firstStartPoly = find(isinsideStart(:,ith_point),1);
        if ~isempty(firstStartPoly)
            startPolys(ith_point,1) = firstStartPoly;
        end
        firstFinishPoly = find(isinsideFinish(:,ith_point),1);
        if ~isempty(firstFinishPoly)
            finishPolys(ith_point,1) = firstFinishPoly;
        end
    end
end

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
    % set up plot
    figure(figNum);
    hold on;
    box on;
    
    legend('Interpreter','none','Location','best');

    % Plot polytopes
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;
    h_polys = fcn_MapGen_plotPolytopes(polytopes, plotFormat, [1 0 0 0 1], figNum);
    set(h_polys,'DisplayName','polytopes')

    % Plot start and finish points
    plot(startXY(:,1), startXY(:,2), 'g.', 'MarkerSize',20, 'DisplayName', 'startXY')
    plot(finishXY(:,1), finishXY(:,2), 'r.', 'MarkerSize',20, 'DisplayName', 'finishXY')

    % Plot points inside    
    plot(startXY(flagsStartIsInPoly,1), startXY(flagsStartIsInPoly,2), 'gx', 'MarkerSize',10, 'LineWidth', 2, 'DisplayName', 'start inside')
    plot(finishXY(flagsFinishIsInPoly,1), finishXY(flagsFinishIsInPoly,2), 'rx', 'MarkerSize',10, 'LineWidth', 2, 'DisplayName', 'finish inside')

    % Make axis bigger
    drawnow;
    set(gca,'XLimitMethod','padded','YLimitMethod','padded')
    temp = axis;
    rangeX = temp(2)-temp(1);
    rangeY = temp(4)-temp(3);
    biggerAxis = [temp(1)-rangeX*0.2 temp(2)+rangeX*0.2 temp(3)-rangeY*0.2 temp(4)+rangeY*0.2];
    axis(biggerAxis);

end 

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