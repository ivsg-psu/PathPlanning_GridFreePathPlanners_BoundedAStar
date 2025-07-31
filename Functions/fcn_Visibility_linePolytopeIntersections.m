function [xings] = fcn_Visibility_linePolytopeIntersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes, varargin)
% fcn_Visibility_linePolytopeIntersections 
% 
% finds polytope-path intersection 
% information
%
% FORMAT:
% [xings] = fcn_Visibility_linePolytopeIntersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes,(fig_num))
% 
% INPUTS: 
%
%   xiP: m-by-1 vector of the x-coordinates of the starting point of 
%       potential paths
%
%   yiP: m-by-1 vector of the y-coordinates of the starting point of 
%       potential paths
%
%   xiQ: m-by-1 vector of the x-coordinates of the finishing points of 
%       potential paths
%
%   yiQ: m-by-1 vector of the y-coordinates of the finishing points of 
%       potential paths
%
%   xjP: 1-by-n vector of the x-coordinates of the starting point of the
%       polytope edge
%
%   yjP: 1-by-n vector of the y-coordinates of the starting point of the
%       polytope edge
%
%   D: m-by-n intersection array, where m = number of finish points 
%       and n = number of polytope edges, where 1 indicates that an
%       intersection occures between the START and FINISH point on the
%       corresponding polytope edge and 0 indicates no intersection
% 
%   di: m-by-n intersection array, where each value gives the percentage of
%       how far along the path from START to FINISH the intersection occurs
% 
%   num_int: m-by-1 vector of the number of intersections between the start
%       and each point in finish
%
%   polytopes: a 1-by-p six field structure of combined polytopes, where 
%       p = number of polytopes, with fields:
%           vertices: a v+1-by-2 matrix of xy points with row1 = rowv+1
%           xv: a 1-by-v vector of vertice x-coordinates
%           yv: a 1-by-v vector of vertice y-coordinates
%           distances: a 1-by-v vector of perimeter distances from one point to the
%               next point, distances(i) = distance from vertices(i) to vertices(i+1)
%           mean: average xy coordinate of the polytope
%           area: area of the polytope
%           max_radius: distance from the mean to the furthest vertex 
% 
%   (optional inputs)
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose
%
% OUTPUTS:
%
%   xings: a 1-by-f three field structure of intersection information, where
%   f = number of finish points, with fields:
%       points: i-by-2 matrix of xy coordinates, where i=number of intersections  
%       index: 1-by-i vector of intersecting line indices
%       obstacles: 1-by-i vector of obstacles intersecting the line
%
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% This function was written on 2018_11_17 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method in function usage examples
% 2025_07_17 - K. Hayes
% -- copied to new function from fcn_visibility_line_polytope_intersections
%    to follow library convention
% 2025_07_31 - K. Hayes
% -- updated function header and formatting
% -- added input and debugging capabilities
%
% TO DO:
% -- add input checking for polytope structure

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 11; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
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
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
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
        narginchk(10,MAX_NARGIN);

        % Check the xiP input, make sure it has 1 column
        fcn_DebugTools_checkInputsToFunctions(...
            xiP, '1column_of_numbers');

        % Check the yiP input, make sure it has 1 column
        fcn_DebugTools_checkInputsToFunctions(...
            yiP, '1column_of_numbers');

        % Check the xiQ input, make sure it has 1 column
        fcn_DebugTools_checkInputsToFunctions(...
            xiQ, '1column_of_numbers');

        % Check the yiQ input, make sure it has 1 column
        fcn_DebugTools_checkInputsToFunctions(...
            yiQ, '1column_of_numbers');
    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        fig_num = temp;
        figure(fig_num);
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

% check input arguments
if nargin ~= 10
    error('Incorrect number of arguments');
end

num_vecs = length(num_int);
xings(num_vecs) = struct('points',[],'index',[],'obstacles',[]);
verts = length([polytopes.xv]);
xing_ind = 1:verts; % create an array of all vector indices that could intersect the reference vector

obs_id = zeros(1,verts);
curpt = 0;
for poly = 1:size(polytopes,2)
    verts = length(polytopes(poly).xv);
    obs_id(curpt+1:curpt+verts) = poly;
    curpt = curpt+verts;
end

for vecs = 1:num_vecs % check each line for intersections
    if num_int(vecs)~=0 % if there are intersections
        % calculate intersection locations and corresponding vector indices
        dimid = di(vecs,D(vecs,:)==1);
        xjPmid = xjP(D(vecs,:)==1);
        yjPmid = yjP(D(vecs,:)==1);
        notNaN = ~isnan(dimid);
        dimod(notNaN) = dimid(notNaN);
        xings(vecs).points = [(xiP(vecs)+dimod.*(xiQ(vecs)-xiP(vecs)))', (yiP(vecs)+dimod.*(yiQ(vecs)-yiP(vecs)))'];
        xings(vecs).points(~notNaN,:) = [xjPmid(~notNaN)' yjPmid(~notNaN)'];
        xings(vecs).index = xing_ind(D(vecs,:)==1);
        xings(vecs).obstacles = obs_id(xings(vecs).index);
    %else leave blank
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