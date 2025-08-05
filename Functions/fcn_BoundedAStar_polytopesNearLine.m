function [close_polytopes] = fcn_BoundedAStar_polytopesNearLine(start,finish,polytopes,varargin)
% fcn_BoundedAStar_polytopesNearLine 
% 
% find polytopes possibly within reach of the line of interest
%
% FORMAT: 
%
% [close_polytopes]=fcn_BoundedAStar_polytopesNearLine(START,FINISH,POLYTOPES)
% 
% INPUTS:
%   polytopes: the original 1-by-i six field structure with the same fields,
%   where i = number of original polytopes
%   
%   start: a 1-by-5 vector of starting point information, including:
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number 
%       beginning/ending indication (1 if the point is a beginning or ending
%       point and 0 otherwise) 
%       Ex: [x y point_id obs_id beg_end]
%   
%   finish: a 1-by-5 vector of ending point information, including the same
%   information as START
%
%   (optional inputs)
% 
%   fig_num: a figure number to plot results. If set to -1, skips any
%   input checking or debugging, no figures will be generated, and sets
%   up code to maximize speed. As well, if given, this forces the
%   variable types to be displayed as output and as well makes the input
%   check process verbose
%
% OUTPUTS:
%   close_polytopes: a 1-by-n seven field structure of polytopes near the 
%   line,where n = number of close polytopes in polytopes, with fields:
%       vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%       the number of the individual polytope vertices
%       xv: a 1-by-m vector of vertice x-coordinates
%       yv: a 1-by-m vector of vertice y-coordinates
%       distances: a 1-by-m vector of perimeter distances from one point to the
%       next point, distances(i) = distance from vertices(i) to vertices(i+1)
%       mean: average xy coordinate of the polytope
%       area: area of the polytope
%       max_radius: distance from the mean to the farthest vertex
% 
% This function was written on 2018_12_18 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
% 2025_07_17 - K. Hayes
% -- copied to new function from
%    fcn_polytope_calculation_polytopes_near_the_line to follow library
%    convention
% 2025_08_05 - K. Hayes
% -- updated fcn format and header
% -- added debug plotting
% -- moved example in header to separate test script

% TO DO:
% (none)

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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
        narginchk(3,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

        % Check the polytope input, make sure it is type struct
        assert(isstruct(polytopes));

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


radius_of_polys = [polytopes.max_radius]';
radius_of_polys_squared = radius_of_polys.^2;
centers_of_polys = reshape([polytopes.mean],2,length(polytopes))';

dist_from_line_to_polys_squared = fcn_BoundedAStar_calculatePointToLineDistSquared([centers_of_polys zeros(size(centers_of_polys,1),1)],[start(1:2) 0],[finish(1:2) 0]);

ref_vec = ones(size(centers_of_polys,1),1)*(finish(1:2) - start(1:2));
straight = sum((start(1:2) - finish(1:2)).^2,2).^0.5;
para_dists = (dot(ref_vec,(centers_of_polys-start(1:2)),2))/straight;

close_polytopes = polytopes((radius_of_polys_squared>dist_from_line_to_polys_squared).*(para_dists+radius_of_polys>=0).*(para_dists-radius_of_polys<=straight) == 1);


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
    % Plot polytope field
    figure(fig_num)
    hold on
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;
    fcn_MapGen_plotPolytopes(polytopes,plotFormat,[1 0 0 1 1],fig_num);
    plotFormat.Color = 'green';
    plotFormat.LineWidth = 2;
    fcn_MapGen_plotPolytopes(close_polytopes,plotFormat,[1 0 0 1 1],fig_num);

    % Plot hypothetical trajectory
    plot([start(1) finish(1)],[start(2) finish(2)],'kx','linewidth',1)
    plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
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
