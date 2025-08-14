function [err,Apoly,Bpoly] = fcn_BoundedAStar_polytopePointsInPolytopes(A,B,polytopes,varargin)
% fcn_BoundedAStar_polytopePointsInPolytopes see if point A or B is in one of the
% polytopes
%
% FORMAT:
%
% [err, Apoly, Bpoly]=fcn_BoundedAStar_polytopePointsInPolytopes(A, B, polytopes, (throw_error), (edge_check), (fig_num))
%
% INPUTS:
%
% A: two field structure of x and y coordinates of starting point A, with 
%   fields:
%   x: x coordinate
%   y: y coordinate
%
% B: two field structure of x and y coordinates of ending point B, with 
%   fields:
%   x: x coordinate
%   y: y coordinate
%
% polytopes: a 1-by-n seven field structure of combined polytopes, where 
%   p = number of polytopes, with fields:
%       vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%       the number of the individual polytope vertices
%       xv: a 1-by-m vector of vertice x-coordinates
%       yv: a 1-by-m vector of vertice y-coordinates
%       distances: a 1-by-m vector of perimeter distances from one point to the
%       next point, distances(i) = distance from vertices(i) to vertices(i+1)
%       mean: average xy coordinate of the polytope
%       area: area of the polytope
%       max_radius: distance from the mean to the furthest vertex
%
% (optional inputs)
%
% throw_error: flag determining whether an error should be thrown (1) for
% points inside any polytope or no error and value assinged to ERR (0) 
%
% edge_check: a flag that determines whether the polytope edges should be
% checked for points
% 
% fig_num: a figure number to plot results. If set to -1, skips any
% input checking or debugging, no figures will be generated, and sets
% up code to maximize speed. As well, if given, this forces the
% variable types to be displayed as output and as well makes the input
% check process verbose
%
% OUTPUTS:
%
% err: value of 0 if no points in any polytopes or 1 if a point is withing
% a polytope
%
% Apoly: polytope A is on
% 
% Bpoly: polytope B is on
%
% DEPENDENCIES:
%
% none
% 
% This function was written on 2018_12_18 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method in function usage examples
% 2025_07_17 - K. Hayes
% -- copied to new function from
%    fcn_polytope_calculation_points_in_polytopes to follow library
%    convention
% 2025_08_14 - K. Hayes
% -- updated fcn header and formatting
% -- added debug plotting to fcn

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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

        % Check the A input, make sure it is struct
        assert(isstruct(A));

        % Check the B input, make sure it is struct
        assert(isstruct(B));

        % Check the polytopes input, make sure it is numeric
        assert(isstruct(polytopes));
        
    end
end

% Does user want to throw errors?
throw_error = 0; % Default is to NOT throw errors
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
       throw_error = temp;
    end
end

% Does user want to check edges?
edge_check = 0;
if 5 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        edge_check = temp;
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


err = 0;
Apoly = -1;
Bpoly = 0;
for polys = 1:size(polytopes,2)
    [Ain,Aon]=inpolygon(A.x,A.y,polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2));
    [Bin,Bon]=inpolygon(B.x,B.y,polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2));
    if Ain*~Aon==1 % if A in polygon
        if throw_error == 1
            error('Point A within obstacle')
        else
            err = 1;
            Apoly = polys;
        end
    elseif Bin*~Bon==1 % if B in polygon
        if throw_error == 1
            error('Point B within obstacle')
        else
            err = 1;
            Bpoly = polys;
        end
    end
    if edge_check == 1
        if Aon
            Apoly = polys;
        end
        if Bon
            Bpoly = polys;
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
    figure(fig_num);
    hold on;
    box on;
    
    % Plot polytopes
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;
    fcn_MapGen_plotPolytopes(polytopes, plotFormat, [1 0 0 0 1], fig_num);

    % Plot points checked
    plot(A.x, A.y, 'rx', 'DisplayName', 'A')
    plot(B.x, B.y, 'rx', 'DisplayName', 'B')

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