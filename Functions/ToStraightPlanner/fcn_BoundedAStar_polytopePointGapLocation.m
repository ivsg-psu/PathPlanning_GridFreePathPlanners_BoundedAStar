function [gap] = fcn_BoundedAStar_polytopePointGapLocation(point,vertices,varargin)
% fcn_BoundedAStar_polytopePointGapLocation find which gap the point is in
%
% FORMAT:
%
% [gap]=fcn_BoundedAStar_polytopePointGapLocation(cross, vertices, (fig_num))
%
% INPUTS:
%
% point: crossing location on the obstacle [x y]
%
% vertices: vertices of the obstacle [x1 y1;...; xn+1 yn+1], where n is the
%   number of vertices in an obstacle
%
% (optional inputs)
%
% fig_num: a figure number to plot results. If set to -1, skips any
% input checking or debugging, no figures will be generated, and sets
% up code to maximize speed. As well, if given, this forces the
% variable types to be displayed as output and as well makes the input
% check process verbose
%
% OUTPUTS:
%
% GAP: the gap the point of interest is between, 
%   ex: gap = 1 means vert1 <= point < vert2
%       gap = 4 means vert4 <= point < vert5
%
% This function was written on 2018_12_21 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from
%    fcn_polytope_calculation_point_gap_location to follow library convention
% 2025_08_25 - K. Hayes
% -- updated fcn header and formatting
% -- moved debug plotting into fcn

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 3; % The largest Number of argument inputs to the function
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
        narginchk(2,MAX_NARGIN);

        % Check the point input, make sure it has 2 columns
        fcn_DebugTools_checkInputsToFunctions(...
            point, '2column_of_numbers');

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

% xy-vertices and crossing point
x1 = vertices(1:end-1,1);
y1 = vertices(1:end-1,2);
x2 = vertices(2:end,1);
y2 = vertices(2:end,2);
xi = point(1);
yi = point(2);

% check if the point is on the line between two points
acc = 1e-8;
TF = fcn_BoundedAStar_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc); % row vector of 1 or 0 if on lines between (x1,y1) and (x2,y2)
gap = find(TF,1);

% ensure gap is defined
while isempty(gap)
    acc = acc*10; % decrease accuracy to account for changes in variable types
    TF = fcn_BoundedAStar_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc); % row vector of 1 or 0 if on lines between (x1,y1) and (x2,y2)
    gap = find(TF,1);
    if acc > 100 % if point too far stop searching
        error('Intersecting point does not appear to be on the line')
    end
end
if acc > 1e-8 % warn if the point is too far away
    warning('Gap assignment may be incorrect, accuracy decreased to %.0e',acc)
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
    figure(fig_num)
    hold on;
    box on;
    plot(vertices(:,1),vertices(:,2),'k-','linewidth',3, 'DisplayName', 'Polytope')
    hold on
    plot(point(1),point(2),'gx','linewidth',1, 'DisplayName', 'Crossing point', 'LineWidth',3)
    axis square

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