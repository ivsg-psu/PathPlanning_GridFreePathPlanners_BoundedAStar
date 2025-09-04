function [bound_polytopes,bound_box,bound_pts,all_bound]= fcn_BoundedAStar_calculateBoundingEllipsePolytope(start,finish,polytopes,all_pts,bound_pts,offsetperp,offsetpara,varargin)
% fcn_BoundedAStar_calculateBoundingEllipsePolytope creates a bounding box around 
% the straight line between A and B and then finds the points within the 
% bounding box and their corresponding polytopes
%
% FORMAT:
%
% [bound_polytopes,bound_box,bound_pts,all_bound]=fcn_BoundedAStar_calculateBoundingEllipsePolytope(start,finish,polytopes,all_pts,bound_pts,offsetperp,offsetpara,(figNum))
% 
% INPUTS:
% 
%   start: a 1-by-5 vector of starting point information with the same
%   information as bound_pts
% 
%   finish: a 1-by-5 vector of finishing point information with the same
%   information as bound_pts
% 
%   polytopes: the original polytopes with the same fields as BOUND_POLYTOPES 
% 
%   all_pts: a-by-5 matrix of all map points, where a = number of map points
% 
%   bound_pts: B-by-5 matrix of the original bound points, where B = number
%   of original bound points
% 
%   offsetperp: 1-by-2 vector indicating how far to offset bounding box sides 
%   from the straight line [+offset, -offset]
% 
%   offsetpara: 1-by-2 vector indicating how far to offset bounding box 
%   along the the straight line [+offset, -offset]
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
%   bound_polytopes: a 1-by-p seven field structure of bound polytopes, where  
%   p = number of bound polytopes, with fields:
%       vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%       the number of the individual polytope vertices
%       xv: a 1-by-m vector of vertice x-coordinates
%       yv: a 1-by-m vector of vertice y-coordinates
%       distances: a 1-by-m vector of perimeter distances from one point to the
%       next point, distances(i) = distance from vertices(i) to vertices(i+1)
% 
%   bound_box: a 4-by-2 matrix of xy corner points of the bounding box 
% 
%   bound_pts: a b-by-5 matrix of bound point information that is a subset of
%   previous bound points, where b = number of bound points, including:
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number 
%       beginning/ending indication (1 if the point is a beginning or ending
%       point and 0 otherwise) 
%       Ex: [x y point_id obs_id beg_end]
%
%   all_bound: w-by-5 matrix of bound point information that is a subset of
%   all map points, where w = number of all bound map points
% 
% DEPENDENCIES:
% 
% fcn_DebugTools_checkInputsToFunctions
%
% This function was written on 2018_12_21 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% REVISION HISTORY:
%
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_bounding_ellipse_polytope_bounding_box
%    to follow library convention
% 2025_09_04 - K. Hayes
% -- updated fcn header and formatting
% -- moved plotting into fcn debug section
%
% TO DO:

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 8; % The largest Number of argument inputs to the function
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
        narginchk(7,MAX_NARGIN);

        % Check the start input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            start, '5column_of_numbers');

        % Check the finish input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '5column_of_numbers');

        % Check the all_pts input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '5column_of_numbers');        

        % Check the bound_pts input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            bound_pts, '5column_of_numbers');        

        % Check the offsetperp input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            offsetperp, '2column_of_numbers');        

        % Check the offsetpara input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            offsetpara, '2column_of_numbers'); 

        % Check the polytopes input, make sure it is struct
        assert(isstruct(polytopes));
    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        figure(figNum);
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

% calculate a bounding box around the region of interest
dx = finish(1)-start(1); % change in line
dy = finish(2)-start(2);
magx = dx/sqrt(dx^2+dy^2); % magnitude of change in both directions
magy = dy/sqrt(dx^2+dy^2);
% bounding box vertices
bound_box = [(start(1)+offsetperp(1)*magy-offsetpara(1)*magx) (start(2)-offsetperp(1)*magx-offsetpara(1)*magy); (finish(1)+offsetperp(1)*magy+offsetpara(2)*magx) (finish(2)-offsetperp(1)*magx+offsetpara(2)*magy); (finish(1)-offsetperp(2)*magy+offsetpara(2)*magx) (finish(2)+offsetperp(2)*magx+offsetpara(2)*magy); (start(1)-offsetperp(2)*magy-offsetpara(1)*magx) (start(2)+offsetperp(2)*magx-offsetpara(1)*magy)];

% find vertices within bounding box
all_bound = all_pts(inpolygon(all_pts(:,1),all_pts(:,2),bound_box(:,1),bound_box(:,2)),:);
bound_pts = bound_pts(inpolygon(bound_pts(:,1),bound_pts(:,2),bound_box(:,1),bound_box(:,2)),:);

% find the corresponding obstacles
bound_polytopes = polytopes(unique(all_bound(:,4)));

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
    plotFormat.Color = 'Red';
    plotFormat.LineWidth = 2;
    fcn_MapGen_plotPolytopes(polytopes,plotFormat, [1 1 0 0 1],figNum);
    plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
    plotFormat.Color = 'Blue';
    fcn_MapGen_plotPolytopes(bound_polytopes,plotFormat,[1 0 0 1 1], figNum);
    plot(bound_box(:,1),bound_box(:,2),'k--','linewidth',2)
    plot(bound_pts(:,1),bound_pts(:,2),'go','linewidth',1)
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
function INTERNAL_fcn_format_timespace_plot()
    % define figure properties
    % opts.width      = 8.8;
    % opts.height     = 6;
    % opts.fontType   = 'Times New Roman';
    % opts.fontSize   = 14;
    % fig = gcf;
    % % scaling
    % fig.Units               = 'centimeters';
    % fig.Position(3)         = opts.width;
    % fig.Position(4)         = opts.height;
    
    % % set text properties
    % set(fig.Children, ...
    %     'FontName',     'Times New Roman', ...
    %     'FontSize',     14);
    
    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [km]')
    ylabel('y [km]')
    zlabel('t [min]')
    view([36 30])
end
