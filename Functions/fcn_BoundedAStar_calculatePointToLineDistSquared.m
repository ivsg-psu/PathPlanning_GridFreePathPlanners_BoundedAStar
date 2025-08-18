function [dist_sq,cross_sign] = fcn_BoundedAStar_calculatePointToLineDistSquared(pt,vert1,vert2,varargin)
% fcn_BoundAStar_calculatePointToLineDistSquared calculate distance to 
% point from a line between vert1 and vert2
%
% [dist_sq]=fcn_BoundAStar_calculatePointToLineDistSquared(pt,vert1,vert2,(fig_num))
%
% INPUTS:
%   pt: n-by-3 matrix of xyz coordinates of points around the line
% 
%   vert1: 1-by-3 vector of xyz coordinates of one line end point
% 
%   vert2: 1-by-3 vector of xyz coordinates of one line end point
%
%   (optional inputs)
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%   input checking or debugging, no figures will be generated, and sets
%   up code to maximize speed. As well, if given, this forces the
%   variable types to be displayed as output and as well makes the input
%   check process verbose
%
% OUTPUT:
%
%   dist_sq: a n-by-1 vector with the orthogonal distances to the line, where
%   n = number of points     
% 
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
% 
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from
%    fcn_general_calculation_point_to_line_distances_squared to follow library
%    convention
% 2025_08_15 - K. Hayes
% -- updated fcn header and formatting
% -- added plotting to fcn debug section

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

        % Check the pt input, make sure it is struct
        assert(isnumeric(pt));

        % Check the vert1 input, make sure it is numeric
        assert(isnumeric(vert1));
        
        % Check the vert2 input, make sure it is numeric
        assert(isnumeric(vert2));

    end
end

% Does user want to specify all_pts?
% all_pts = []; % Default is to NOT show plots
% if 2 <= nargin
%     temp = varargin{1};
%     if ~isempty(temp) % Did the user NOT give an empty figure number?
%        all_pts = temp;
%     end
% end

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

% make sure vert1 and vert2 are 1-by-3
[row,col] = size(vert1);
if col == 1
    if row == 3 % sideways
        vert1 = vert1';
    elseif row == 2 % sideways with 2 coordinates
        vert1 = [vert1' 0];
    else
        error('vert1 should have 2 to 3 coordinates')
    end
elseif col == 2 
    if row == 1 % 2 coordinates
        vert1 = [vert1 0];
    else
        error('vert1 should have 2 to 3 coordinates')
    end
elseif col == 3
    if row > 1
        error('vert1 should have 2 to 3 coordinates')
    end
else
    error('vert1 should have 2 to 3 coordinates')
end

[row,col] = size(vert2);
if col == 1
    if row == 3 % sideways
        vert2 = vert2';
    elseif row == 2 % sideways with 2 coordinates
        vert2 = [vert2' 0];
    else
        error('vert2 should have 2 to 3 coordinates')
    end
elseif col == 2 
    if row == 1 % 2 coordinates
        vert2 = [vert2 0];
    else
        error('vert2 should have 2 to 3 coordinates')
    end
elseif col == 3
    if row > 1
        error('vert2 should have 2 to 3 coordinates')
    end
else
    error('vert2 should have 2 to 3 coordinates')
end

% make sure pt is n-by-3
[row,col] = size(pt);
if col == 1
    if row == 3 % sideways
        pt = pt';
    elseif row == 2 % sideways with 2 coordinates
        pt = [pt' 0];
    else
        error('pt should have 2 to 3 coordinates per point')
    end
elseif col == 2 
    pt = [pt zeros(row,1)];
elseif col > 3
    error('pt should have 2 to 3 coordinates per point')
end

% calculate the distance squared
vert1 = repmat(vert1,size(pt,1),1);
vert2 = repmat(vert2,size(pt,1),1);
ref_vec = vert2 - vert1;
end_vec = pt - vert1;
cross_pdct = cross(ref_vec,end_vec,2);
dist_sq = sum(cross_pdct.^2,2)./sum(ref_vec.^2,2);

cross_sign = sign(cross_pdct);

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
    view(3);
   
    % Plot points
    plot3(pt(:,1),pt(:,2),pt(:,3),'b.','MarkerSize',10);

    % Plot vertices
    plot3(vert1(1), vert1(2), vert1(3), 'rx');
    plot3(vert2(1), vert2(2), vert2(3), 'rx');
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
