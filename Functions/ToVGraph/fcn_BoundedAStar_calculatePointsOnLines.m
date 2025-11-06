function [tf] = fcn_BoundedAStar_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc,varargin)
% fcn_BoundedAStar_calculatePointsOnLines determines whether the points are
% on lines between point set 1 and point set 2
%
% FORMAT: 
%
% [TF]=fcn_BoundedAStar_calculatePointsOnLines(X1,Y1,X2,Y2,XI,YI,ACC)
%
% INPUTS: 
%
% x1: x values for the polytope
% 
% y1: y values for the polytope
% 
% x2: x values from X1 with the first value moved to the end
% 
% y2: y values from Y1 with the first value moved to the end
% 
% xi: the x values for the points of interest
% 
% yi: the y values for the points of interest
% 
% acc: variable for determining how close to the line the point of interest
% must be to be considered "on" the line (accounts for calculation
% rounding)
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
% tf: n-by-1 vector of True(1) or False(0) values determining whether the
% point is on the line between the corresponding points in [X1 Y1] and
% [X2 Y2]
% 
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_calculatePointsOnLines
% for a full test suite.
%
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_general_calculation_points_on_lines
%    to follow library convention
% 2025_08_19 - K. Hayes
% -- updated fcn header and formatting

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
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

        % Check the x1 input, make sure it is numeric
        assert(isnumeric(x1));

        % Check the y1 input, make sure it is numeric
        assert(isnumeric(y1));
        
        % Check the x2 input, make sure it is numeric
        assert(isnumeric(x2));

        % Check the y2 input, make sure it is numeric
        assert(isnumeric(y2));

        % Check the xi input, make sure it is numeric
        assert(isnumeric(xi));

        % Check the yi input, make sure it is numeric
        assert(isnumeric(yi));

        % Check the acc input, make sure it is numeric
        assert(isnumeric(acc));
        
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


% rows of TF correspond to points in xi,yi
% columns of TF correspond to lines composed from x1,y1,x2,y2

x1s = size(x1);
x2s = size(x2);
y1s = size(y1);
y2s = size(y2);
xis = size(xi);
yis = size(yi);
if (sum(x1s==x2s)+sum(y1s==y2s)+sum(x1s==y1s)) ~= 6
    error('x1,y1,x2,y2 must all have the same dimensions')
end
if sum(xis==yis) ~= 2
    error('xi and yi must have the same dimensions')
end

row = xis(1);
col = xis(2);
if col ~= 1 % 0 or more than 1 column
    if row ~= 1 % xi and yi are matrices or empty
        error('xi and yi must be vectors')
    else % need to transpose xi and yi
        xi = xi';
        yi = yi';
        row = col;
    end
else % 1 column vector
    if row == 0
        error('xi and yi are empty vectors')
    end
end
col = length(x1);

tf = zeros(row,col); % numerical true or false matrix
dx = x2-x1;
dy = y2-y1;
for val = 1:length(dx) % check each dx dy value
    if abs(dx(val)) <= acc
%         tf(:,val) = (abs(xi - x1(val))<acc).*((((y1(val)-yi)<acc).*((yi-y2(val))<acc))+(((y2(val)-yi)<acc).*((yi-y1(val))<acc)));
        tf(:,val) = (abs(xi - x1(val))<acc).*(sign(dy(val)*(yi-y1(val))) + sign(dy(val)*(y2(val)-yi)) > 0);
    elseif abs(dy(val)) <= acc
%         tf(:,val) = (abs(yi - y1(val))<acc).*((((x1(val)-xi)<acc).*((xi-x2(val))<acc))+(((x2(val)-xi)<acc).*((xi-x1(val))<acc)));
        tf(:,val) = (abs(yi - y1(val))<acc).*(sign(dx(val)*(xi-x1(val))) + sign(dx(val)*(x2(val)-xi)) > 0);
    else
%         tf(:,val) = (abs(yi - ((dy(val)/dx(val)).*(xi-x1(val)) + y1(val)))<acc).*((((x1(val)-xi)<acc).*((xi-x2(val))<acc))+(((x2(val)-xi)<acc).*((xi-x1(val))<acc))); 
        tf(:,val) = (abs(yi - ((dy(val)/dx(val)).*(xi-x1(val)) + y1(val)))<acc).*(sign(dx(val)*(xi-x1(val))) + sign(dx(val)*(x2(val)-xi)) > 0); 
    end
end

tf = tf==1; % logical true or false matrix


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
    ind = [];
    figure(fig_num)
    plot([x1 x1(1)],[y1 y1(1)],'k-','linewidth',1)
    hold on
    
    ind = find(tf==1);
    if ~isempty(ind)
        plot(xi,yi,'gx','linewidth',2)
    else
        plot(xi,yi,'rx','linewidth',2)
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