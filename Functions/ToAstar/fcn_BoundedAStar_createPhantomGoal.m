function [vgraph, cgraph, hvec, finish, all_pts] = fcn_BoundedAStar_createPhantomGoal(vgraph, cgraph, hvec, finish, all_pts, varargin)
% fcn_BoundedAStar_createPhantomGoal
%
% this function creates a fictional "phantom goal" connected to all true goals
% this phantom goal exists in the graph but has no spatial or temporal state
% if A* is instructed to find the phantom goal, it will pass through the optimal
% true goal on the way.  Therefore this method can be used to transform a multi-
% goal problem into a single-goal problem.  See:
% Section 6.5 of Stephen Harnett's PhD dissertation
% Likhachev, M. (2019) 6-350 Planning Techniques for Robotics Search Algorithms:
% Multi-goal A*, IDA*, Tech. rep., Carnegie Mellon University.
% https://www.cs.cmu.edu/~maxim/classes/robotplanning/
% https://www.cs.cmu.edu/~maxim/classes/robotplanning/lectures/informedastar_16350_sp25.pdf
%
%
% FORMAT:
% [vgraph, cgraph, hvec, finish, all_pts] = fcn_BoundedAStar_createPhantomGoal(vgraph, cgraph, hvec, finish, all_pts, (fig_num))
%
% INPUTS:
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%    cgraph: the cost graph matrix. A cost matrix is an nxn matrix where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element i-j is the cost of routing from i to j.
%
%    hvec: the heuristic cost vector. A 1xn vector where n is
%      the number of points (nodes) in the map including the start and goal.
%      The value of element k is the estimated cost of routing from point k to
%      the finish based on a heuristic cost estimation method.
%
%   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,id)
%
%   all_pts: the point matrix of all point that can be in the route, except the start and finish where
%       each row is a single point vector (x,y,id)
%
%   (optional inputs)
%
%   fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
% OUTPUTS:
%
%   Outputs are the same as the inputs except modified to include the phantom goal point
%    which is connected to each input finish point with a zero weight edge
%
% DEPENDENCIES:
%
%   none
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a test that can be run by flagging on "do_phantom".
%
% This function was written on spring 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% Feb 2 2024, spring by Steve Harnett
% -- first write of function
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- moved function to new file from fcn_algorithm_create_phantom_goal.m to
%    follow library conventions
% 2025_08_12 - K. Hayes
% -- updated fcn header and formatting
% -- moved plotting into fcn
%
% TO DO:
%
% -- fill in to-do items here.

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
        narginchk(5,MAX_NARGIN);

        % Check the vgraph input, make sure it is numeric
        assert(isnumeric(vgraph));

        % Check the cgraph input, make sure it is numeric
        assert(isnumeric(cgraph));

        % Check the hvec input, make sure it is numeric
        assert(isnumeric(hvec));

        % Check the finish input, make sure it has 4 columns
        fcn_DebugTools_checkInputsToFunctions(...
            finish, '4column_of_numbers');

        % Check the all_pts input, make sure it has 4 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '4column_of_numbers');

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

    % Save finish vector for plotting
    finishplot = finish;

    % create a new vgraph column and row of zeros for the imaginary finish
    orig_size = size(vgraph,1);
    new_row = zeros(1,orig_size);
    new_col = new_row';
    vgraph = [[vgraph, new_col]; [new_row, 1]];

    % row can stay zeros (don't need to go from finish to anywhere)
    % column gets a 1 at every finish ID
    % this implies you can go from each finish to the phantom goal
    finish_idx = finish(:,4);
    vgraph(finish_idx,end) = 1;

    % cgraph gets a cost of 0 to go from each finish ID to the phantom
    % and a cost of inf. otherwise
    new_row = inf*ones(1,orig_size);
    new_col = inf*new_row';
    cgraph = [cgraph, new_col; new_row, inf];
    cgraph(finish_idx,end) = 0;

    % HVEC gets a 1 at every finish ID
    hvec(finish_idx) = 1;
    hvec = [hvec 0];

    % append old finish to all_pts since the original finishes are now considered
    % ordinary points in the all_pts table (which conventionally excludes starts
    % and finishes)
    all_pts = [all_pts; finish];

    % make phantom finish: recall it doesn't need x, y, or t coords so these are NaN
    finish = [NaN NaN NaN max(finish_idx)+1];

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
   % Plot formatting
   figure(fig_num);
   view(3);
   hold on;
   box on;
   INTERNAL_fcn_format_timespace_plot();
   
   % Plot all_pts
   plot3(all_pts(:,1), all_pts(:,2), all_pts(:,3), '.', 'Color', 'black', 'DisplayName', 'All Points');
   plot3(finishplot(:,1),finishplot(:,2),finishplot(:,3), 'rx','MarkerSize',10,'LineWidth',2, 'DisplayName', 'Finishes');
   % plot3(start(:,1),start(:,2),start(:,3),'gx','MarkerSize',10,'LineWidth',2, 'DisplayName', 'Starts');

   legend
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