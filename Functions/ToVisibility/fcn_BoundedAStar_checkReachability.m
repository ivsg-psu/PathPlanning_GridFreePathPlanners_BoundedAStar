function [is_reachable, num_steps, rgraph_total] = fcn_BoundedAStar_checkReachability(vgraph,start_id,finish_id, varargin)
% fcn_BoundedAStar_checkReachability
%
% From the visibility graph describing node visible from each node,
% finds and analyzes the reachability graph describing nodes that have a valid
% multistep route from each node
%
%
%
% FORMAT:
% [is_reachable, num_steps, rgraph_total] = fcn_BoundedAStar_checkReachability(vgraph,start,finish, (all_pts), (fig_num))
%
%
% INPUTS:
%
%   start_id: integer ID of the start point
%
%   finish_id: integer IDs of the finish points
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%   (optional inputs)
%
%   all_pts: all points matrix for plotting purposes
% 
%   fig_num: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose
%
% OUTPUTS:
%
%     is_reachable: binary set to 1 if the finish is reachable from the start in any number of steps.  0 otherwise.
%
%     num_steps: the minimum number of steps (path segments) required to reach finish from star
%
%     rgraph_total: the total reachability graph as an nxn matrix where n is the number of pointes (nodes) in the map.
%         A 1 is in position i,j if j is reachable from point i in a path with n or fewer steps (path segments). 0 otherwise.
%
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a full test suite.
%
% This function was written on summer 2023 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2023, summer by Steve Harnett
% -- first write of function
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied function from fcn_check_reachability to follow library
%    conventions
% 2025_08_12 - K. Hayes
% -- updated fcn header and formatting 
% -- moved plotting into fcn

% TO DO:
%
% -- fill in to-do items here.

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 5; % The largest Number of argument inputs to the function
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

        % Check the vgraph input, make sure it is numeric
        assert(isnumeric(vgraph));

        % Check the start_id input, make sure it is numeric
        assert(isnumeric(start_id));
        
        % Check the finish_id input, make sure it is numeric
        assert(isnumeric(finish_id));
        
    end
end

% Does user want to specify all_pts?
all_pts = []; % Default is to NOT show plots
if 2 >= nargin
    temp = varargin{1};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
       all_pts = temp;
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

    num_pts = size(vgraph,1);
    start_id_repeated = ones(size(finish_id,1),size(finish_id,2))*start_id; % duplicate start IDs if multiple finishes
    is_reachable = 0; % initialize assuming not reachable
    rgraph_total = zeros(num_pts);
    % only want to check for paths with up to n steps because that is a path that uses every point once
    for num_steps = 1:num_pts
        % vgraph^k gives the rgraph describing reachability using paths with exactly k steps
        rgraph = vgraph^num_steps; % see Judith Gersting's Mathematical Structures for Computer Science for formula
        rgraph_total = rgraph_total + rgraph; % add rgraph for num_steps steps to all prior rgraphs with 1 to num_steps steps
        ind = sub2ind([num_pts,num_pts],start_id_repeated,finish_id);  % want to check all start finish combos
        if sum(rgraph(ind)) > 0
            is_reachable = 1; % if any start to finish combo has more than 0, some finish is reachable
        end
    end
    rgraph_total = (rgraph_total>0); % make rgraph binary

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
    % Set up plotting
    figure(fig_num)
    hold on;
    box on;
    INTERNAL_fcn_format_timespace_plot();
    
    for i = 1:length(rgraph)
        for j = 1:length(rgraph)
           if rgraph(i,j) 
                plot3([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],[all_pts(i,3),all_pts(j,3)],'-g')
           end
        end
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