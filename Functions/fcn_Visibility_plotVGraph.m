function fcn_Visibility_plotVGraph(vgraph, all_pts,  styleString, varargin)
% fcn_Visibility_plotVGraph
%
% Plots a visibility graph given a generated vgraph and list of all points
%
% FORMAT:
%
% fcn_Visibility_plotVGraph(vgraph, all_pts,  styleString)
%
% INPUTS:
%
%    vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
%    A 1 is in position i,j if point j is visible from point i.  0 otherwise.
%
%    all_pts: the nx5 list of all points in the space to be searched, with
%    the exception of the start and finish, with columns containing the
%    following information
%       x-coordinate
%       y-coordinate
%       point id number
%       obstacle id number (-1 if none)
%       is beginning/end of obstacle (1 if yes, 0 if no)
%
%    styleString: string of format '-g' indicating the line style to be
%    used when plotting the visibility graph
%
%
% OUTPUTS:
%
% 
% DEPENDENCIES:
%
% fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_3d_polytope_multiple
% for a full test suite.
%

% REVISION HISTORY:
%
% 2025_10_08 - K. Hayes, kaeleahayes@psu.edu
% -- added function header and standard formatting
%
% TO DO:
%

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

        % Check the all_surfels input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            all_pts, '5column_of_numbers');

    end
end

% % Does user want to show the plots?
% flag_do_plots = 0; % Default is to NOT show plots
% if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
%     temp = varargin{end};
%     if ~isempty(temp) % Did the user NOT give an empty figure number?
%         fig_num = temp;
%         figure(fig_num);
%         flag_do_plots = 1;
%     end
% end

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
Npoints = size(vgraph,1);
for ith_fromIndex = 1:Npoints
    pointsToPlot = [];
    for jth_toIndex = 1:size(vgraph,1)
        if vgraph(ith_fromIndex,jth_toIndex) == 1
            % plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
            % pause(0.01);
            pointsToPlot = [pointsToPlot; [all_pts(ith_fromIndex,1:2); all_pts(jth_toIndex,1:2); nan(1,2)]]; %#ok<AGROW>

        end
    end
    plot(pointsToPlot(:,1),pointsToPlot(:,2),styleString)
    drawnow;

    if 1==0 % To save movie
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image

        % Write the frame to the GIF
        if ith_fromIndex == 1
            % Create a new GIF file for the first frame
            imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
        else
            % Append subsequent frames
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
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

end % Ends the function

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