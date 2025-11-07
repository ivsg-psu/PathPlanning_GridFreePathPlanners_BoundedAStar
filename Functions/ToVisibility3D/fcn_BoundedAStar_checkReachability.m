function [flagsIsReachable, numSteps, rgraphTotal] = fcn_BoundedAStar_checkReachability(...
    vgraph,startIndex,finishIndex, varargin)
% fcn_BoundedAStar_checkReachability
%
% From the visibility graph describing node visible from each node, finds
% and analyzes the reachability graph describing nodes that have a valid
% multistep route from each node
%
% FORMAT:
% [flagsIsReachable, numSteps, rgraphTotal] = fcn_BoundedAStar_checkReachability(...
%   vgraph, startIndex, finishIndex, (pointsWithData), (figNum))
%
% INPUTS:
%
%   vgraph: the visibility graph as an nxn matrix where n is the number of
%   points (nodes) in the map.
%       A 1 is in position i,j if point j is visible from point i.  0
%       otherwise.
%
%   startIndex: integer ID of the start point
%
%   finishIndex: integer IDs of the finish points
%
%   (optional inputs)
%
%   pointsWithData: all points matrix for plotting purposes
%
%   figNum: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the
%       input check process verbose
%
% OUTPUTS:
%
%     flagsIsReachable: binary set to 1 if the finish is reachable from the
%     start in any number of steps.  0 otherwise.
%
%     numSteps: the minimum number of steps (path segments) required to
%     reach finish from star
%
%     rgraphTotal: the total reachability graph as an nxn matrix where n is
%     the number of pointes (nodes) in the map.
%         A 1 is in position i,j if j is reachable from point i in a path
%         with n or fewer steps (path segments). 0 otherwise.
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
% 2025_11_06 - S. Brennan
% -- added waitbar
% -- fixed inequality bug in input area
% -- Cleaned up variable naming:
%    % * From fig_num to figNum
%    % * From all_pts to pointsWithData
%    % * From start_id to startIndex
%    % * From finish_id to finishIndex
%    % * From is_reachable to flagsIsReachable
%    % * From num_steps to numSteps
%    % * From rgraph_total to rgraphTotal
% -- fixed docstrings (variables out of order, wrong names, etc)
% -- fixed plotting to preallocate plotting data, do plot in one command

% TO DO:
% -- fill in to-do items here.

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
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
    debug_figNum = 999978; %#ok<NASGU>
else
    debug_figNum = []; %#ok<NASGU>
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

        % Check the startIndex input, make sure it is numeric
        assert(isnumeric(startIndex));

        % Check the finishIndex input, make sure it is numeric
        assert(isnumeric(finishIndex));

    end
end

% Does user want to specify pointsWithData?
pointsWithData = []; % Default is to NOT show plots
if 2 <= nargin
    temp = varargin{1};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        pointsWithData = temp;
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

num_pts = size(vgraph,1);
startIndex_repeated = ones(size(finishIndex,1),size(finishIndex,2))*startIndex; % duplicate start IDs if multiple finishes
flagsIsReachable = 0; % initialize assuming not reachable
rgraphTotal = zeros(num_pts);
% only want to check for paths with up to n steps because that is a path that uses every point once

h_waitBar = waitbar(0, 'Processing intersections...');
updateInterval = round(num_pts/100);
nextUpdate = updateInterval;
for numSteps = 1:1:num_pts
    if numSteps>nextUpdate
        waitbar(numSteps / num_pts, h_waitBar, sprintf('Processing expansion %d of %d...', numSteps, num_pts));
        nextUpdate = nextUpdate + updateInterval;
    end

    % vgraph^k gives the rgraph describing reachability using paths with exactly k steps
    rgraph = vgraph^numSteps; % see Judith Gersting's Mathematical Structures for Computer Science for formula
    rgraphTotal = rgraphTotal + rgraph; % add rgraph for numSteps steps to all prior rgraphs with 1 to numSteps steps
    ind = sub2ind([num_pts,num_pts],startIndex_repeated,finishIndex);  % want to check all start finish combos
    if sum(rgraph(ind)) > 0
        flagsIsReachable = 1; % if any start to finish combo has more than 0, some finish is reachable
    end
end
close(h_waitBar);

rgraphTotal = (rgraphTotal>0); % make rgraph binary

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
    figure(figNum)
    hold on;
    box on;
    fcn_INTERNAL_formatTimespacePlot();

    % Grab data to plot
    numEdges = length(find(rgraph));
    dataToPlot = nan(numEdges*3,3);
    nFound = 0;
    for ith_row = 1:length(rgraph)        
        for jth_col = 1:length(rgraph)
            if rgraph(ith_row,jth_col)
                nFound = nFound+1;
                rowToFill = (nFound-1)*3+1;
                dataToPlot(rowToFill,:)   = pointsWithData(ith_row,1:3); 
                dataToPlot(rowToFill+1,:) = pointsWithData(jth_col,1:3); 
                % plot3([pointsWithData(i,1),pointsWithData(j,1)],[pointsWithData(i,2),pointsWithData(j,2)],[pointsWithData(i,3),pointsWithData(j,3)],'-g')
            end
        end
    end

    plot3(dataToPlot(:,1), dataToPlot(:,2), dataToPlot(:,3), '-g')
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
function fcn_INTERNAL_formatTimespacePlot()
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