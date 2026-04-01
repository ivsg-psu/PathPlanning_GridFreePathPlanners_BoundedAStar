function region = fcn_BoundaryXP_makeRegion(boundaryPoints, varargin)
% fcn_BoundaryXP_makeRegion
% replaces an internal function which converts a set of vertices of a
% region into a polyshape for boundary checking operations
%
% FORMAT:
%
% region = fcn_BoundaryXP_makeRegion(boundaryPoints, (figNum))
%
% INPUTS:
%     
%     boundaryPoints: the Nx2 matrix containing the vertices in the
%     boundary to be converted to a region
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     region: a polyshape representing the region outlined by the vertices
%     in boundaryPoints
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundaryXP_makeRegion
% for a full test suite.
%
% This function was written on 2026_04_01 by K. Hayes 
% Questions or comments? contact S. Brennan sbrennan@psu.edu
% or K. Hayes, kaeleahayes@psu.edu

% REVISION HISTORY:
% 2026_04_01 - K. Hayes
% -- First write of function using fcn_BoundaryXP_checkBoundaries as a
%    starter
%
% TO-DO
%

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 2; % The largest Number of argument inputs to the function
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
		narginchk(1,MAX_NARGIN);

		% Check the radii input, make sure it is '1column_of_numbers'
		% type, 1 row
		%fcn_DebugTools_checkInputsToFunctions(...
			%radii, '1column_of_numbers');
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

uniquePoints = flipud(regionBoundary); % for some silly reason, polyshape takes points "backwards" (?!)
uniquePoints = unique(uniquePoints,'rows','stable');
region = polyshape(uniquePoints,'KeepCollinearPoints', true,'Simplify', false);


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
	figure(figNum)

	% Check to see if hold is already on. If it is not, set a flag to turn it
	% off after this function is over so it doesn't affect future plotting
	flag_shut_hold_off = 0;
	if ~ishold
		flag_shut_hold_off = 1;
		hold on
	end

	% Turn on legend
	legend('Interpreter','none','Location','best');

	plot(boundaryPoints(:,1), boundaryPoints(:,2), '.', 'MarkerSize', 10)
    plot(region)

    axis equal
	grid on;
	% Shut the hold off?
	if flag_shut_hold_off
		hold off;
	end

end % Ends the flag_do_plot if statement

if flag_do_debug
	fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end


end % Ends the main function



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

