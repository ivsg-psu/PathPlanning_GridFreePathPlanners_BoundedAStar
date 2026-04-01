function [pointsMovedOutward, movementsOutward, indicesKept, varargout] = fcn_BoundaryXP_expandVerticesOutward(startingPoints, radius, varargin)
% fcn_BoundaryXP_expandVerticesOutward
% replaces an internal function which performed the math to find the
% orthogonal direction from the previous boundary and expand the points in
% that direction by a specified radius
%
% FORMAT:
% [pointsMovedOutward, movementsOutward, indicesKept, overLimitIndices] = ...
%   fcn_BoundaryXP_expandVerticesOutward();
%overLimitIndices
% INPUTS:
%     
%     startingPoints: the Nx2 matrix containing the coordinates of each
%     point in the boundary to be expanded.
% 
%     radius: a scalar or vector describing the distance by which to expand
%     the boundary. If a scalar is given, all points will be expanded by
%     that number. If a vector is given, it must be Nx1, specifying a
%     radius for each point in startingPoints.
%
%     (optional inputs)
%
%     prevStates: the Nx2 matrix containing the non-location/energy states
%     of the system at each point in the boundary.
% 
%     platModel: a Model object containing the following properties:
% 
%           Platform: a structure containing platform-specific information, with
%           fields:
%               MassState: current fuel mass state
%               TempState: current fuel temperature state
%               Airframe: aerodynamic information, containing subfields
%               emptyWeight, CDv, CLv, k, Sref
%               Ambient: ambient environmental information, containing
%               subfields T_C and rho_slgpft3
%               HeatLoad: preset heat loads, with subfields indicating High and
%               Low operating modes
%               TMS: thermal management system information, with subfields
%           I   InletArea_ft2, Treject_C, cp_air_KJpkgK, and cp_fuel_kJpkgK
%
%           route: a structure containing route-specific information, with
%           fields:
%               v_ftps: platform nominal velocity in ft/s
%               mode: High/Low, indicating HeatLoad operation mode
%               fpa_rad: angle between flight path and ground, in radians
%
%     figNum: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose.
%
% OUTPUTS:
%
%     movementsOutward: an Mx2 matrix which specifies the outward movements
%     associated with each point in the previous boundary
%
%     indicesKept: a vector indicating which points were kept after
%     post-processing the points
%
%     (optional outputs)
%
%     overLimitIndices: a vector of logical indices indicating which points
%     in the boundary would require violation of a specified thrust limit.
%     For use only in range-altitude calculations.
%
% DEPENDENCIES:
%
%     fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundedAStar_altRangeVariableRadius
% for a full test suite.
%
% This function was written on 2026_03_19 by K. Hayes
% Questions or comments? contact S. Brennan sbrennan@psu.edu
% or K. Hayes, kaeleahayes@psu.edu

% REVISION HISTORY:
% 2026_03_19 by K. Hayes
% -- first write of function, using fcn_BoundedAStar_altRangeDoExpansion as
%    starter
%
% 2026_03_25 by K. Hayes
% -- minor changes to support move to OOP model handling
%
% TO-DO
% -- fix plotting


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
		narginchk(2,MAX_NARGIN);

		% Check the radii input, make sure it is '1column_of_numbers'
		% type, 1 row
		%fcn_DebugTools_checkInputsToFunctions(...
			%radii, '1column_of_numbers');
	end
end

% Does user want to specify states?
prevStates = [];
if 3 <= nargin
	temp = varargin{1};
	if ~isempty(temp) % Did the user NOT give an empty figure number?
		prevStates = temp;
	end
end

% % Does user want to specify Platform structure?
% Platform = [];
% if 4 <= nargin
% 	temp = varargin{2};
% 	if ~isempty(temp) % Did the user NOT give an empty figure number?
% 		Platform = temp;
% 	end
% end
% 
% % Does user want to specify route structure?
% route = [];
% if 5 <= nargin
% 	temp = varargin{3};
% 	if ~isempty(temp) % Did the user NOT give an empty figure number?
% 		route = temp;
% 	end
% end

platModel = [];
if 4 <= nargin
    temp = varargin{2};
    if ~isempty(temp) 
        platModel = temp;
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

nout = max(nargout,1)-3;
if nout>=1
	varargout = cell(nout,1);
	flagRangeAltitude = 1;
else
	flagRangeAltitude = 0;
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

%%%%
% Need to expand all vertices outward. A method to do this is to
% calculate the projection vector at each vertex that bisects the
% angle, and thus projects straight "outward". The distance of this
% vector can be calculated such that both edges move outward by the
% same radius. The code below implements this method.

% Make sure first and last points repeat
if ~isequal(startingPoints(1,:),startingPoints(end,:))
	startingPoints = [startingPoints; startingPoints(1,:)];
end

%%%%%%% move this to a test case when test script for this function
%%%%%%% developed
% for debugging
if 1==0
	startingPoints = [0 0; 1 0; 2 0; 3 0; 4 0; 3 0; 2 0; 2 1];
end
%%%%%%%

% Find unit edge vectors, and ortho projections. Note: if there are N
% vertices, there will be N-1 edges. So that the angles match with the
% end of each point, we repeat the 1st edge vector onto the end so the
% number of edge vectors is equal to the number of points.
edgeVectors = startingPoints(2:end,:)-startingPoints(1:end-1,:);
edgeLengths = sum(edgeVectors.^2,2).^0.5;
unitEdgeVectors = edgeVectors./edgeLengths;
allUnitEdgeVectors = [unitEdgeVectors(end,:); unitEdgeVectors; unitEdgeVectors(1,:)];

if any(isnan(allUnitEdgeVectors),'all')
	error('Unexpected error in calculating allUnitEdgeVectors: NaN value encountered - this should not happen.');
end

%%%
% Remove "jogs"

% Sometimes vectors are created that point exactly opposite each other.
% Remove these
flagKeepGoing = 1;
count = 0;
NStartingPoints = length(startingPoints(:,1));
fixedVectors = [allUnitEdgeVectors, [NStartingPoints; (1:NStartingPoints)']];

indicesRemoved = [];
while 1==flagKeepGoing
	temp = fixedVectors(1:end-1,1:2)+fixedVectors(2:end,1:2);
	tempMag = sum(temp.^2,2);
	tol = 1E-8;
	badElement = find(tempMag<tol,1);
	if ~isempty(badElement)
		indicesRemoved = [indicesRemoved; fixedVectors(badElement:(badElement+1),3)]; %#ok<AGROW>
		fixedVectors(badElement:(badElement+1),:) = [];
	else
		flagKeepGoing = 0;
	end
	count = count+1;
	if count>=NStartingPoints
		error('Unable to clean vectors - seem to be trapped while removing forward/backward pairs.');
	end
end
indicesKept = fixedVectors(2:end,3);
fixedStartingPoints = startingPoints(indicesKept,:);
if ~isequal(fixedStartingPoints(1,:),fixedStartingPoints(end,:))
	indicesKept = [indicesKept; indicesKept(1)];
	fixedStartingPoints = startingPoints(indicesKept,:);
	fixedVectors = [fixedVectors; fixedVectors(1,:)];
end
indicesRemoved = sort(indicesRemoved);

% For debugging. Should see any forward/backward jogs removed. NOTE:
% because the analysis is done on UNIT edge vectors, it may seem like some
% of the points prior to the "jog" in/out are removed. This is normal.
if 0==1
	figure(44444);
	clf;
	plot(startingPoints(:,1),startingPoints(:,2),'b.-','MarkerSize',40, 'LineWidth',5);
	hold on;
	plot(startingPoints(indicesRemoved,1),startingPoints(indicesRemoved,2),'r.','MarkerSize',30, 'LineWidth',5);
	plot(fixedStartingPoints(:,1),fixedStartingPoints(:,2),'g.-','MarkerSize',20, 'LineWidth',2);
end

%%%
% Find offsets

% The following method avoids the use of sines, cosines, and tangents
% as those are slow to calculate. To find the vector projection from
% each point, the average of the projection vector tips at each vertex
% angle is calculated. The projection scaling distance, D, is
% calculated by using the relationship that the sin(theta) = x/D = a/x,
% then solving for D and noting that x=1. Here, theta is the half angle
% formed by the "kite" apex, created by the before/after projection of
% the orthogonal vectors from each startPoint.

% Perform a -90 degree rotation
orthoEdgeVectors = fixedVectors(:,1:2)*[0 -1; 1 0];
positionsIncomingVector = fixedStartingPoints+orthoEdgeVectors(1:end-1,:);
positionsOutgoingVector = fixedStartingPoints+orthoEdgeVectors(2:end,:);
averagePositions = (positionsIncomingVector + positionsOutgoingVector)/2;

% Remove values where average positions are zero
averagePositionsMagnitude = sum(averagePositions.^2,2);
badPoints = find(averagePositionsMagnitude<1E-6);
if ~isempty(badPoints)
	averagePositions(badPoints,:) = [];
	fixedStartingPoints(badPoints,:) = [];
	indicesKept(badPoints,:) = [];
end

averagePositionVectors = averagePositions - fixedStartingPoints;
averagePositionVectorLengths = sum(averagePositionVectors.^2,2).^0.5;

badPoints2 = find(averagePositionVectorLengths<1E-6);
if ~isempty(badPoints2)
	averagePositionVectors(badPoints2,:) = [];
	averagePositionVectorLengths(badPoints2,:) = [];
	fixedStartingPoints(badPoints2,:) = [];
	indicesKept(badPoints2,:) = [];
end

unitProjectionVectors = averagePositionVectors./averagePositionVectorLengths;


fixedRadius = radius;

if length(radius) > 1
    if size(fixedRadius,1) < indicesKept(end)
	    for i = 1:(indicesKept(end) - size(fixedRadius,1))
		    fixedRadius(end+1,:) = radius(end);
	    end
    end
    fixedRadius = fixedRadius(indicesKept);
end

if any(isnan(unitProjectionVectors),'all')
	error('Unexpected error in calculating unitProjectionVectors: NaN value encountered - this should not happen.');
end

% Use the sin(theta) method to calculate D
D = 1./averagePositionVectorLengths;
movementsOutward = fixedRadius.*D.*unitProjectionVectors;

if flagRangeAltitude
    % Use movements outward to check thrust limitation
    vectorAngles = atan2(movementsOutward(:,2), movementsOutward(:,1));
    platModel.route.fpa_rad = vectorAngles;
    

    thrust = computeThrustReq(platModel, prevStates);
    thrustLimits = 2000*platModel.Platform.Ambient.rho_slgpft3/0.001096221635506;
    
    % Remove movements outward for cases where thrust required exceeds limit
    overLimitIndices = thrust > thrustLimits;
end

% Apply thrust limited movementsOutward
RawPointsMovedOutward = fixedStartingPoints + movementsOutward;

% Make sure first and last points repeat
if ~isequal(RawPointsMovedOutward(1,:),RawPointsMovedOutward(end,:))
	RawPointsMovedOutward(end,:) = RawPointsMovedOutward(1,:);
	indicesKept(end) = indicesKept(1);
end

if flagRangeAltitude
    negativeIndices = RawPointsMovedOutward(:,1) < 0;
    RawPointsMovedOutward(negativeIndices,2) =0;
end

% Save output
pointsMovedOutward = RawPointsMovedOutward;
if flagRangeAltitude
    varargout{1} = overLimitIndices;
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

	plot(startingPoints(:,1),startingPoints(:,2),'.-','MarkerSize',20);
	hold on;
	plot(pointsMovedOutward(:,1),pointsMovedOutward(:,2),'.-','MarkerSize',20,'LineWidth',2);
	quiver(startingPoints(:,1), startingPoints(:,2), pointsMovedOutward(:,1)-startingPoints(:,1), pointsMovedOutward(:,2)-startingPoints(:,2))
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

