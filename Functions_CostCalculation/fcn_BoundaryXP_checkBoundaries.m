function boundedPoints = fcn_BoundaryXP_checkBoundaries(unboundedPoints, boundingRegion, axisRange, varargin)
% fcn_BoundaryXP_checkBoundaries
% replaces a previously internal function which checked the expanded
% boundary against the 'legal' region, which represents the mapped region
% and any areas not inside a designated keep-out zone
%
% FORMAT:
% 
%       boundedPoints = fcn_BoundaryXP_checkBoundaries(unboundedPoints, ...
%           boundingRegion, maxX, maxY, minX, minY, axisRange, (keepOutZones),...
%           (figNum));
%
% INPUTS:
%     
%       unboundedPoints: an Nx2 matrix of the vertices of the expanded
%       boundary before any checks are applied
%
%       boundingRegion: a polyshape representing the region that is mapped
%
%       axisRange: the 1x4 vector listing the maximum and minimum bounds of
%       the axes, in the format [minX, minY, maxX, maxY]       
%
%       (optional inputs)
%
%       keepOutZones: a cell array of polyshapes representing areas of the
%       map that the vehicle cannot traverse
%
%       figNum: a figure number to plot results. If set to -1, skips any
%       input checking or debugging, no figures will be generated, and sets
%       up code to maximize speed. As well, if given, this forces the
%       variable types to be displayed as output and as well makes the input
%       check process verbose.
%
% OUTPUTS:
%
%       boundedPoints: an Nx2 matrix of the vertices of the expanded
%       boundary after checks with the 
%
% DEPENDENCIES:
%
%       fcn_DebugTools_checkInputsToFunctions
%       fcn_BoundaryXP_makeRegion
%
% EXAMPLES:
%
% See the script: script_test_fcn_BoundaryXP_checkBoundaries
% for a full test suite.
%
% This function was written on 2026_04_01 by K. Hayes
% Questions or comments? contact S. Brennan sbrennan@psu.edu
% or K. Hayes, kaeleahayes@psu.edu

% REVISION HISTORY:
% 2026_04_01 - K. Hayes
% -- First write of function
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
		narginchk(3,MAX_NARGIN);

		% Check the radii input, make sure it is '1column_of_numbers'
		% type, 1 row
		%fcn_DebugTools_checkInputsToFunctions(...
			%radii, '1column_of_numbers');
	end
end

% Does user want to specify states?
keepOutZones = [];
if 4 <= nargin
	temp = varargin{1};
	if ~isempty(temp) % Did the user NOT give an empty figure number?
		keepOutZones = temp;
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
% Bound the XY values
if 1==1
    % TO DO - functionalize this below
    % NEW way
    unboundedRegion = fcn_BoundaryXP_makeRegion(unboundedPoints);

    if ~isempty(keepOutRegions)
        nRegions = numel(keepOutRegions);
        for ith_region = 1:nRegions
            unboundedRegion = subtract(unboundedRegion, keepOutRegions{ith_region});
        end

    end

    insideRegion = intersect(boundingRegion, unboundedRegion);

    indicatorRow = nan;
    % Check for NaN values due to enclosed region
    if any(isnan(insideRegion.Vertices)) 
        keepRow = find(~isnan(insideRegion.Vertices(:,1)));
        indicatorRow = find(isnan(insideRegion.Vertices(:,1)));
        intShape1 = [insideRegion.Vertices(1:indicatorRow-1,:); insideRegion.Vertices(1,:)];
        intShape2 = [insideRegion.Vertices(indicatorRow+1:end,:); insideRegion.Vertices(indicatorRow+1,:)];
        
        if size(intShape1, 1) > size(intShape2,1)
            intShape = intShape1;
        else
            intShape = intShape2;
        end

        % Fix edge case where outer boundaries get flipped into the first
        % 'boundary' because they are equal size in the final expansion
        if intShape(1,:) == boundingRegion.Vertices(1,:)
            insideRegion.Vertices = intShape;
        % Fix edge case where boundary gets flipped and we keep the wrong
        % shape
        elseif ~isempty(keepOutRegions)
            if insideRegion.Vertices(indicatorRow+1,:) == keepOutRegions{1}.Vertices(1,:)
                insideRegion.Vertices = intShape;
            end
        else
            %insideRegion.Vertices = insideRegion.Vertices(indicatorRow+1:end,:);
            insideRegion.Vertices = intShape;
        end
    end

    boundedVertices = flipud(insideRegion.Vertices);
    newStartPoints = [ boundedVertices(end,:); boundedVertices];  
    boundedPoints = newStartPoints;

    if 1==0
        figure(1010)
        hold on
        plot(startPointsSparse(:,1), startPointsSparse(:,2), 'LineWidth', 3, 'MarkerSize', 10);
        plot(unboundedRegion)
        plot(insideRegion)
    end
    

    % If region breaks apart into different areas, need to reconnect
    % them. Do this by converting nan values "between" regions into
    % slivers that are near boundaries.
    if any(isnan(newStartPoints),'all')
        unboundedVertices = flipud(unboundedRegion.Vertices);
        unboundedPoints = [unboundedVertices; unboundedVertices(1,:)];
        nanIndices = find(isnan(newStartPoints(:,1)));
        pointsBelow = newStartPoints(nanIndices-1,:);
        pointsAbove = newStartPoints(nanIndices+1,:);
        meanPoints = (pointsBelow+pointsAbove)./2;
        
        
        if 1==1
            closestWalls = fcn_INTERNAL_findClosestWall(meanPoints, axisRange);
            Npoints = length(nanIndices);
            if ~all(isequal(closestWalls,ones(Npoints,1)*closestWalls(1)))
                error('Not all wals are equal');
            end
            
            % Find max/min points in region
            minXallRegions = min(minX,min(unboundedVertices(:,1)));
            maxXallRegions = max(maxX,max(unboundedVertices(:,1)));
            minYallRegions = min(minY,min(unboundedVertices(:,2)));
            maxYallRegions = max(maxY,max(unboundedVertices(:,2)));
            if 1==closestWalls
                intersectWallStart = [minXallRegions minY];
                intersectWallEnd   = [maxXallRegions minY];
                offsetDirection = [0 1];
                
            elseif 2==closestWalls
                intersectWallStart = [maxX minYallRegions];
                intersectWallEnd   = [maxX maxYallRegions];
                offsetDirection = [-1 0];
            elseif 3==closestWalls
                intersectWallStart = [maxXallRegions maxY];
                intersectWallEnd   = [minXallRegions maxY];
                offsetDirection = [0 -1];
            elseif 4==closestWalls
                intersectWallStart = [minX maxYallRegions];
                intersectWallEnd   = [minX minYallRegions];
                offsetDirection = [1 0];
            else
                error('unknown wall index encountered');
            end
            % FORMAT:
            %      [distance, location, wall_segment, t, u] = ...
            %         fcn_Path_findSensorHitOnWall(...
            %         wall_start, wall_end,...
            %         sensor_vector_start,sensor_vector_end,...
            %         (flag_search_return_type), (flag_search_range_type), ...
            %         (tolerance), (fig_num))
            [~, locations, ~, ~, u_values] = ...
                fcn_Path_findSensorHitOnWall(...
                unboundedPoints(1:end-1,:), unboundedPoints(2:end,:),...
                intersectWallStart,intersectWallEnd,...
                (1), (0), ...
                ([]), (-1));
            [~,minSensorPercentageIndex] = min(u_values);
            [~,maxSensorPercentageIndex] = max(u_values);

            minPoint = locations(minSensorPercentageIndex,:); 
            maxPoint = locations(maxSensorPercentageIndex,:);

            if 1==closestWalls
                wallStart = [minPoint(1,1) minY];
                wallEnd   = [maxPoint(1,1) minY];                   
            elseif 2==closestWalls
                wallStart = [maxX minPoint(1,2)];
                wallEnd   = [maxX maxPoint(1,2)];
            elseif 3==closestWalls
                wallStart = [maxPoint(1,1) maxY];
                wallEnd   = [minPoint(1,1) maxY];
            elseif 4==closestWalls
                wallStart = [minX maxPoint(1,2)];
                wallEnd   = [minX minPoint(1,2)];
            else
                error('unknown wall index encountered');
            end


            offsetAmount = 0.1;
            offsetWall = [minPoint; maxPoint] + offsetAmount*ones(2,1)*offsetDirection;
            offsetRegionPoints = [offsetWall; wallEnd; wallStart];
            offsetRegion = polyshape(offsetRegionPoints,'KeepCollinearPoints', true,'Simplify', false);

            
            insideRegionMerged = union(insideRegion, offsetRegion);
            insideRegionMergedBounded = intersect(boundingRegion, insideRegionMerged);
            boundedVertices = flipud(insideRegionMergedBounded.Vertices);
            boundedPoints = [boundedVertices; boundedVertices(1,:)];

            if 0==1
                figure(775757)
                clf;
                hold on;
                plot(insideRegionMergedBounded);
                % plot(offsetRegion);
                % plot(insideRegion);
            end

            % Delete any remaining jogs * EXPERIMENTAL *
            if any(isnan(boundedPoints))
                nanInd2 = find(isnan(boundedPoints(:,1)));
                boundedPoints = boundedPoints(1:nanInd2-1, :);
            end
        end
    end % Ends if statement to check if nan values are there

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

