function [polytopes, starts, finishes, resolution_scale] = fcn_util_load_test_map(map_idx, varargin)
% fcn_util_load_test_map
%
% A simple utility for loading a test fixture mat file containing a polytope map
% and its associated starts and finishes.  Many of the maps this can load
% are flood plain maps formed from FEMA floodplains which will be converted
% from LLA to ENU in this utility.  Many maps have one start and finish
% but some have multiple that can be looped over to try different missions.
% This function exists to reduce boilerplate at the beginning of test scripts.
%
%
%
% FORMAT:
% [polytopes, starts, finishes] = fcn_util_load_test_map(map_idx)
%
%
% INPUTS:
%    map_idx: the integer ID of the map the user wishes to load
%
%    (optional inputs)
%
%    add_boundary: set a 1 to append a polytope to the front of the polytope struct array
%        (i.e. in position 1) representing a boundary rectangle around all other polytopes.
%        This is useful for forming hte medial axis/Voronoi boundary graph so the free space
%        surrounding the obstacles does not extend to inifinity.  If this is left
%        blank or set to anyting other than 1, the function defaults to the behavior
%        of not including a boundary, which is more conservative as the boundary polytope
%        overlaps all other polytopes so will be confusing if its inclusion is not expected.
%
%
% OUTPUTS:
%     starts: n-by-2 vector qty. n (x,y) pairs representing n possible starts
%
%     finishes: n-by-2 vector qty. n (x,y) pairs representing n possible finishes
%
%     polytopes - the polytope struct array of obstacles in the map
%
% DEPENDENCIES:
%
% this function will need access to the .mat files in the test fixtures directory
%
% EXAMPLES:
%
% See the scripts: script_test_polytope_canyon* for several examples of this in use
% See the script: script_test_voronoi_planning* for examples of the script in use.
%
% This function was written on 2 May 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu

%
% REVISION HISTORY:
%
% 2024, May by Steve Harnett
% -- first write of function
%
% TO DO:
%
% -- fill in to-do items here.
    %% check input arguments
    if nargin < 1 || nargin > 2
        error('Incorrect number of arguments, this function requires a map ID and optionally a flag for Voronoi planning');
    end
    % if there is no value in varargin...
    if nargin == 1
        % default is to assume no boundary should be provided
        add_boundary = 0;
    end
    % if there is a value in varargin...
    if nargin == 2
        % check what it is
        if varargin{1} == 1
            % set concave flag if it was passed in
            add_boundary = 1;
        elseif varargin{1} == 0
            add_boundary = 0;
        else
            % throw error if it was passed in with an incorrect value
            error('optional argument is the add_boundary flag and can either be 1 or 0')
        end
    end

    resolution_scale = 1; % default to 1 unless over written somewhere
    if map_idx == 1 % generic canyon map
        %% load test fixtures for polytope map rather than creating it here
        % load distribution north of canyon
        load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes1.mat'));
        % this test fixture was made with the following block of code using functions from the MapGen repo
        % tiled_polytopes1 = fcn_MapGen_haltonVoronoiTiling([1,20],[2 1]);
        % % remove the edge polytope that extend past the high and low points
        % % shink the polytopes so that they are no longer tiled
        % des_radius = 0.05; % desired average maximum radius
        % sigma_radius = 0.002; % desired standard deviation in maximum radii
        % min_rad = 0.0001; % minimum possible maximum radius for any obstacle
        % [shrunk_polytopes1,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes1,des_radius,sigma_radius,min_rad);

        % load polytopes representing canyon
        load(strcat(pwd,'\..\Test_Fixtures\canyon_polys_without_exterior.mat'));
        % these polytopes were manually defined

        % load distribution south of canyon
        load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes2.mat'));
        % this test fixture was made with the following block of code using functions from the MapGen repo
        % tiled_polytopes2 = fcn_MapGen_haltonVoronoiTiling([1, 20],[2 1]);
        % % remove the edge polytope that extend past the high and low points
        % % shink the polytopes so that they are no longer tiled
        % [shrunk_polytopes2,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes2,des_radius,sigma_radius,min_rad);
        %% move second polytope field north of canyon
        second_field_vertical_translation = 1.5;
        for i = 1:length(shrunk_polytopes2)
            num_verts_this_poly = length(shrunk_polytopes2(i).yv);
            shrunk_polytopes2(i).yv = shrunk_polytopes2(i).yv + second_field_vertical_translation;
            shrunk_polytopes2(i).vertices = shrunk_polytopes2(i).vertices + [zeros(num_verts_this_poly+1,1) second_field_vertical_translation*ones(num_verts_this_poly+1,1)];
        end

        %% combine two polytope fields and canyon choke point into one field
        polytopes = [shrunk_polytopes1, shrunk_polytopes2, polytopes_manual_canyon];
        %% define start and finish
        start = [0 1.25];
        finish = [2 1.25];
        if add_boundary
            my_warn = sprintf('boundary is not defined for map_idx %i.\n Either define a boundary in fcn_util_load_test_map or set the add_boundary flag to 0.', map_idx);
            warning(my_warn)
        end
    elseif map_idx == 2 % the lower triangular flood plain
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_1.mat'));
        polytopes = flood_plain_1;
        start = [-78.3 40.88];
        % finish = [-78.1 40.9];
        finish = [-78.07 40.82];
        if add_boundary
            my_warn = sprintf('boundary is not defined for map_idx %i.\n Either define a boundary in fcn_util_load_test_map or set the add_boundary flag to 0.', map_idx);
            warning(my_warn)
        end
    elseif map_idx == 3 % the mustafar mining rig map (the comb)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_2.mat'));
        polytopes = flood_plain_2;
        start = [-78.02 40.96];
        % finish = [-77.86 40.93];
        finish = [-77.82 40.97];
        if add_boundary
            my_warn = sprintf('boundary is not defined for map_idx %i.\n Either define a boundary in fcn_util_load_test_map or set the add_boundary flag to 0.', map_idx);
            warning(my_warn)
        end
    elseif map_idx == 4 % also good for edge deletion case (the long river valleys)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_3.mat'));
        polytopes = flood_plain_3;
        start = [-77.49 40.84];
        % finish = [-77.58 40.845];
        finish = [-77.68 40.85];
        if add_boundary
            my_warn = sprintf('boundary is not defined for map_idx %i.\n Either define a boundary in fcn_util_load_test_map or set the add_boundary flag to 0.', map_idx);
            warning(my_warn)
        end
    elseif map_idx == 5 % bridge map, good for random edge deletion case
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_4.mat'));
        polytopes = flood_plain_4;
        start = [-77.68 40.9];
        finish = [-77.5 40.8];
        if add_boundary
            %% make a boundary around the polytope field
            boundary.vertices = [-77.7 40.78; -77.7 40.92; -77.45 40.92; -77.45 40.78];
            boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % close the shape by repeating first vertex
            boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
            boundary.parent_poly_id = nan; % ignore parend ID
            polytopes = [boundary, polytopes]; % put the boundary polytope as the first polytope
        end
    elseif map_idx == 6 % large map, good for dilation case, nearly fully tiled
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_5.mat'));
        polytopes = flood_plain_5;
        start = [-78.01 41.06];
        finish = [-77.75 40.93];

        if add_boundary
            X = max([polytopes.xv]) + 0.02;
            Y = max([polytopes.yv]) + 0.02;
            x = min([polytopes.xv]) - 0.02;
            y = min([polytopes.yv]) - 0.02;
            %% make a boundary around the polytope field
            boundary.vertices = [x y; x Y; X Y; X y];
            boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % close the shape by repeating first vertex
            boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
            polytopes = [boundary, polytopes]; % put the boundary polytope as the first polytope
        end
        resolution_scale = 0.8;
    elseif map_idx == 7 % generic polytope map
        % pull halton set
        rng(1);
        halton_points = haltonset(2);
        points_scrambled = scramble(halton_points,'RR2'); % scramble values

        % pick values from halton set
        Halton_range = [1801 1851];
        low_pt = Halton_range(1,1);
        high_pt = Halton_range(1,2);
        seed_points = points_scrambled(low_pt:high_pt,:);

        % fill polytopes from tiling
        AABB = [0 0 1 1];
        stretch = AABB(3:4);
        tiled_polytopes = fcn_MapGen_generatePolysFromVoronoiAABBWithTiling(seed_points,AABB, stretch);

        % stretch polytopes to cover more area
        new_stretch = [30 40];
        stretched_polytopes = [];
        for poly = 1:length(tiled_polytopes) % pull each cell from the voronoi diagram
            stretched_polytopes(poly).vertices  = tiled_polytopes(poly).vertices.*new_stretch;
        end % Ends for loop for stretch
        stretched_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(stretched_polytopes);

        % shrink polytopes to desired radius
        des_rad = 2.4; sigma_radius = 1.2; min_rad = 0.1;
        [polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(stretched_polytopes,des_rad,sigma_radius,min_rad);

        clear Halton_range
        clear halton_points
        clear points_scrambled

        start = [-2 20];
        finish = [32 20];
        if add_boundary
            %% make a boundary around the polytope field
            boundary.vertices = [-3 -5; -3 45; 33 45; 33 -5];
            boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % close the shape by repeating first vertex
            boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
            polytopes = [boundary, polytopes]; % put the boundary polytope as the first polytope
        end
        resolution_scale = 20; % this map has many fine features and resolution can be 10x the nominal
    elseif map_idx == 8 % Josh's polytope map from 24 April 2024
        load(strcat(pwd,'\..\Test_Fixtures\april_24_example_josh.mat'));
        start = [1 30];
        finish = [100 50];
        if add_boundary
            %% make a boundary around the polytope field
            boundary.vertices = [-5 -5; -5 105; 105 105; 105 -5];
            boundary.vertices = [boundary.vertices; boundary.vertices(1,:)]; % close the shape by repeating first vertex
            boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary); % fill polytope fields
            % boundary.parent_poly_id = nan; % ignore parend ID
            boundary.cost_uncertainty = nan;
            boundary.indv = nan;
            boundary = rmfield(boundary,'mean_radius');
            boundary = rmfield(boundary,'radii');
            polytopes = [boundary, polytopes]; % put the boundary polytope as the first polytope
        end
    end % if conditions for different map test fixtures
    if map_idx <=6 && map_idx >= 2 % for the floodplain maps we have to convert from LLA to km
        %% convert from LLA to QGS84
        datum = 'nad83';
        centre_co_avg_alt = 351.7392;
        lla0 = [40.765144 -77.87615 centre_co_avg_alt];
        % start = INTERNAL_WGSLLA2xyz(start(2),start(1),centre_co_avg_alt);
        start = lla2enu([start(2) start(1) centre_co_avg_alt], lla0, 'flat');
        % start = ll2utm(start(2),start(1),datum);
        start = start(1:2)';
        start = start(1:2)';
        start = start/1000;
        % finish = INTERNAL_WGSLLA2xyz(finish(2),finish(1),centre_co_avg_alt);
        finish = lla2enu([finish(2) finish(1) centre_co_avg_alt], lla0, 'flat');
        % finish = ll2utm(finish(2),finish(1),datum);
        finish = finish(1:2)';
        finish = finish(1:2)';
        finish = finish/1000;
        new_polytopes = [];
        for i = 1:length(polytopes)
            poly = polytopes(i);
            lats = poly.vertices(:,2);
            longs = poly.vertices(:,1);
            alts = centre_co_avg_alt*ones(size(lats));
            wgs_verts = [];
            for j = 1:length(lats)
                % xyz = INTERNAL_WGSLLA2xyz(lats(j),longs(j),alts(j));
                xyz = lla2enu([lats(j),longs(j),alts(j)], lla0, 'flat');
                xyz = xyz/1000;
                wgs_verts(j,:) = [xyz(1),xyz(2)];
            end
            new_polytopes(i).vertices = wgs_verts;
        end
        polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(new_polytopes);
    end
    if map_idx == 6 % for map 6 we can loop over many start goal pairs
        starts = [start; 1015,-4704; 1000,-4722; 1017 -4721; 995, -4714; 1025, -4704; 1030, -4708];
        finishes = [finish; 1010, -4722 ; 1027, -4704; 1007 -4707; 1030, -4712; 1005, -4722; 995 -4722];
        starts = [-10 15; 2 15; 15 15; -15 25; -5 36; 12 36];
        finishes = [10 37; 2 36; -15 36; 18 26; -5 15; 12 15];
    elseif map_idx == 3
        starts = [1002, -4715.9];
        finishes = [1017, -4719];
    else % if we only have one start goal pair
        starts = start;
        finishes = finish;
    end
end

function xyz = INTERNAL_WGSLLA2xyz(wlat, wlon, walt)
    %Function xyz = wgslla2xyz(lat, lon, alt) returns the
    %equivalent WGS84 XYZ coordinates (in meters) for a
    %given geodetic latitude "lat" (degrees), longitude "lon"
    %(degrees), and altitude above the WGS84 ellipsoid
    %in meters.  Note: N latitude is positive, S latitude
    %is negative, E longitude is positive, W longitude is
    %negative.
    %
    %Ref: Decker, B. L., World Geodetic System 1984,
    %Defense Mapping Agency Aerospace Center.

    A_EARTH = 6378137;
    flattening = 1/298.257223563;
    NAV_E2 = (2-flattening)*flattening; % also e^2
    deg2rad = pi/180;

    slat = sin(wlat*deg2rad);
    clat = cos(wlat*deg2rad);
    r_n = A_EARTH/sqrt(1 - NAV_E2*slat*slat);
    xyz = [ (r_n + walt)*clat*cos(wlon*deg2rad);
            (r_n + walt)*clat*sin(wlon*deg2rad);
            (r_n*(1 - NAV_E2) + walt)*slat ];

    if ((wlat < -90.0) | (wlat > +90.0) | (wlon < -180.0) | (wlon > +360.0))
        error('WGS lat or WGS lon out of range');
    end
end
