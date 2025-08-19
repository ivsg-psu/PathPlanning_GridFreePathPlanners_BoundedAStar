% script_demo_BoundedAStar.m
% This is a script that shows the capabilities of the "BoundedAStar" class of
% functions via demonstrations.

% Revision history:
% 2025_07_03 - Sean Brennan / Kaelea Hayes
% - First write of the function, using the "MapGen" library as starter
%
% 2025_07_07 - Sean Brennan and Kaelea Hayes
% - Starting integration of MapGen into codes
% - Updated MapGen library to support integration
%
% 2025_07_26 - Sean Brennan
% - Updated DebugTools to DebugTools_v2025_07_15
%
% 2025_07_28 - Sean Brennan
% - Updated MapGen to MapGenClass_v2025_07_28b
% - Added Path library, PathClass_v2025_07_06
% - DEPRECATED fcn_polytope_generation_halton_voronoi_tiling
%    % * now: fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset',
%    %   % [low_pt high_pt])
% - DEPRECATED fcn_polytope_editing_remove_edge_polytopes
%    % * now: fcn_MapGen_polytopesDeleteByAABB
% - DEPRECATED fcn_polytope_editing_shrink_to_average_max_radius_with_variance
%    % * now: fcn_MapGen_polytopesShrinkToRadius
% - DEPRECATED fcn_polytope_editing_set_all_costs
%    % * now: fcn_MapGen_polytopesSetCosts
% - Updated script_demo_fcn_BoundedAStar_Astar with MapGen functions
% - Working through all other scripts
% - DEPRECATED fcn_general_calculation_euclidean_point_to_point_distance
%   % * now hard-coded as sum((diff).^2,2).^0.5;
% - DEPRECATED fcn_Map_Generation_map_name_to_map
%   % * now fcn_MapGen_generatePolysFromName
% - DEPRECATED fcn_polytope_calculation_centroid_and_area
%   % * now fcn_MapGen_polytopeCentroidAndArea
% - Removed folder: Example_Map_Generation_Code
%
% 2025_07_28 - K. Hayes
% - DEPRECATED: fcn_algorithm_Astar
%    % * now: fcn_BoundedAStar_Astar
% - DEPRECATED: fcn_algorithm_Astar3d
%    % * now: fcn_BoundedAStar_Astar3d
% - DEPRECATED: fcn_algorithm_bound_Astar
%    % * now: fcn_BoundedAStar_AstarBounded
% - DEPRECATED: fcn_algorithm_setup_bound_Astar_for_tiled_polytopes
%    % * now: fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes
% - DEPRECATED: fcn_bounding_ellipse_min_perimeter_path
%    % * now: fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath
% - DEPRECATED: fcn_visibility_clear_and_blocked_points
%    % * now: fcn_Visibility_clearAndBlockedPoints
% - Added test script starters for new function versions
%
% 2025_07_29 - Sean Brennan (in progress)
% - Fixed bug with generation of polys
%   % * AABB and expansion were switched in argument ordering
% - DEPRECATED: fcn_BoundedAStar_plotPolytopes
% - DEPRECATED: fcn_plot_polytopes
%   % * now fcn_MapGen_plotPolytopes
% - DEPRECATED: fcn_BoundedAStar_polytopeEditingShrinkEvenly
%   % * now trying to get this to be fixed in script_test_fcn_Visibility_clearAndBlockedPoints
%   % * moved this function to MapGen
%   % * now called fcn_MapGen_polytopesShrinkEvenly
% - updated MapGen to MapGenClass_v2025_07_29, to address issues above
% - In fcn_BoundedAStar_AstarBounded
%   % * added plotting as part of standard debug output
%   % * pulled plotting out of test script
%   % * converted if-then (break) into if/then/else for clear code
%   %   % pass-through
% - In fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes
%   % * added plotting as part of standard debug output
%   % * pulled plotting out of test script
%   % * fixed incorrect argument list in docstrings
% (still need to finish cleanup of fcn_BoundedAStar_plotPolytopes and
% fcn_plot_polytopes function calls)
%
% 2025_07_30 - Sean Brennan (in progress)
% - Added MATLABs TSP example
% - Added missing folder to MapGen install to support GridMapGen
% - Working on fcn_BoundedAStar_reachabilityWithInputs (in progress)
%
% 2025_07_31 - K. Hayes
% - Formatting and input checking changes in remaining visibility functions
% - DEPRECATED: fcn_visibility_clear_and_blocked_points_global
%   % * now fcn_Visibility_clearAndBlockedPointsGlobal
% - DEPRECATED: fcn_BoundedAStar_AstarOLD
% - DEPRECATED: fcn_BoundedAStar_Astar3dOLD
% - DEPRECATED: fcn_visibility_graph_3d_global
%   % * now fcn_Visibility_3dGraphAddPoints
% - DEPRECATED: fcn_visibility_self_blocked_pts
%   % * now fcn_Visibility_selfBlockedPoints
% - DEPRECATED: fcn_visibility_graph_3d_add_points
%   % * now fcn_Visibility_3dGraphAddPoints
% - DEPRECATED: fcn_visibility_line_polytope_intersections
%   % * now fcn_Visibility_linePolytopeIntersections
% - DEPRECATED: fcn_visibility_graph_add_obstacle
%   % * now fcn_Visibility_addObstacle
% - DEPRECATED: fcn_visibility_graph_remove_obstacle
%   % * now fcn_Visibility_removeObstacle
% - In fcn_BoundedAStar_Astar
%   % * added debug plotting options
%   % * added polytopes to fcn inputs to allow plotting
% - In script_demo_fcn_BoundedAStar_Astar
%   % * moved plotting capabilities into function debug options
%
% 2025_07_31 - Sean Brennan (in progress)
% - Working on fcn_BoundedAStar_reachabilityWithInputs (in progress)
% - fcn_BoundedAStar_plotWindField
%   % * added h_plot output, so that the handle to plots is available
%
% 2025_08_01 - Sean Brennan
% - finished 1st version of reachabilityWithInputs.
%   % * Saved as fcn_BoundedAStar_reachabilityWithInputs
%   % * Timing speed is 10 ms per cycle, with vast majority dealing with 
%   %   % polytope expansion and merging
%   % * For expected sim length (500 steps), this is 5 seconds per sim
%   % * Assuming 10k sims, this comes out to 750 seconds per optimization,
%   %   % which is about 10 hours. Need this to be less than 1 hour if
%   %   % possible, so need 10x speed improvement minimum.
%
% 2025_08_01 - K. Hayes (in progress)
% - updating test scripts for visibility fcns for new MapGen
%   compatibility. changes made to
%   % * script_test_fcn_Visibility_clearAndBlockedPointsGlobal
%   % * script_test_fcn_Visibility_clearAndBlockedPoints
%   % * script_test_fcn_Visibility_linePolytopesIntersections
%   % * script_test_fcn_Visibility_addObstacle
% - adding debug plotting for visibility fcns. changes made to 
%   % * fcn_Visibility_clearAndBlockedPointsGlobal
%   % * fcn_Visibility_clearAndBlockedPoints
%   % * fcn_Visibility_linePolytopeIntersections
%   % * fcn_Visibility_addObstacle
%
% 2025_08_02 by S. Brennan
% Significant rewrite of fcn_BoundedAStar_reachabilityWithInputs and as
% well a "slow" form: fcn_BoundedAStar_reachabilityWithInputs_SLOW
% - Vectorized for loops for sampling speed 
% - added script_time_fcn_BoundedAStar_reachabilityWithInputs for timing 
%   tests
% - added specialized set bounds to merge search space across input points
% - modified the set methods to use edge-projection expansions for speeds
%   % * This is about 40 times faster than using set methods (!)
% - optimized code where easily changed to get fastest speeds. Single loop
%   % calls appear to take about 0.2 milliseconds with fastest settings 
% - added rule in main loop to "clean up points". Namely: the projection is
%   % only valid if the segment length created by the adjacent unit vectors
%   % has length greater than 0.5 (typically, this value is greater than
%   % .90 in real-world data)
% - updated PathClass library to PathClass_v2025_08_02
%   % * fixes bugs seen in fixing "jogs" in reachabilityWithInputs codes
% - added internal function, fcn_INTERNAL_sparsifyPoints, 
%   % * makes sure that points on bounding perimeter do not get too dense
% - Added: fcn_BoundedAStar_expandReachabilityWithWind
% - Added: script_test_fcn_BoundedAStar_expandReachabilityWithWind
%
% 2025_08_03 by S. Brennan
% - Added fcn_BoundedAStar_expandReachabilityWithWind and test script
%   % * This performs repeated calls to the expansion set, so one can 
%   %   % "see" where the feasible final set is at, trajectory, and
%   %   % difficulty
% - updated PathClass library to PathClass_v2025_08_03
%   % * gives option to set jog threshold
% 
% 2025_08_04 by s. Brennan, sbrennan@psu.edu
% - In fcn_BoundedAStar_expandReachabilityWithWind
%   % * Added cellArrayOfWindExitConditions
%   % * Added exitCondition output information
%   % * Added cellArrayOfExitInfo output information
% - in script_test_fcn_BoundedAStar_expandReachabilityWithWind
%   % * added exit condition outputs and tests
%   % * added cellArrayOfExitInfo output info and tests
%   % * added cellArrayOfWindExitConditions inputs and tests in TEST section
%
% 2025_08_05 by K. Hayes
% - updating remaining visibility fcns. see cleaning spreadsheet for
%   details
% - updating test scripts for visibility fcns for new MapGen
%   compatibility. changes made to
%   % * script_test_fcn_Visibility_addObstacle
% - added starter test scripts:
%   % * script_test_fcn_Visibility_removeObstacle
%   % * script_test_fcn_Visibility_3dGraphGlobal
%   % * script_test_fcn_Visibility_3dGraphAddPoints
%   % * script_test_fcn_Visibility_selfBlockedPoints
% - adding debug plotting for visibility fcns. changes made to 
%   % * fcn_Visibility_addObstacle
%   % * fcn_Visibility_removeObstacle
%   % * fcn_Visibility_3dGraphGlobal
%   % * fcn_Visibility_3dGraphAddPoints
%   % * fcn_Visibility_selfBlockedPoints
% - continued cleaning of other BoundedAStar fcns
%   % * fcn_BoundedAStar_polytopesGenerateAllPtsTable
% - created test scripts for other BoundedAStar fcns
%   % * script_test_fcn_BoundedAStar_polytopesGenerateAllPtsTable
%   % * script_test_fcn_BoundedAStar_straightPlanner
%   % * script_test_fcn_BoundedAStar_polytopesNearLine
% - DEPRECATED: fcn_polytopes_generate_all_pts_table
%   % * now fcn_BoundedAStar_polytopesGenerateAllPtsTable
% - DEPRECATED: fcn_algorithm_straight_planner
%   % * now fcn_BoundedAStar_straightPlanner
% - DEPRECATED: fcn_polytope_calculation_polytopes_near_the_line
%   % * now fcn_BoundedAStar_polytopesNearLine
%
% 2025_08_05 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * first write of function using fcn_BoundedAStar_solveTSPwithWind
%   %   % as a starter
%   % * got TSP solution intialization working with greedy method, which
%   %   % produces a quick upper bound estimate on sim time
% - In script_test_fcn_BoundedAStar_solveTSPwithWind:
%   % * first write of script 
%   % * using script_test_fcn_BoundedAStar_solveTSPwithWind as starter
%
% 2025_08_06 by K. Hayes
% - continued cleaning BoundedAStar fcns
% - DEPRECATED: fcn_algorithm_generate_cost_graph
%   % * now fcn_BoundedAStar_generateCostGraph 
% - added test scripts:
%   % * script_test_fcn_BoundedAStar_generateCostGraph
%
% 2025_08_06 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * TSP working with Djkstra's method. Minor bug found
%
% 2025_08_07 by K. Hayes
% - DEPRECATED: script_test_fcn_algorithm_greedy_planner
%   % * now script_test_fcn_BoundedAStar_greedyPlanner
% - DEPRECATED: fcn_algorithm_greedy_planner
%   % * now fcn_BoundedAStar_greedyPlanner
% - DEPRECATED: script_test_fcn_find_edge_weights
%   % * now script_test_fcn_BoundedAStar_findEdgeWeights
% - DEPRECATED: fcn_find_edge_weights
%   % * now fcn_BoundedAStar_findEdgeWeights
%
% 2025_08_07 by S. Brennan
% - in fcn_BoundedAStar_solveTSPwithWind
%   % * TSP working with Djkstra's method. Prior bug fixed. 
%   % * Added plotting of results
%
% 2025_08_08 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_reachabilityWithInputs
%   % * Added test scripts to check new output: boundingPolytopeVertices
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * added boundingPolytopeVertices to outputs
%
% 2025_08_11 by K. Hayes
% - DEPRECATED: script_test_fcn_interpolate_route_spatially
%   % * now script_test_fcn_BoundedAStar_interpolateRouteSpatially
% - DEPRECATED: fcn_interpolate_route_spatially
%   % * now fcn_BoundedAStar_interpolateRouteSpatially
% - DEPRECATED: script_test_fcn_convert_polytope_struct_to_deduped_points
%   % * now script_test_fcn_BoundedAStar_convertPolytopetoDedupedPoints
% - DEPRECATED: fcn_convert_polytope_struct_to_deduped_points
%   % * now fcn_BoundedAStar_convertPolytopetoDedupedPoints
%
% 2025_08_11 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_reachabilityWithInputs
%   % * Added test scripts to check new output: boundingPolytopeVertices
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * added boundingPolytopeVertices to outputs
%
% 2025_08_12 by K. Hayes
% - DEPRECATED: fcn_algorithm_create_phantom_goal
%   % * now fcn_BoundedAStar_createPhantomGoal
% - Added script_test_fcn_createPhantomGoal
% - DEPRECATED: fcn_check_reachability
%   % * now fcn_BoundedAStar_checkReachability
% - Added script_test_fcn_checkReachability
%
% 2025_08_13 by K. Hayes
% - DEPRECATED: fcn_interpolate_polytopes_in_time
%   % * now fcn_BoundedAStar_interpolatePolytopesInTime
% - Added script_test_fcn_interpolatePolytopesInTime
% - DEPRECATED: fcn_interpolate_route_in_time
%   % * now fcn_BoundedAStar_interpolateRouteInTime
% - Added script_test_fcn_interpolateRouteInTime
%
% 2025_08_14 by K. Hayes
% - DEPRECATED: fcn_make_facets_from_verts
%	% * now fcn_BoundedAStar_makeFacetsFromVerts
% - added script_test_fcn_BoundedAStar_makeFacetsFromVerts
% - DEPRECATED: fcn_util_load_test_map
% 	% * now fcn_BoundedAStar_loadTestMap
% - added script_test_fcn_BoundedAStar_loadTestMap
% - DEPRECATED: fcn_calculation_points_in_polytopes
%	% * now fcn_BoundedAStar_polytopesPointsInPolytopes
% - added script_test_fcn_BoundedAStar_polytopesPointsInPolytopes
%
% 2025_08_15 by K. Hayes
% - DEPRECATED: fcn_make_timespace_polyhedra_from_polygons
% 	% * now fcn_BoundedAStar_makeTimespacePolyhedrafromPolygons
% - added script_test_fcn_BoundedAStar_makeTimespacePolyhedrafromPolygons
% - DEPRECATED: fcn_make_triangular_surfels_from_facets
% 	% * now fcn_BoundedAStar_makeTriangularSurfelsFromFacets
% - added script_test_fcn_BoundedAStar_makeTriangularSurfelsFromFacets
% - DEPRECATED: fcn_general_calculation_point_to_line_distances_squared
% 	% * now fcn_BoundedAStar_calculatePointToLineDistSquared
% - added script_test_fcn_BoundedAStar_calculatePointToLineDistSquared
%
% 2024_08_15 to 2024_08_16 by S. Brennan
% - In fcn_BoundedAStar_reachabilityWithInputs
%   % * moved meshgrid cacluation into debugging, as it is not used for
%   %   % main outputs - only for debugging
%   % * improved the header description for clarity
%   % * fixed bug where the expansion due to wind disturbance was
%   %   % after the state expansion, instead of prior
%   % * updated function to output intermediate calculations
%   % * moved set simplification steps to PRIOR to states
%
% 2025_08_16 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_reachabilityWithInputs
%   % * Added test scripts to check new output: cellArrayOfIntermediateCalculations
%
% 2025_08_17 to 2025_08_18 by S. Brennan, sbrennan@psu.edu
% - Added fcn_BoundedAStar_pathCalculationBackToStart
%   % * Given a destination, finds the path and control
%   %   % inputs back to startPoint, such that, if one
%   %   % starts at the startPoint, applies the control inputs, one arrives
%   %   % at the destination.
% - In fcn_BoundedAStar_pathCalculation
%   % * first write of function
%   % * using fcn_BoundedAStar_matrixEnvelopeExpansion as a starter
% - In script_test_fcn_BoundedAStar_pathCalculation
%   % * first write of script 
%   % * using script_test_fcn_BoundedAStar_reachabilityWithInputs as 
%   %   % starter
%
% 2025_08_18 by K. Hayes
% - added script_test_fcn_BoundedAStar_Astar
% - in fcn_BoundedAStar_greedyPlanner
%   % * fixed debug plotting
% - in script_test_fcn_BoundedAStar_AStarBounded
%   % * fixed formatting and debug plotting calls
%
% 2025_08_19 by K. Hayes
% - updated all_pts generation to use
%   fcn_BoundedAStar_polytopesGenerateAllPtsTable
% - DEPRECATED: fcn_general_calculation_points_on_lines
%   % * now fcn_BoundedAStar_calculatePointsOnLines
% - added script_test_fcn_BoundedAStar_calculatePointsOnLines


% TO-DO:
% 2025_07_03 - Sean Brennan
% -- Where to start?! All functions need to be checked, verified, etc.
% 2025_07_28 - Sean Brennan
% -- Remove all "addpath" commands nested inside scripts and functions
% -- DEPRECATE: fcn_polytope_editing_shrink_evenly
% -- need to de-spaghetti fcn_BoundedAStar_AStarBounded
% 2025_08_01 - K. Hayes
% -- add fcn to plot visibility graph
% -- fast modes for visibility test scripts
% 2025_08_18 - K. Hayes
% -- add 3d formatted all_pts capabilities to
%    fcn_BoundedAStar_polytopesGenerateAllPtsTable


clear library_name library_folders library_url

ith_library = 1;
library_name{ith_library}    = 'DebugTools_v2025_07_15';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/archive/refs/tags/DebugTools_v2025_07_15.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'MapGenClass_v2025_07_29';
library_folders{ith_library} = {'Functions','testFixtures','GridMapGen'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary/archive/refs/tags/MapGenClass_v2025_07_29.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'PathClass_v2025_08_03';
library_folders{ith_library} = {'Functions', 'Data'};                                
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary/archive/refs/tags/PathClass_v2025_08_03.zip';

% ith_library = ith_library+1;
% library_name{ith_library}    = 'GPSClass_v2023_04_21';
% library_folders{ith_library} = {''};
% library_url{ith_library}     = 'https://github.com/ivsg-psu/FieldDataCollection_GPSRelatedCodes_GPSClass/archive/refs/tags/GPSClass_v2023_04_21.zip';
% 
% ith_library = ith_library+1;
% library_name{ith_library}    = 'GetUserInputPath_v2023_02_01';
% library_folders{ith_library} = {''};
% library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_PathTools_GetUserInputPath/blob/main/Releases/GetUserInputPath_v2023_02_01.zip?raw=true';
% 
% ith_library = ith_library+1;
% library_name{ith_library}    = 'AlignCoordinates_2023_03_29';
% library_folders{ith_library} = {'Functions'};
% library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_AlignCoordinates/blob/main/Releases/AlignCoordinates_2023_03_29.zip?raw=true';


%% Clear paths and folders, if needed
if 1==0
    clear flag_BoundAStar_Folders_Initialized
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;

end

%% Do we need to set up the work space?
if ~exist('flag_BoundAStar_Folders_Initialized','var')
    this_project_folders = {'Functions','Functions_CostCalculation','Test_Fixtures','Utilities_gif','Utilities_TriangleRayIntersection','Utilities_streamcolor'}; % {'Functions','Data'};
    fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders);  
    flag_BoundAStar_Folders_Initialized = 1;
end

%% Set environment flags for input checking in HSOV library
% These are values to set if we want to check inputs or do debugging
setenv('MATLABFLAG_BOUNDASTAR_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_BOUNDASTAR_FLAG_DO_DEBUG','0');

%% Set environment flags for input checking in Geometry library
% setenv('MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS','0');
% setenv('MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG','0');

% %% Set environment flags that define the ENU origin
% % This sets the "center" of the ENU coordinate system for all plotting
% % functions
% 
% % Location for Test Track base station
% setenv('MATLABFLAG_PLOTROAD_REFERENCE_LATITUDE','40.86368573');
% setenv('MATLABFLAG_PLOTROAD_REFERENCE_LONGITUDE','-77.83592832');
% setenv('MATLABFLAG_PLOTROAD_REFERENCE_ALTITUDE','344.189');
% 
% 
% %% Set environment flags for plotting
% % These are values to set if we are forcing image alignment via Lat and Lon
% % shifting, when doing geoplot. This is added because the geoplot images
% % are very, very slightly off at the test track, which is confusing when
% % plotting data
% setenv('MATLABFLAG_PLOTROAD_ALIGNMATLABLLAPLOTTINGIMAGES_LAT','-0.0000008');
% setenv('MATLABFLAG_PLOTROAD_ALIGNMATLABLLAPLOTTINGIMAGES_LON','0.0000054');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%    _____      _   _   _                _____ _             _           _ 
%   / ____|    | | | | (_)              / ____| |           | |         | |
%  | |  __  ___| |_| |_ _ _ __   __ _  | (___ | |_ __ _ _ __| |_ ___  __| |
%  | | |_ |/ _ \ __| __| | '_ \ / _` |  \___ \| __/ _` | '__| __/ _ \/ _` |
%  | |__| |  __/ |_| |_| | | | | (_| |  ____) | || (_| | |  | ||  __/ (_| |
%   \_____|\___|\__|\__|_|_| |_|\__, | |_____/ \__\__,_|_|   \__\___|\__,_|
%                                __/ |                                     
%                               |___/                                      
% See http://patorjk.com/software/taag/#p=display&f=Big&t=Getting%20Started
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Welcome to the Bounded A-Star library!')

%% 2d demo scripts
% script_demo_fcn_BoundedAStar_Astar.m
% a basic test of a 2D path planning scenario 
% Generate an obstacle field and calculate an A-star path through it
%
% NOTE: should delete script_test_fcn_algorithm_Astar AFTER all codes are
% tested and confirmed to work.

script_demo_fcn_BoundedAStar_Astar

%% 3d demo scripts that have been checked
% Tested on 2025_07_07 - works and produces 5 plots that show avoidance of
% a moving wall. This script is an update of script_test_3d.m written by S.
% Harnett. 
% 
% NOTE: should delete script_test_3d AFTER all codes are tested
% and confirmed to work.

script_demo_fcn_BoundedAStar_Astar3d;



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

%% function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
% Clear out the variables
clear global flag* FLAG*
clear flag*
clear path

% Clear out any path directories under Utilities
path_dirs = regexp(path,'[;]','split');
utilities_dir = fullfile(pwd,filesep,'Utilities');
for ith_dir = 1:length(path_dirs)
    utility_flag = strfind(path_dirs{ith_dir},utilities_dir);
    if ~isempty(utility_flag)
        rmpath(path_dirs{ith_dir});
    end
end

% Delete the Utilities folder, to be extra clean!
if  exist(utilities_dir,'dir')
    [status,message,message_ID] = rmdir(utilities_dir,'s');
    if 0==status
        error('Unable remove directory: %s \nReason message: %s \nand message_ID: %s\n',utilities_dir, message,message_ID);
    end
end

end % Ends fcn_INTERNAL_clearUtilitiesFromPathAndFolders

%% fcn_INTERNAL_initializeUtilities
function  fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders)
% Reset all flags for installs to empty
clear global FLAG*

fprintf(1,'Installing utilities necessary for code ...\n');

% Dependencies and Setup of the Code
% This code depends on several other libraries of codes that contain
% commonly used functions. We check to see if these libraries are installed
% into our "Utilities" folder, and if not, we install them and then set a
% flag to not install them again.

% Set up libraries
for ith_library = 1:length(library_name)
    dependency_name = library_name{ith_library};
    dependency_subfolders = library_folders{ith_library};
    dependency_url = library_url{ith_library};

    fprintf(1,'\tAdding library: %s ...',dependency_name);
    fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url);
    clear dependency_name dependency_subfolders dependency_url
    fprintf(1,'Done.\n');
end

% Set dependencies for this project specifically
fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders);

disp('Done setting up libraries, adding each to MATLAB path, and adding current repo folders to path.');
end % Ends fcn_INTERNAL_initializeUtilities


function fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url, varargin)
%% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES - MATLAB package installer from URL
%
% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES installs code packages that are
% specified by a URL pointing to a zip file into a default local subfolder,
% "Utilities", under the root folder. It also adds either the package
% subfoder or any specified sub-subfolders to the MATLAB path.
%
% If the Utilities folder does not exist, it is created.
% 
% If the specified code package folder and all subfolders already exist,
% the package is not installed. Otherwise, the folders are created as
% needed, and the package is installed.
% 
% If one does not wish to put these codes in different directories, the
% function can be easily modified with strings specifying the
% desired install location.
% 
% For path creation, if the "DebugTools" package is being installed, the
% code installs the package, then shifts temporarily into the package to
% complete the path definitions for MATLAB. If the DebugTools is not
% already installed, an error is thrown as these tools are needed for the
% path creation.
% 
% Finally, the code sets a global flag to indicate that the folders are
% initialized so that, in this session, if the code is called again the
% folders will not be installed. This global flag can be overwritten by an
% optional flag input.
%
% FORMAT:
%
%      fcn_DebugTools_installDependencies(...
%           dependency_name, ...
%           dependency_subfolders, ...
%           dependency_url)
%
% INPUTS:
%
%      dependency_name: the name given to the subfolder in the Utilities
%      directory for the package install
%
%      dependency_subfolders: in addition to the package subfoder, a list
%      of any specified sub-subfolders to the MATLAB path. Leave blank to
%      add only the package subfolder to the path. See the example below.
%
%      dependency_url: the URL pointing to the code package.
%
%      (OPTIONAL INPUTS)
%      flag_force_creation: if any value other than zero, forces the
%      install to occur even if the global flag is set.
%
% OUTPUTS:
%
%      (none)
%
% DEPENDENCIES:
%
%      This code will automatically get dependent files from the internet,
%      but of course this requires an internet connection. If the
%      DebugTools are being installed, it does not require any other
%      functions. But for other packages, it uses the following from the
%      DebugTools library: fcn_DebugTools_addSubdirectoriesToPath
%
% EXAMPLES:
%
% % Define the name of subfolder to be created in "Utilities" subfolder
% dependency_name = 'DebugTools_v2023_01_18';
%
% % Define sub-subfolders that are in the code package that also need to be
% % added to the MATLAB path after install; the package install subfolder
% % is NOT added to path. OR: Leave empty ({}) to only add 
% % the subfolder path without any sub-subfolder path additions. 
% dependency_subfolders = {'Functions','Data'};
%
% % Define a universal resource locator (URL) pointing to the zip file to
% % install. For example, here is the zip file location to the Debugtools
% % package on GitHub:
% dependency_url = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/blob/main/Releases/DebugTools_v2023_01_18.zip?raw=true';
%
% % Call the function to do the install
% fcn_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url)
%
% This function was written on 2023_01_23 by S. Brennan
% Questions or comments? sbrennan@psu.edu

% Revision history:
% 2023_01_23:
% -- wrote the code originally
% 2023_04_20:
% -- improved error handling
% -- fixes nested installs automatically

% TO DO
% -- Add input argument checking

flag_do_debug = 0; % Flag to show the results for debugging
flag_do_plots = 0; % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end


%% check input arguments
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

if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(3,4);
end

%% Set the global variable - need this for input checking
% Create a variable name for our flag. Stylistically, global variables are
% usually all caps.
flag_varname = upper(cat(2,'flag_',dependency_name,'_Folders_Initialized'));

% Make the variable global
eval(sprintf('global %s',flag_varname));

if nargin==4
    if varargin{1}
        eval(sprintf('clear global %s',flag_varname));
    end
end

%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if ~exist(flag_varname,'var') || isempty(eval(flag_varname))
    % Save the root directory, so we can get back to it after some of the
    % operations below. We use the Print Working Directory command (pwd) to
    % do this. Note: this command is from Unix/Linux world, but is so
    % useful that MATLAB made their own!
    root_directory_name = pwd;

    % Does the directory "Utilities" exist?
    utilities_folder_name = fullfile(root_directory_name,'Utilities');
    if ~exist(utilities_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(root_directory_name,'Utilities');

        % Did it work?
        if ~success_flag
            error('Unable to make the Utilities directory. Reason: %s with message ID: %s\n',error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The Utilities directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',error_message, message_ID);
        end

    end

    % Does the directory for the dependency folder exist?
    dependency_folder_name = fullfile(root_directory_name,'Utilities',dependency_name);
    if ~exist(dependency_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(utilities_folder_name,dependency_name);

        % Did it work?
        if ~success_flag
            error('Unable to make the dependency directory: %s. Reason: %s with message ID: %s\n',dependency_name, error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The %s directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',dependency_name, error_message, message_ID);
        end

    end

    % Do the subfolders exist?
    flag_allFoldersThere = 1;
    if isempty(dependency_subfolders{1})
        flag_allFoldersThere = 0;
    else
        for ith_folder = 1:length(dependency_subfolders)
            subfolder_name = dependency_subfolders{ith_folder};
            
            % Create the entire path
            subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);
            
            % Check if the folder and file exists that is typically created when
            % unzipping.
            if ~exist(subfunction_folder,'dir')
                flag_allFoldersThere = 0;
            end
        end
    end

    % Do we need to unzip the files?
    if flag_allFoldersThere==0
        % Files do not exist yet - try unzipping them.
        save_file_name = tempname(root_directory_name);
        zip_file_name = websave(save_file_name,dependency_url);
        % CANT GET THIS TO WORK --> unzip(zip_file_url, debugTools_folder_name);

        % Is the file there?
        if ~exist(zip_file_name,'file')
            error(['The zip file: %s for dependency: %s did not download correctly.\n' ...
                'This is usually because permissions are restricted on ' ...
                'the current directory. Check the code install ' ...
                '(see README.md) and try again.\n'],zip_file_name, dependency_name);
        end

        % Try unzipping
        unzip(zip_file_name, dependency_folder_name);

        % Did this work? If so, directory should not be empty
        directory_contents = dir(dependency_folder_name);
        if isempty(directory_contents)
            error(['The necessary dependency: %s has an error in install ' ...
                'where the zip file downloaded correctly, ' ...
                'but the unzip operation did not put any content ' ...
                'into the correct folder. ' ...
                'This suggests a bad zip file or permissions error ' ...
                'on the local computer.\n'],dependency_name);
        end

        % Check if is a nested install (for example, installing a folder
        % "Toolsets" under a folder called "Toolsets"). This can be found
        % if there's a folder whose name contains the dependency_name
        flag_is_nested_install = 0;
        for ith_entry = 1:length(directory_contents)
            if contains(directory_contents(ith_entry).name,dependency_name)
                if directory_contents(ith_entry).isdir
                    flag_is_nested_install = 1;
                    install_directory_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name);
                    install_files_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name,'*.*');
                    install_location_to = fullfile(directory_contents(ith_entry).folder);
                end
            end
        end

        if flag_is_nested_install
            [status,message,message_ID] = movefile(install_files_from,install_location_to);
            if 0==status
                error(['Unable to move files from directory: %s\n ' ...
                    'To: %s \n' ...
                    'Reason message: %s\n' ...
                    'And message_ID: %s\n'],install_files_from,install_location_to, message,message_ID);
            end
            [status,message,message_ID] = rmdir(install_directory_from);
            if 0==status
                error(['Unable remove directory: %s \n' ...
                    'Reason message: %s \n' ...
                    'And message_ID: %s\n'],install_directory_from,message,message_ID);
            end
        end

        % Make sure the subfolders were created
        flag_allFoldersThere = 1;
        if ~isempty(dependency_subfolders{1})
            for ith_folder = 1:length(dependency_subfolders)
                subfolder_name = dependency_subfolders{ith_folder};
                
                % Create the entire path
                subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);
                
                % Check if the folder and file exists that is typically created when
                % unzipping.
                if ~exist(subfunction_folder,'dir')
                    flag_allFoldersThere = 0;
                end
            end
        end
         % If any are not there, then throw an error
        if flag_allFoldersThere==0
            error(['The necessary dependency: %s has an error in install, ' ...
                'or error performing an unzip operation. The subfolders ' ...
                'requested by the code were not found after the unzip ' ...
                'operation. This suggests a bad zip file, or a permissions ' ...
                'error on the local computer, or that folders are ' ...
                'specified that are not present on the remote code ' ...
                'repository.\n'],dependency_name);
        else
            % Clean up the zip file
            delete(zip_file_name);
        end

    end


    % For path creation, if the "DebugTools" package is being installed, the
    % code installs the package, then shifts temporarily into the package to
    % complete the path definitions for MATLAB. If the DebugTools is not
    % already installed, an error is thrown as these tools are needed for the
    % path creation.
    %
    % In other words: DebugTools is a special case because folders not
    % added yet, and we use DebugTools for adding the other directories
    if strcmp(dependency_name(1:10),'DebugTools')
        debugTools_function_folder = fullfile(root_directory_name, 'Utilities', dependency_name,'Functions');

        % Move into the folder, run the function, and move back
        cd(debugTools_function_folder);
        fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        cd(root_directory_name);
    else
        try
            fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        catch
            error(['Package installer requires DebugTools package to be ' ...
                'installed first. Please install that before ' ...
                'installing this package']);
        end
    end


    % Finally, the code sets a global flag to indicate that the folders are
    % initialized.  Check this using a command "exist", which takes a
    % character string (the name inside the '' marks, and a type string -
    % in this case 'var') and checks if a variable ('var') exists in matlab
    % that has the same name as the string. The ~ in front of exist says to
    % do the opposite. So the following command basically means: if the
    % variable named 'flag_CodeX_Folders_Initialized' does NOT exist in the
    % workspace, run the code in the if statement. If we look at the bottom
    % of the if statement, we fill in that variable. That way, the next
    % time the code is run - assuming the if statement ran to the end -
    % this section of code will NOT be run twice.

    eval(sprintf('%s = 1;',flag_varname));
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

    % Nothing to do!



end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends function fcn_DebugTools_installDependencies

