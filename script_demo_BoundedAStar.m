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
% 2025_08_18 by S. Brennan
% - In fcn_BoundedAStar_Astar
%   % * fixed unrecognized variables bug
% - In fcn_BoundedAStar_pathCalculationBackToStart
%   % first write of function
%   % * using fcn_BoundedAStar_matrixEnvelopeExpansion as a starter
%
% 2025_08_19 by K. Hayes
% - updated all_pts generation to use
%   fcn_BoundedAStar_polytopesGenerateAllPtsTable
% - DEPRECATED: fcn_general_calculation_points_on_lines
%   % * now fcn_BoundedAStar_calculatePointsOnLines
% - added script_test_fcn_BoundedAStar_calculatePointsOnLines
%
% 2025_08_19 by S. Brennan
% - In fcn_BoundedAStar_pathCalculationBackToStart
%   % * completed first end-to-end working version
% - In script_test_fcn_BoundedAStar_pathCalculationBackToStart
%   % * Added working example of backward path calculation
%
% 2025_09_04 by K. Hayes
% - DEPRECATED: fcn_bounding_ellipse_polytope_bounding_box
%   % * now fcn_BoundedAStar_calculateBoundingEllipsePolytopeBoundingBox
% - added script_test_fcn_BoundedAStar_calculateBoundingEllipsePolytopeBoundingBox
%
% 2025_10_03 by K. Hayes
% - In script_test_fcn_Visibility_linePolytopeIntersections
%   % * fixed bug causing assertion failure in DEMO case 1
% - In script_test_fcn_Visibility_clearAndBlockedPointsGlobal
%   % * fixed bug causing Npoly assertion failures in DEMO cases
%   % * fixed bug with missing variables in DEMO case 3
%   % * fixed bug with missing variables in TEST case 1
% - In script_test_fcn_Visibility_removeObstacle
%   % * fixed bug with missing variables in DEMO case 1
% - In script_test_fcn_Visibility_selfBlockedPoints
%   % * fixed bug with missing variables in DEMO case 1
% - In script_test_fcn_Visibility_3dGraphAddPoints
%   % * fixed bug causing assertion failure in DEMO case 1
% - In script_test_fcn_Visibility_3dGraphGlobal
%   % * fixed bug causing assertion failure in DEMO case 1
% - Changed
%   script_test_fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath to
%   script_test_fcn_BoundedAStar_calculateBoundingEllipseMinPerimPa to
%   avoid overlength error
% - In script_test_fcn_BoundedAStar_findEdgeWeights
%   % * fixed bug causing assertion failure in DEMO case 1
% - In script_test_fcn_BoundedAStar_generateCostGraph
%   % * fixed bug causing assertion failure in DEMO case 1
% - In script_test_fcn_BoundedAStar_polytopesNearLine
%   % * fixed bug causing assertion failure in DEMO case 1
%
% 2025_10_06 - S. Brennan
% -- removed addpath calls in many scripts
% -- removed calls to fcn_util_load_test_map, replaced with fcn_BoundedAStar_loadTestMap
% -- added explicit deprecation message to fcn_util_load_test_map
% -- fixed calls to fcn_MapGen_polytopesStatistics, replaced with fcn_MapGen_statsPolytopes
% -- fixed calls to fcn_polytope_editing_shrink_evenly, replace with fcn_MapGen_polytopesShrinkEvenly
%
% 2025_10_07 - S. Brennan
% -- trying to get this working:
%    % *script_test_polytope_canyon_corridor_width_incentive_weighting
% -- removed calls to fcn_visibility_clear_and_blocked_points_global,
%    % replaced with fcn_Visibility_clearAndBlockedPointsGlobal
% -- removed calls to fcn_polytopes_generate_all_pts_table,
%    % replaced with fcn_BoundedAStar_polytopesGenerateAllPtsTable
% -- removed calls to fcn_check_reachability,
%    % replaced with fcn_BoundedAStar_checkReachability
% -- removed calls to fcn_algorithm_generate_cost_graph,
%    % replaced with fcn_BoundedAStar_generateCostGraph
% -- removed calls to fcn_algorithm_generate_dilation_robustness_matrix,
%    % replaced with fcn_BoundedAStar_generateDilationRobustnessMatrix
% -- removed calls to fcn_MapGen_fillPolytopeFieldsFromVertices,
%    % replaced with fcn_MapGen_polytopesFillFieldsFromVertices
% 
% 2025_10_07 - S. Brennan
% -- updated DebugTools_v2025_09_26b
% -- updated MapGenClass_v2025_10_07
% -- added function, plotVgraph, to plot visibility graph
% -- In script_fcn_BoundedAStar_generateDilationRobustnessMatrix
%    % * Merged plotting to create "sequence" for MECC 2025 presentation
%
% 2025_10_10 - K. Hayes
% -- In fcn_Visibility_clearAndBlockedPoints
%    % * fixed bug causing deletion of vgraph edges between adjacent
%    non-obstacle vertices
%
% 2025_10_17 - S. Brennan
% -- In fcn_BoundedAStar_generateDilationRobustnessMatrix
%    % * added full plotting capability including plotting options that
%    %   % allow saving images to file
%    % * fixed bug where edge widths were including stand-alone points
%    % * fixed bug where infinite edge widths were counted as zero corridor
%    %   % widths
%    % * added GeometryClass_v2025_10_20 to simplify 
%    %   % fcn_BoundedAStar_generateDilationRobustnessMatrix
%
% 2025_10_22 - K. Hayes
% -- In script_test_fcn_BoundedAStar_AStarBounded
%    % * fixed bug with start and finish points being incorrectly passed
%    into fcn
%    % * fixed bug causing length assertion failure in demo case 1
% -- In fcn_BoundedAStar_AStarBounded
%    % * replaced reference to
%    fcn_BoundedAStar_calculateBoundingEllipsePolytopeBoundingBox, which is
%    now called fcn_BoundedAStar_calculateBoundingEllipsePolytope
%    % * fixed bug where cur_pt, finish inputs were passed to
%    calculateBoundingEllipsePolytope with the wrong dimensions
% -- In fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes
%    % * added all_pts generation using
%    fcn_BoundedAStar_polytopesGenerateAllPtsTable instead of manually
% -- In script_test_fcn_BoundedAStar_calculatePointsOnLines
%    % * fixed bug causing assertion failures for all demo cases
%
% 2025_10_28 - S. Brennan
% -- In this demo script:
%    % * Removed to-do items related to visibility calculations, these
%    %   % copied now into newly created VisibilityGraph repo
%
% 2025_10_30 - K. Hayes
% -- In Functions/DEPRECATED
%    % * added deprecation warning messages to all remaining functions
%
% 2025_11_02 by S. Brennan
% - DEPRECATED: fcn_BoundedAStar_loadTestMap
%   % * now fcn_MapGen_loadTestMap 
%   %   % NOTE: this function also deprecated many of data loading files
%   % * function kept in ToMapGen subfolder until move and edits complete
%   % * rewrote function and test script for clarity (prior version was
%   %   % spaghetti coded)
%   % * removed calls to fcn_BoundedAStar_loadTestMap, replaced with
%   %   % fcn_MapGen_loadTestMap in following functions:
%   %   % -- script_animate_dilation_replan
%   %   % -- script_animate_dilation_replan
%   %   % -- script_test_alternate_path_generation
%   %   % -- script_test_polytope_canyon_corridor_width_incentive_weighting
%   %   % -- script_test_polytope_canyon_replan
%   %   % -- script_test_polytope_canyon_replan_with_dilation
%   %   % -- script_test_voronoi_planning
%   %   % -- script_test_voronoi_planning_alt_paths
%   %   % -- script_test_voronoi_planning_alt_paths_from_node
%   %   % -- script_test_voronoi_planning_alt_paths_local
%   %   % -- script_test_voronoi_planning_hill
% - DEPRECATED: fcn_BoundedAStar_generateDilationRobustnessMatrix
%   % * now fcn_Visibility_generateDilationRobustnessMatrix 
%   % * function kept in ToVisibility subfolder until move complete
%   % * replaced fcn_BoundedAStar_generateDilationRobustnessMatrix, 
%   %   % with fcn_Visibility_generateDilationRobustnessMatrix
%   %   % in following functions:
%   %   % -- script_animate_dilation_replan
%   %   % -- script_test_alternate_path_generation
%   %   % -- script_test_polytope_canyon_corridor_width_incentive_weighting
%   %   % -- script_test_polytope_canyon_replan_with_dilation
%   %   % -- script_test_voronoi_compare_vgraph_corridor_width
% - DEPRECATED: fcn_BoundedAStar_polytopesGenerateAllPtsTable
%   % * now fcn_Visibility_polytopesGenerateAllPtsTable 
%   % * function kept in ToVisibility subfolder until move complete
%   % * changed function inputs to have start/finish as optional inputs
%   % * changed function outputs 
%   %   % -- to include start/finish in point listing, 
%   %   % -- label start/end points as 1 and 2 (rather than both 1) 
%   % * replaced fcn_BoundedAStar_polytopesGenerateAllPtsTable, 
%   %   % with fcn_Visibility_polytopesGenerateAllPtsTable
%   %   % in following functions:
%   %   % -- fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes (untested)
%   %   % -- script_test_fcn_BoundedAStar_Astar (works) <-- Astar code
%   %   % -- script_Path_Planning_testing (untested) <-- core BoundedAStar
%   %   % -- script_animate_dilation_replan (half tested) <-- MECC
%   %   % -- script_test_polytope_canyon_replan_with_dilation (untested)
%   %   % -- script_test_alternate_path_generation (untested)
%   %   % -- script_test_concave_visibility_and_planning (untested)
%   %   % -- script_test_fcn_BoundedAStar_AstarBounded (untested)
%   %   % -- script_test_fcn_BoundedAStar_calculateBoundingEllipsePolytope (untested)
%   %   % -- script_test_fcn_BoundedAStar_convertPolytopetoDedupedPoints.m (untested)
%   %   % -- script_test_fcn_BoundedAStar_countObstaclesInPath (untested)
%   %   % -- script_test_fcn_find_edge_weights (untested)
%   %   % -- script_test_fcn_BoundedAStar_generateCostGraph (untested)
%   %   % -- script_test_fcn_BoundedAStar_greedyPlanner (untested)
%   %   % -- script_test_fcn_BoundedAStar_straightPlanner (untested)
%   %   % -- script_test_fcn_Visibility_removeObstacle (untested)
%   %   % -- script_test_fcn_Visibility_selfBlockedPoints (untested)
%   %   % -- script_test_3d_polytope_canyon (untested)
%   %   % -- script_test_polytope_canyon_replan (untested)
%   %   % -- script_test_fcn_Visibility_clearAndBlockedPointsGlobal (tested)
% - DEPRECATED: fcn_BoundedAStar_clearAndBlockedPoints
%   % * now fcn_Visibility_clearAndBlockedPoints
%   % * function kept in ToVisibility subfolder until move complete
% - DEPRECATED: fcn_BoundedAStar_clearAndBlockedPointsGlobal
%   % * now fcn_Visibility_clearAndBlockedPointsGlobal 
%   % * function kept in ToVisibility subfolder until move complete
% - DEPRECATED: fcn_BoundedAStar_convertPolytopetoDedupedPoints
%   % * now fcn_Visibility_convertPolytopetoDedupedPoints 
%   % * function kept in ToVisibility subfolder until move complete
%   % * NOTE: no idea what this function does!
% - script_test_visibility_graph_modification
%   % * moved to ToVisibility subfolder until move is complete
%   % * renamed to script_demo_visibilityGraphAddRemoveObstacles
% - fcn_Visibility_removeObstacle
%   % * moved to ToVisibility subfolder until move is complete
%   % * updated internal variable naming
% - fcn_Visibility_addObstacle
%   % * moved to ToVisibility subfolder until move is complete
%   % * updated internal variable naming
%
% 2025_11_04 - K. Hayes
% - In this demo script:
%   % * updated DebugTools dependency to Debug_Tools_v2025_11_04c
%
% 2025_11_04 - S. Brennan
% - In this demo script:
%   % * updated DebugTools dependency to Debug_Tools_v2025_11_04c
%
% 2025_11_06 - S. Brennan
% - Moved MapGen files/data out and into MapGen
% - Merged this with repo
% - Updated MapGenClass_v2025_11_06
% - Updated DebugTools_v2025_11_06
%
% 2025_11_13 - S. Brennan
% - set up auto-loading of dependencies using new DebugTools features
% - added new VGraph dependency, moved VGraph functions out of this repo
% - updated script_test_all_functions
% - replaced '_BOUND'+'ASTAR_' with '_BOUNDEDASTAR_' in global variables
% - changed Documentation folder to Documents, for consistency with
%   % template
% -- fixed file naming errors within repo, AStar is preferred over Astar
%    % * from fcn_BoundedAStar_AstarBounded
%    % * to fcn_BoundedAStar_AStarBounded
%    % * from fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes
%    % * to fcn_BoundedAStar_AStarBoundedSetupForTiledPolytopes
% (in fcn_BoundedAStar_polytopePointsInPolytopes)
% - Refactored code to change from structure inputs to vector inputs
% - Refactored code to vectorize outputs
% - Deleted extra figure call in input section
% - Fixed bad function formatting at end
% - Changed variable inputs to names that are more clear or that fit style
%   % standards
%   % from A (structure) to startXY (1x2 vector)
%   % from B (structure) to finishXY (1x2 vector)
%   % fig_num to figNum
%   % throw_error to flagThrowError
%   % edge_check to flagEdgeCheck
%   % Apoly to startPolys
%   % Bpoly to finishPolys
%   % err to flagsAtLeastOnePointIsInPoly
% - Improved plotting
% - Tested funtion - test script works now


% - updated header flags for clearing path, to do fast checking without
%   % skipping
% - updated deprecation warnings so that they only show once:
%   % * fcn_Path_findIntersectionsBetweenTraversals
%   % * fcn_Path_convertPathToTraversalStructure
%   % * fcn_Path_fillOffsetTraversalsAboutTraversal
%   % * fcn_Path_findOrthogonalTraversalVectorsAtStations
%   % * fcn_Path_removePinchPointInTraversal
%   % * fcn_Path_snapPointOntoNearestTraversal
%   % * fcn_Path_findTraversalStationSegment
%   % * fcn_Path_calcSingleTraversalStandardDeviation
%   % * fcn_Path_findOrthogonalHitFromTraversalToTraversal
%   % * fcn_Path_fillRandomTraversalsAboutTraversal
%   % * fcn_Path_convertTraversalXYtoSy
%   % * fcn_Path_findAverageTraversalViaOrthoProjection
%   % * fcn_Path_findClosestPointsToTraversal
%   % * fcn_Path_findTraversalWithMostData
%   % * fcn_Path_newTraversalByStationResampling
%   % * fcn_Path_findCenterlineVoteFromTraversalToTraversal
% - renamed all global variables to _PATH_ instead of
%   cat(2,'_PATH','CLASS_')
% - fixed one instance where _GEOMETRY_ global variable used instead of 
%   % _PATH_
% - moved demo script to its own folder
% - removed deprecated functions and scripts for clean-up into subfolder
% - fixed minor errors in testing scripts. All pass currently.
% (new release)

% TO-DO:
% 2025_07_28 - Sean Brennan
% -- need to de-spaghetti fcn_BoundedAStar_AStarBounded
% 2025_08_18 - K. Hayes
% -- add 3d formatted all_pts capabilities to
%    fcn_BoundedAStar_polytopesGenerateAllPtsTable
% 2025_11_01 - Sean Brennan
% -- Need to move following into MECC2025 scripts:
%    % * script_animate_dilation_replan (core function)
%    % * script_test_alternate_path_generation
%    % * script_test_polytope_canyon_corridor_width_incentive_weighting
%    % * script_test_polytope_canyon_replan
%    % * script_test_polytope_canyon_replan_with_dilation
% - Can deprecate fcn_BoundedAStar_calculatePointsOnLines with Path library?
% - Need to rewrite fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes

%% Fixes bad global variables
% fcn_DebugTools_replaceStringInDirectory(pwd, cat(2,'_BOUND','ASTAR_'), '_BOUNDEDASTAR_', ('BoundedAStar'), (-1));
% fcn_DebugTools_replaceStringInDirectory(fullfile(pwd,'Functions'), cat(2,'_BOUND','ASTAR_'), '_BOUNDEDASTAR_', ('BoundedAStar'), (-1));
% fcn_DebugTools_replaceStringInDirectory(fullfile(pwd,'Functions'), '_MAPGEN_', '_BOUNDEDASTAR_', ('BoundedAStar'), (-1));
% fcn_DebugTools_replaceStringInDirectory(fullfile(pwd,'Functions'), '_Visibility_', '_VGraph_', ('BoundedAStar'), (-1));


%% Make sure we are running out of root directory
st = dbstack; 
thisFile = which(st(1).file);
[filepath,name,ext] = fileparts(thisFile);
cd(filepath);

%% Clear paths and folders, if needed
if 1==1
    clear flag_BoundAStar_Folders_Initialized
end
if 1==0
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;
end

%% Install dependencies
% Define a universal resource locator (URL) pointing to the repos of
% dependencies to install. Note that DebugTools is always installed
% automatically, first, even if not listed:
clear dependencyURLs dependencySubfolders
ith_repo = 0;

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','testFixtures','GridMapGen'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_VGraph';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','Data'};

%% Do we need to set up the work space?
if ~exist('flag_BoundAStar_Folders_Initialized','var')

    % Clear prior global variable flags
    clear global FLAG_*

    % Navigate to the Installer directory
    currentFolder = pwd;
    cd('Installer');
    % Create a function handle
    func_handle = @fcn_DebugTools_autoInstallRepos;

    % Return to the original directory
    cd(currentFolder);

    % Call the function to do the install
    func_handle(dependencyURLs, dependencySubfolders, (0), (-1));

    % Add this function's folders to the path
    this_project_folders = {...
        'Functions',...
        'Data',...
        'Functions_CostCalculation',...
        'Test_Fixtures',...
        'Utilities_gif',...
        'Utilities_TriangleRayIntersection',...
        'Utilities_streamcolor'};
    fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders)

    flag_BoundAStar_Folders_Initialized = 1;
end

%% Set environment flags for input checking in BoundedAStar library
% These are values to set if we want to check inputs or do debugging
setenv('MATLABFLAG_BOUNDEDASTAR_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_BOUNDEDASTAR_FLAG_DO_DEBUG','0');

%% Set environment flags that define the ENU origin
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

disp('Welcome to the BoundedAStar library!')

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
