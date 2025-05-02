![image](https://github.com/user-attachments/assets/013672eb-d5e9-44b3-ae52-158e4eda80fe)

## Introduction
This documents features and demonstrations for planning along the Voronoi diagram edges (the Medial Axis) as a graph rather than planning along the visibility graph.  This creates a graph with one edge per corridor.  This is particularly helpful for generating alternate paths that use different corridors to avoid impassable corridors.  This behavior is demonstrated in test scripts.  For documentation, please see [Documentation/medial_axis_planning.pptx](https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_BoundedAStar/blob/main/Documentation/medial_axis_planning.pptx)

### Scripts Demonstrating Applications
The main scripts demonstrating this behavior are:
- `script_test_voronoi_planning` - the base example case of planning along a Voronoi diagram's edges (the medial axis graph)
- `script_test_voronoi_planning_alt_paths` - an example like the above but generating a series of alternate routes from the global start to the global finish, each with wider corridors
- `script_test_voronoi_planning_alt_poaths_from_node` - an example like `script_test_voronoi_planning_alt_paths` but alternate paths are generated from arbitrary nodes along the initial path rather than from the global start
- `script_test_voronoi_planning_alt_paths_local` - an example like  `script_test_voronoi_planning_alt_paths` but alternate paths are generated from all nodes along the initial path rather than from the global start.  This script can be run in different modes to block only the next segment in the initial path, all segments in the initial path, or all segments in all previously planned alternate path (as in coverage path planning)
- `script_test_voronoi_planning_hill` - this is an example like `script_test_voronoi_planning` but rather than using a corridor width incentive, elevation data is incorporated into the graph and ascending hills is avoided
### Implemented Features
The key feature described here is medial axis graph (Voronoi diagram edge) planning.  This process is broken into several functions.
A full flow chart of the tool chain described in the following functions is in: [Documentation/medial_axis_planning.pptx](https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_BoundedAStar/blob/main/Documentation/medial_axis_planning.pptx).  The chart is reproduced here:
![image](https://github.com/user-attachments/assets/f751f176-d333-4f09-b2f3-0d02a0b95ceb)
A brief summary of the added functions follows.
- `fcn_MedialAxis_makeAdjacencyMatrixAndTriangleChains` - forms the adjacency matrix (indicating which nodes are connected) and triangle chains data structure (describing the curving path segments making up the edges between nodes).  These two data structures essentially make up the medial axis graph.
- `fcn_MedialAxis_pruneGraph` - wraps two functions (1) `fcn_MedialAxis_removeDeadEnds` and (2) `fcn_MedialAxis_removeThroughNodes` which are used to remove edges that depart a node but lead nowhere and nodes that connect two edges but functionally offer no choice.  This wrapper thus trims the graph to its smallest size such that only nodes that afford multiple choices to the planner are left.
- `fcn_MedialAxis_addCostsToTriangleChains` - takes the structure describing the path segments making up the edges of the medial axis graph and adds length and estimated width to each
- `fcn_MedialAxis_addPointToAdjacencyMatrixAndTriangleChains` - adds arbitrary points to the medial axis graph.  Useful for adding starts and finishes.
- `fcn_MedialAxis_makeCostGraphAndAllPoints` - makes the cgraph and all_pts table of the form used by A*
- `fcn_MedialAxis_generateAltRoutesFromNode` - generates series of paths from an arbitrary node to the finish, each departing the arbitrary node from a different edge until all departing edges have been used.
- `fcn_MedialAxis_processRoute` - turns a series of nodes into a reference path of x-y positions.  This is necessary as the nodes in the medial axis graph may not be connected by straight lines but rather are connected by the curving path segments forming the Voronoi diagram boundaries (medial axis edges)
- `fcn_MedialAxis_plannerWrapper` - wraps medial axis graph creation, pruning, edge cost evaluation, start and finish point creation, and planning into a single neat interface so medial axis planning can be exposed more easily to users (e.g., MPC local path following)
- `fcn_MedialAxis_replanWrapper` - if the above function has already been called, this function can be used to replan in the existing, pruned medial axis graph.  The edge containing the replanning point is removed, as it is assumed to be blocked, and a new path is found, if possible, without it.
