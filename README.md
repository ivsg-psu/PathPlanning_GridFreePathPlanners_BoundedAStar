# PathPlanning_GridFreePathPlanners_BoundedAStar
The repo for grid-free path planning using Bounded A-star algorithm, first started by Seth Tau (2019+) and currently maintained by Dr. Brennan (sbrennan@psu.edu) and Steve Harnett (sjharnett@psu.edu).  This repo contains the bounded A* grid free path planner, variations of this path planner, and supporting infrastructure such as visibility graph (vgraph) creation tools.


## General Architecture

- `fcn_algorithm_bound_Astar.m` – core planning algorithm
- `fcn_algorithm_straight_planner.m` – alternative planner that only plans a straight path without diverting around obstacles
- `fcn_algorithm_setup_bound_Astar_for_tiled_polytopes.m` – wraps and calls planning algorithms.  This is generally the entry point for calling the planner stack.  One purpose of this code is to find the search space boundary which will be explained later.
- `fcn_visibility_clear_and_blocked_points.m` – creates the visibility graph for the polytope field from the given location
- `fcn_visibility_self_blocked_pts.m` – determines points blocked by the current obstacle (i.e. points that would be visible if the current obstacle was transparent)

## Key Features
### The Elliptical Boundary
Bounded A* is an algorithm which improves efficiency over A* by limiting the search space to paths navigable in less distance than the path that the Bug2 algorithm would plan.  This is visualized as a bounding ellipse below. For more on this, see § 3.2 of Seth Tau’s thesis: THE EFFECTS OF PATH-PLANNING UNCERTAINTY ONINTELLIGENT VEHICLE PERFORMANCE. This is the main planner that all subsequent planners are variations of.


![image](https://user-images.githubusercontent.com/67085752/200880560-4a5fa92d-a4f5-484c-951a-f77c3dec27f9.png)


The functions `fcn_bounding_ellipse_min_perimeter_path` and `fcn_bounding_ellipse_polytope_bounding_box` perform this calculation.



### Starting in Obstacles
For the purposes of replanning the path after probing an obstacle (i.e. navigating into it to update the map's estimate of this obstacle's cost), the planner can be started in obstacles.  Note that the planner may backtrack out of high-cost obstcles to globally minimize path cost, rather than continuing straight through the obstacle.


![image](https://user-images.githubusercontent.com/67085752/200881599-922f0d10-b0fe-4d7f-84d1-e5e6a03c8197.png)


![image](https://user-images.githubusercontent.com/67085752/200881638-3bcec94f-3e0a-45c1-9543-50afc77c5c3d.png)



## Planner Modes
The argument `planner_mode` for `fcn_algorithm_bound_Astar` and `fcn_algorithm_setup_bound_Astar_for_tiled_polytopes` can be used to modify the planner modality to one of the following:
- `legacy` only goes around obstacles


![image](https://user-images.githubusercontent.com/67085752/200879889-53d1bb8b-75f0-44f1-8981-a7213db9ef76.png)


- `through at vertices` allows the planner to go through or around each obstacle but only entering or exiting at vertices.  This is largely influenced by the `cost` field on the polytope struct which indicates the difficulty of traversing some distance through the obstacle as a portion greater than the difficulty of traversing free space (i.e. a polytope with a cost of 0.2 will be 20% more difficult to traverse so travelling 1 meter through this polytope is as costly as travelling 1.2 meters through free space).


![image](https://user-images.githubusercontent.com/67085752/200877936-cb150243-3f18-49a2-8755-52b2169d9a32.png)
![image](https://user-images.githubusercontent.com/67085752/200877955-7ea4e136-b3a1-48e0-ae65-a8cd21ac690a.png)


- `straight through` the planner only goes straight from the start to the goal, calculating the cost


![image](https://user-images.githubusercontent.com/67085752/200876748-f987bbf4-f197-4a76-879c-d3a60fede909.png)


- `through or around` allows the planner to go through all obstacles or around all


![image](https://user-images.githubusercontent.com/67085752/200880072-0354dd56-1f0a-415a-afc0-22bdd5975bb2.png)
![image](https://user-images.githubusercontent.com/67085752/200880027-b5f986d4-0c2b-436c-836d-2c4f9f303293.png)

## Cost Functions
There are numerous cost functions that can be used when planning.  To see how some of these behave, there is an interactive GUI here for tuning cost function terms and viewing the resulting paths: `Functions\app_gui_cost_function_tuning_front_end.mlapp`.
An outline of some of the cost functions and tools available for incentivizing different planner behaviors is below.  Note this is not an apples-to-apples list of different objective functions that can be interchanged with each other as some of these are fundamentally different planning paradigms. For example, the length cost matrix in row 1 and the visibility cost matrix in row 5 are both nxn matrices and can be linearly combined but the Pagan Path Planner in row 3 is a route post processer that operates in a fundamentally different manner.  The path angle penalty in row 4 is a cost placed on every pair of edges rather than a cost placed on each individual edge, like length cost, so while it can be used at the same time as length cost to reduce path length and path turns, they are not 1:1 cost functions that can be directly compared or interchanged.
| Problem   |      Goal      |  Cost Function/tool | Location |
|----------|---------|------|------|
| in a spatial or timespace map, long paths can be slow to navigate and costly in terms of fuel | penalize path length | path length cost | `Functions\fcn_algorithm_generate_cost_graph.m` |
| in a timespace map, high mission duration is undesirable | penalize expected path navigation time | path time cost | `Functions\fcn_algorithm_generate_cost_graph.m` |
| in a spatial map, speed at which the mission is executed should be increaed | modify a point to point path to navigate it at maximum speed | Pagan Path Planner for transforming linear path segments to high-speed curves | `PathPlanning_PathKinematics_PathKinematicsClass` |
| turning can slow down vehicles and be energy expensive in the case of skid-steer vehicles | limit turns in the path | add a cost term for change in angle between path segments | alpha version of this functionality is here: `Functions\fcn_algorithm_bound_Astar.m:305` |
| obstacles that are not mapped can block edges that were thought to be available at the time of planning thus making rerouting difficult |  want to methodically lower the cost of visiting nodes with more departing edges | visibility/rechability based cost function | Example of this is in use here: `Functions\script_test_polytope_canyon_replan.m:148` |
| mapped obstacles may have scale or position errors thus narrow spaces near obstacles may be cut off, making rereouring difficult | want to incentivize being in more open space | corridor width cost function will incentivize routing down wider corridors | cost matrix can be generated here: `Functions\fcn_algorithm_generate_dilation_robustness_matrix.m` this is shown in use here: `Functions\script_test_polytope_canyon_replan_with_dialation.m` |
| obstacle fields may not be binary obstacle and free space maps, there may be zones with higher cost | need to represent cost scaling via polytopes that are traversable | cost function scaled by polytope cost | alpha version of this functionality is here: `Functions\fcn_algorithm_bound_Astar.m:278` multiple layers of overlapping polytopes can also be flattened to have their costs combined: `PathPlanning_MapTools_MapGenClassLibrary\Functions\fcn_MapGen_flattenPolytopeMap.m` |
