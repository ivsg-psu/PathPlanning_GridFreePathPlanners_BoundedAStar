# 3D (XYT) Grid Free Path Planning

The repo for functions to plan through 3D (XYT or XYZ) maps representing either dynamic 2D obstacles or static 3D obstacles.
<!-- PROJECT LOGO -->
<br>
<p align="center">
  <h2 align="center">
    3D (XYT) Grid Free Path Planning
  </h2>
  <pre align="center">
        <img src=".\Images\2d_to_3d.png">
        <!-- figcaption>Fig.1 - The typical progression of map generation.</figcaption -->
    </font>
  </pre>
</p>

<p align="left">
    This repo hosts tools to plan paths useful for studying path planning, autonomy, etc. The purpose of this code is to process 3D maps of obstacle fields and plan paths thorugh them.  The obstacles are stored as polytopes in vertex representation.
    <br>
    <a href="https://github.com/ivsg-psu/ivsg_master/wiki/Path-Planning#map-tools"><strong>Explore the docs</strong></a>
    <a href="https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_BoundedAStar">View Demo</a>
    <a href="https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_BoundedAStar/issues">Report Bug</a>
    <a href="https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_BoundedAStar/issues">Request Feature</a>
</p>

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
    <ol>
        <li>
            <a href="#about-the-project">About the Project</a>
        </li>
        <li>
        <a href="#getting-started">Getting Started</a>
        <ul>
                <li><a href="#installation">Installation</a></li>
        </ul>
        </li>
        <li><a href="structure">Repo Structure</a>
            <ul>
                <li><a href="#directories">Top-Level Directories</li>
                <li><a href="#dependencies">Dependencies</li>
            </ul>
        </li>
        <li><a href="#functions">Functions</li>
            <ul>
                <li><a href="#main_algorithms">Main Algorithms</li>
                <li><a href="#infrastructure">Infrastructure</li>
            </ul>
        <li><a href="#usage">Usage</a></li>
            <ul>
                <li><a href="#examples">Examples</li>
            </ul>
        <li><a href="#license">License</a></li>
        <li><a href="#contact">Contact</a></li>
    </ol>
</details>

***
<!-- ABOUT THE PROJECT -->
## About The Project

This repo hosts tools to process and plan through 3D maps for studying path planning, autonomy, etc. These maps can represent obstacles as polytopes, stored as their vertices.  These obstacles represent regions to be avoided.  The 3D nature of this project can be used to represent three spatial dimensions (XYZ) in the case of an air or undersea vehicle or these can be two spatial dimensions and a temporal dimension (XYT) to represent a flat 2D plane with obstacles that change position over time in the case of a surface or ground vehicle in a dynamic environment.
<pre align="center">
<img src=".\Images\timespace_example.png">
<figcaption>An example of a 2D dynamic obstacle in timespace.</figcaption>
</pre>

* Inputs:
  * polytopes in XYZ or XYT space
  * cost function
  * simulation time step
  * speed limit (if XYT space)
* Outputs
  * series of points representing a route
  * animations and plots to aid in visualizing this result

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

***
<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Installation

1. Make sure to run MATLAB 2023b or higher. Why? The "combinations" function is used to vectorize intersection checking.

2. Install dependencies from MATLAB File Exchange and add to path:

    2.1 The <a href="https://www.mathworks.com/matlabcentral/fileexchange/63239-gif">gif</a> function for creating animations.

    2.2 The <a href="https://www.mathworks.com/matlabcentral/fileexchange/33073-triangle-ray-intersection">triangle ray intersection</a> function for performing the Moller-Trumbore intersection checking method, necessary for forming the visibliity graph.

2. Clone the repo

   ```sh
   git clone https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_BoundedAStar
   ```

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

<!-- STRUCTURE OF THE REPO -->
***

## Structure

### Directories

The following are the top level directories within the repository:
<ul>
    <li>/Documents folder: Descriptions of the functionality and usage of the various MATLAB functions and scripts in the repository.</li>
    <li>/Functions folder: The majority of the code functionalities are implemented in this directory. All functions as well as test scripts are provided.</li>
    <!--TODO uncomment this when dependency management is added to this repo: <li>/Utilities folder: Dependencies that are utilized but not implemented in this repository are placed in the Utilities directory. These can be single files but are most often folders containing other cloned repositories.</li>-->
  <li>/testFixtures folder: This includes .mat workspace files that some test scripts use as inputs so that they can opperate on a known workspace without having to run numerous preprocessing steps and other functions.  This saves time and prevents test flaking.</li>
</ul>

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

### Dependencies

* <a href="https://www.mathworks.com/matlabcentral/fileexchange/63239-gif">gif</a> function for creating animations.

* The <a href="https://www.mathworks.com/matlabcentral/fileexchange/33073-triangle-ray-intersection">triangle ray intersection</a> function for performing the Moller-Trumbore intersection checking method, necessary for forming the visibliity graph.

    <!--TODO uncomment this when dependency management is added to this repo: Each should be installed in a folder called "Utilities" under the root folder, namely `./Utilities/DebugTools/`. If you wish to put this code in different directories, the main call stack in `script_demo_MapGenLibrary` can be easily modified with strings specifying the different location, but the user will have to make these edits directly. -->

    <!--TODO uncomment this when dependency management is added to this repo: For ease of getting started, the zip files of the directories used - without the .git repo information, to keep them small - are included in this repo.-->

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

<!-- FUNCTION DEFINITIONS -->
## Functions


<pre align="center">
<img src=".\Images\call_flow.png">
<figcaption>This is the basic flow of data and function calls to get from a 2D polytope map and a maximum translation distance for each polytope, to a route through 3D timespace.  Not all parts of this call flow are necessary (e.g. obstacles could be manually defined, interpolation of polytopes in time could be skipped, etc.) but the general flow remains unchanged.</figcaption>
</pre>

### Main Algorithms

The following are the core algorithms that make 3D planning possible:

`fcn_algorithm_Astar3d.m` : The main path planner.  A simple implementation of the A* graph search algorithm.  It is easy to change the cost functions or heuristic costs inside this function.

`fcn_algorithm_Astar.m` : A 2D implementation of the above.

`fcn_visibility_graph_3d_global` : Forms the 3D visibility graph, the edges of which connect nodes that are connected by straight, collision-free path segments.  This leverages the Moller-Trumbore algorithm to check potential graph edges for intersections with the 3D obstacles.  The visibility graph can also take in a speed limit, which in the case of XYT rather than XYZ is useful for pruning edges that would go backwards in time or traverse too much distance in too short of a time.

<pre align="center">
<img src=".\Images\intersection_check.png">
<figcaption>A ray drawn between a start and goal in the presence of a triangulated obstacle.  The Moller-Trumbore algorithm found a collision along this line segment with the plane segment.</figcaption>
</pre>

<pre align="center">
<img src=".\Images\vgraph_cartoon.png">
<figcaption>A cartoon of a visibility graph formed around a parallelogram obstacle in timespace green edges are collision free, the red edge has a collision with the blue facet.</figcaption>
</pre>

<pre align="center">
<img src=".\Images\speed_limit.png">
<figcaption>An example of a speed limit being enforced on edges leaving a single node.  The black edges have been pruned while the green ones are still valid.  The speed limit is shown as a rainbow cone that the edges must fall within to avoid pruning.</figcaption>
</pre>

`fcn_check_reachability.m` : Raises the visibility graph to the powers 1 through N where N is the number of nodes.  The visibility graph raised to the Nth power describes nodes reachable in N steps.  Thus this function can tell whether or not the goal is reachable from the start, and the minimum number of steps to perform this mission.

### Infrastructure

The following are more basic functions that support the above functions:

`fcn_make_timespace_polyhedra_from_polygons` : Takes an array of polytopes as an input and gives them random, bounded velocities, thus forming dynamic polytopes in timespace.

`fcn_make_facets_from_verts` : Takes a timespace or 3D polytope, defined by its vertices, and associates these vertices into facets (the 2D faces of the 3D shape) so it is clear how the vertices should be connected by plane segments.

<pre align="center">
<img src=".\Images\facets.png">
<figcaption>An example of a triangular polytope moving in time, shown as individual facets which can be "flat" representing the triangular polytope at different times or "side walls" representing the translation of the polytope in time.</figcaption>
</pre>

`fcn_make_triangular_surfels_from_facets` : Decomposes timespace polytopes into triangular surface elements (surfels) which are necessary for performing intersection checking between possible visibity graph edges and polytope obstacles.

<pre align="center">
<img src=".\Images\surfels.png">
<figcaption>An example of the flat and side wall facets shown for the timespace obstacle above, now broken down into triangular surface elements.</figcaption>
</pre>

`fcn_interpolate_polytopes_in_time.m` : Adds nodes to the input polytopes to increase the density of nodes in the time (z) dimension so that the planner has more options of where to route.

<pre align="center">
<img src=".\Images\interpolated.png">
<figcaption>A cartoon of a polytope with the minimum vertices (left) and the interpolated vertices (right).</figcaption>
</pre>

<pre align="center">
<img src=".\Images\interpolated_verts.png">
<figcaption>The triangular timespace obstalce shown above, displayed as only its vertices, after interpolation, in cyan.</figcaption>
</pre>


`fcn_interpolate_route_in_time.m` : Adds waypoints to the input route to increase the density of points according to the input time step.  This is useful prior to crafting an animation to get the route and polytopes to have the same time step.

`fcn_animate_timepace_path_plan.m` : Uses the `gif` library to plot the vehicle position, route progress, and polytope positions at each time step, then creating a gif frame from this plot, and saving all plotted frames as a gif for animating timespace (XYZ) routes and obstalces as a series of 2D (XY) plots.  See examples below.

Additionally, each of the functions includes a docstring that explains inputs and outputs. These are supported by MATLAB's help style so that one can type:

```sh
help fcn_fcnname
```

for any function to view function details.

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

<!-- USAGE EXAMPLES -->
## Usage
<!-- Use this space to show useful examples of how a project can be used.
Additional screenshots, code examples and demos work well in this space. You may
also link to more resources. -->

### Examples


The following are scripts that demonstrate 3D planning capabilites

#### script_test_3d

This script sets up a timespace map consisting of a two point moving wall in between the start and the goal.  It runs the planner on this map and plots the results.
<pre align="center">
<img src=".\Images\moving_wall_still.png ">
<figcaption>Route around a two point moving wall shown in timespace.</figcaption>
</pre>

<pre align="center">
<img src=".\Images\moving_wall.gif">
<figcaption>Route around a two point moving wall shown in 2D space as an animation.</figcaption>
</pre>

#### script_test_3d_polytope

This script sets up a timespace map consisting of a three point moving polytope in between the start and the goal.  It runs the planner on this map and plots the results.

<pre align="center">
<img src=".\Images\moving_poly_still.png">
<figcaption>Route around a three point moving polytope shown in timespace.</figcaption>
</pre>

<pre align="center">
<img src=".\Images\moving_poly.gif">
<figcaption>Route around a three point moving polytope shown in 2D space as an animation.</figcaption>
</pre>

#### script_test_3d_polytopes_multiple

This script sets up a timespace map consisting of a many moving polytopes with random bounded velocities in between the start and the goal.  It runs the planner on this map and plots the results.

<pre align="center">
<img src=".\Images\time_space_route.png">
<figcaption>Route thorugh dynamic polytope field shown in timespace.</figcaption>
</pre>

<pre align="center">
<img src=".\Images\timespace_route.gif">
<figcaption>Route through dynamic polytope field shown in 2D space as an animation.</figcaption>
</pre>

#### script_test_3d_polytope_canyon


This script sets up a timespace map consisting of a polytope field with a canyon-like bottle neck in the middle representing the obvious path between the start and the goal.  It runs the planner on this map and plots the results.  This map is useful for highlighting what happens if visibility and reachability are prioritized in the cost function as the chokepoint will be avoided.

<pre align="center">
<img src=".\Images\canyon.png">
<figcaption>Routes thorugh polytope field with a chokepoint showing the effects of raising the weight on the visibility and reachability cost terms.</figcaption>
</pre>

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

## Major release versions

This code is still in development (alpha testing)

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

<!-- CONTACT -->
## Contact

Steve Harnett - sjharnett@psu.edu

Project Link: [https://github.com/ivsg-psu/3d_grid_free_path_planning](https://github.com/ivsg-psu/3d_grid_free_path_planning)

<a href="#3d_(xyt)_grid_free_path_planning">Back to top</a>

<!-- MARKDOWN LINKS & IMAGES -->

<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links
[contributors-shield]: https://img.shields.io/github/contributors/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[contributors-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[forks-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/network/members
[stars-shield]: https://img.shields.io/github/stars/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[stars-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/stargazers
[issues-shield]: https://img.shields.io/github/issues/ivsg-psu/reFeatureExtraction_Association_PointToPointAssociationpo.svg?style=for-the-badge
[issues-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/issues
[license-shield]: https://img.shields.io/github/license/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[license-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/blob/master/LICENSE.txt -->
