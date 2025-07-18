function time_space_polytopes = fcn_BoundedAStar_makeTimespacePolyhedrafromPolygons(shrunk_polytopes, max_translation_distance, final_time)
% fcn_BoundedAStar_makeTimespacePolyhedrafromPolygons
%
% Takes an array of polytopes as an input and gives them random,
% bounded velocities, thus forming dynamic polytopes in timespace.
%
%
%
% FORMAT:
%
% time_space_polytopes =
% fcn_BoundedAStar_makeTimespacePolyhedrafromPolygons(
%   shrunk_polytopes, max_translation_distance, final_time)
%
% INPUTS:
%
%   shrunk_polytopes: the 2D polytope struct array of the form generated by the MapGen repo, typically
%   generated with the functions fcn_MapGen_haltonVoronoiTiling and fcn_MapGen_polytopesShrink*
%   see MapGen repo for examples of how to generate polytope maps with these functions
%
%   max_translation_distance: the maximum distance any polytope will be allowed to translate from
%   time = 0 to the final time.  If this is set at half the gap size, no polytope collisions will occur
%
%   final_time: the end time of the timespace polytopes.  This is the maximum t-axis (z-axis) value
%
%
% OUTPUTS:
%
%     time_space_polytopes: a struct array with a vertices field holding the vertices of each polytope
%     vertices consists of 4 columns: x position, y position, time (z-axis position) and vertex id
%     the vertex ID is necessary for correctly mapping a vertex at one time to its position at the next time
%
% DEPENDENCIES:
%
% the repo PathPlanning_MapTools_MapGenClassLibrary is used for creating the inputs to this function
% but there are no dependencies in the source code of this function
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
% -- copied to new function from fcn_make_timespace_polyhedra_from_polygons
%    to follow library convention
%
% TO DO:
%
% -- fill in to-do items here.

    % loop through each 2D polytope
    for i = 1:length(shrunk_polytopes)
        % we want to give a random translation distance for the vertices so they are at their original
        % position, plus some random value less than max_translation_distance, by final_time

        % rand gives a value between 0 and 1 thus rand-0.5 gives a value between -0.5 and 0.5 thus 2*(rand-0.5)
        % gives a value between -1 and 1 thus we randomize magnitude and direction of the max_translation_distance
        vel_this_poly = 2*[(rand-0.5)*max_translation_distance (rand-0.5)*max_translation_distance];
        num_verts = length(shrunk_polytopes(i).xv); % get total number of verts
        vert_ids = (1:1:num_verts)'; % assign ID to each vert so it can be identified as itself at each time
        verts = [shrunk_polytopes(i).xv' shrunk_polytopes(i).yv' 0*ones(num_verts,1) vert_ids; % place each vert at its original position at time 0
                 shrunk_polytopes(i).xv'+vel_this_poly(1) shrunk_polytopes(i).yv'+vel_this_poly(2) final_time*ones(num_verts,1) vert_ids]; % place each vert at orig position, plus translation distance, at time final_time
        time_space_polytopes(i).vertices = verts;
    end
end
