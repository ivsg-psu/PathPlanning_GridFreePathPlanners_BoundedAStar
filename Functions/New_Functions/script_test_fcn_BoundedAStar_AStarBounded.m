% script_test_fcn_BoundedAStar_AstarBounded
% a basic test of a Bounded A Star planner functionality

% Revision history
% 2025_07_25 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_algorithm_bound_Astar

% TO DO:
% (none)

%% Set up the workspace
close all

%% Code demos start here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____                              ____   __    _____          _
%  |  __ \                            / __ \ / _|  / ____|        | |
%  | |  | | ___ _ __ ___   ___  ___  | |  | | |_  | |     ___   __| | ___
%  | |  | |/ _ \ '_ ` _ \ / _ \/ __| | |  | |  _| | |    / _ \ / _` |/ _ \
%  | |__| |  __/ | | | | | (_) \__ \ | |__| | |   | |___| (_) | (_| |  __/
%  |_____/ \___|_| |_| |_|\___/|___/  \____/|_|    \_____\___/ \__,_|\___|
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Demos%20Of%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 1

close all;
fprintf(1,'Figure: 1XXXXXX: DEMO cases\n');

%% DEMO case: plan a path through a specified field of polytope obstacles
fig_num = 10001;
titleString = sprintf('DEMO case: plan a path through a specified field of polytope obstacles');
fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
figure(fig_num); clf;

addpath([pwd '\Example_Map_Generation_Code'])

 % Load map from name 
 map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.04 0.008 1e-6 1111";
 plot_flag = 0; disp_name = 0; fig_num = 654654;
 [polytopes,fig]=fcn_MapGen_generatePolysFromName(map_name,plot_flag,disp_name);

 % Do path planning with shrunk polytopes
 % Get x and y locations of vertices
 xv = [polytopes.xv];
 yv = [polytopes.yv];
 point_tot = length(xv);

 % Specify start and finish point locations and IDs
 start = [0 0.5 point_tot+1 0 0];
 finish = [1 0.5 point_tot+2 -1 0];

 % Initialize beginning/end and obstacle ID vectors
 beg_end = zeros(point_tot,1);
 obs_id = zeros(point_tot,1);
 
 % Loop through polytopes to assign obstacle ID and beginning/end flags to
 % vertices
 curpt = 0;
 for poly = 1:size(polytopes,2) % check each polytope
     verts = length(polytopes(poly).xv);
     obs_id(curpt+1:curpt+verts) = ones(verts,1)*poly; % obs_id is the same for every vertex on a single polytope
     beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
     curpt = curpt+verts;
 end

 % Create all_pts matrix
 all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];

 % Create set of bound_pts
 bound_pts = all_pts;

 % Set planner mode and call path planner
 planner_mode = 'legacy';
 [cost,route]=fcn_BoundedAStar_AstarBounded(start,finish,polytopes,all_pts,bound_pts,planner_mode,(ellipse_polytopes),([]));
 disp(['Path Cost: ' num2str(cost)])
 
 % Plot results
 fcn_BoundedAStar_plotPolytopes(polytopes,100,'b-',2,[0 1 0 1],'square')
 plot(route(:,1),route(:,2),'k-','linewidth',2)
 plot([start(1) finish(1)],[start(2) finish(2)],'kx','linewidth',2)
 box on
 xlabel('X Position')
 ylabel('Y Position')