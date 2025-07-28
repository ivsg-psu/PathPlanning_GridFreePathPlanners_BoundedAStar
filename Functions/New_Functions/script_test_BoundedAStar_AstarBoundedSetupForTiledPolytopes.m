% script_test_fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes
% a basic test of a Bounded A Star planner setup function

% Revision history
% 2025_07_25 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_algorithm_setup_bound_Astar_for_tiled_polytopes

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

% Use tiling to generate polytopes
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[100 100],[],-1);

% Trim polytopes along edge
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0 0 100 100], (-1));

% Shrink polytopes to form obstacles
shrunk_polytopes=fcn_BoundedAStar_polytopeEditingShrinkEvenly(trim_polytopes,2.5);

% Set start and end points
start.x = 0; start.y = 50;
finish.x = 100; finish.y = 50;

% Set up and find path
planner_mode = 'legacy';
bounds = [];
[path,cost,err]=fcn_BoundedAStar_AstarBoundedSetupForTiledPolytopes(shrunk_polytopes, start, finish, planner_mode,(bounds),(-1));
disp(['Path Cost: ' num2str(cost)])

% Plot results
fcn_BoundedAStar_plotPolytopes(shrunk_polytopes,100,'b-',2,[0 100 0 100],'square')
plot(path(:,1),path(:,2),'k-','linewidth',2)
plot([start.x finish.x],[start.y finish.y],'kx','linewidth',2)