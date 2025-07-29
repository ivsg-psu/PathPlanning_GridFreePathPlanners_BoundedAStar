% script_test_fcn_Visibility_clearAndBlockedPoints

% a basic test of calculation of clear and blocked points for visibility
% graph creation

% Revision history
% 2025_07_25 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_visibility_clear_and_blocked_points

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

% Create polytope field
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],[], ([100 100]), (-1));

% Trim polytopes on edge of boundary
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0.1 0.1 99.9 99.9], (-1));

% Shrink polytopes to form obstacle field
shrunk_polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));

% Get x and y coordinates of each polytope
xvert = [shrunk_polytopes.xv];
yvert = [shrunk_polytopes.yv];
point_tot = length(xvert);

% Create start and finish points
start = [0 50 point_tot+1 -1 0];
finish = [[100; xvert'] [50; yvert'] [point_tot+2; [1:point_tot]'] [0; ones(point_tot,1)] [0; zeros(point_tot,1)]];

% Call function to determine clear and blocked points
isConcave = [];
[clear_pts,blocked_pts]=fcn_Visibility_clearAndBlockedPoints(shrunk_polytopes,start,finish,(isConcave),(-1));

% Plot results
plotFormat.LineWidth = 3;
plotFormat.MarkerSize = 10;
plotFormat.LineStyle = '-';
plotFormat.Color = [0 0 1];

% fillFormat = [1 0 0 0 0.5];
fillFormat = [];
h_plot = fcn_MapGen_plotPolytopes(shrunk_polytopes, (plotFormat),(fillFormat),(fig_num));

plot([start(1) finish(1,1)],[start(2) finish(1,2)],'kx','linewidth',1)
plot(clear_pts(:,1),clear_pts(:,2),'go','linewidth',1)
plot(blocked_pts(:,1),blocked_pts(:,2),'rx','linewidth',1)

