% script_test_fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath
% a basic test of a function to calculate minimum perimeter path

% Revision history
% 2025_07_25 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_bounding_ellipse_min_perimeter_path
% 2025_10_03 - K. Hayes
% -- changed name to
%    script_test_fcn_BoundedAStar_calculateBoundingEllipseMinPerimPa to avoid
%    overlength error

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

% Generate polytopes
polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 100],([]),([100 100]),(-1));

% Trim polytopes from edges
trim_polytopes = fcn_MapGen_polytopesDeleteByAABB( polytopes, [0 0 100 100], (-1));

% Shrink polytopes to form obstacle field
shrunk_polytopes = fcn_MapGen_polytopesShrinkEvenly(trim_polytopes, 2.5, (-1));
point_tot = length([shrunk_polytopes.xv]);

% Specify start, finish points
start = [0 50 point_tot+1 0 0];
finish = [100 50 point_tot+2 -1 0];

% Create visibility graph
[clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_Visibility_clearAndBlockedPoints(shrunk_polytopes,start,finish);

% Determine polytopes that have intersections with the visibility graph (?)
intersections=fcn_Visibility_linePolytopeIntersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,shrunk_polytopes);
int_polytopes = shrunk_polytopes(intersections(end).obstacles);

% Call function to calcluate the minimum perimeter path (?)
max_dist=fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath(int_polytopes,intersections,start,finish,(fig_num));

% Plot results
plotFormat.LineWidth = 2;
plotFormat.MarkerSize = 10;
plotFormat.LineStyle = '-';
plotFormat.Color = [0 0 1];

% fillFormat = [1 0 0 0 0.5];
fillFormat = [];
fcn_MapGen_plotPolytopes(shrunk_polytopes, (plotFormat),(fillFormat),(fig_num));

plotFormat.LineWidth = 2;
plotFormat.MarkerSize = 10;
plotFormat.LineStyle = '-';
plotFormat.Color = [1 0 0];

% fillFormat = [1 0 0 0 0.5];
fillFormat = [];
fcn_MapGen_plotPolytopes(int_polytopes, (plotFormat),(fillFormat),(fig_num));



plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
for xing = 1:length(intersections(end).index)
 plot(intersections(end).points(xing,1),intersections(end).points(xing,2),'kx','linewidth',1)
end
sgtitle(titleString)


