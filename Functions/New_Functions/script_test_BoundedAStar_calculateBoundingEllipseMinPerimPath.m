% script_test_fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath
% a basic test of a function to calculate minimum perimeter path

% Revision history
% 2025_07_25 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_bounding_ellipse_min_perimeter_path

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
polytopes=fcn_polytope_generation_halton_voronoi_tiling(1,100,[100,100]);

% Trim polytopes from edges
trim_polytopes=fcn_polytope_editing_remove_edge_polytopes(polytopes,0,100,0,100);

% Shrink polytopes to form obstacle field
shrunk_polytopes=fcn_BoundedAStar_polytopeEditingShrinkEvenly(trim_polytopes,2.5);
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
fcn_BoundedAStar_plotPolytopes(shrunk_polytopes,fig_num,'b-',2,[0 100 0 100],'square')
fcn_BoundedAStar_plotPolytopes(int_polytopes,fig_num,'r-',2)
plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
for xing = 1:length(intersections(end).index)
 plot(intersections(end).points(xing,1),intersections(end).points(xing,2),'kx','linewidth',1)
end
sgtitle(titleString)


