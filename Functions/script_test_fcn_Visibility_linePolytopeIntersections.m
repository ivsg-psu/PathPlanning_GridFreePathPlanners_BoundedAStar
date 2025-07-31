% script_test_fcn_Visibility_linePolytopeIntersections

% a basic test of calculation of line intersections with polytopes

% Revision history
% 2025_07_31 - K. Hayes, kxh1031@psu.edu
% -- first write of script, code taken from example originally in
%    fcn_visibility_line_polytope_intersections

% TO DO:
% -- fix plotting to be inside fcn and not use deprecated plotting fcn

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

% Set vertices of polytope
xv = [2 3 5 6 6 5 3 2];
yv = [3 2 2 3 5 6 6 5];
polytopes.vertices = [[xv xv(1)]' [yv yv(1)]'];
polytopes.xv = xv;
polytopes.yv = yv;

% Find distances of polytope edges
polytopes.distances = sum((polytopes.vertices(1:end-1,:) - polytopes.vertices(2:end,:)).^2,2).^0.5;

% Find polytope area and centroid location
[centroid,polytope.area] = fcn_MapGen_polytopeCentroidAndArea ([[xv xv(1)]',[yv yv(1)]']);
polytopes.mean = [centroid];

% Find maximum radius of polytope
polytopes.max_radius = max(sum((polytopes.vertices(1:end-1,:) - ones(length(xv),1)*polytopes.mean).^2,2).^0.5);

% Append start and finish points
point_tot = length(xv);
start = [0 0 point_tot+1 -1 0];
finish = [8 8 point_tot+2 0 0];

% Find clear and blocked points
[clear_pts,blocked_pts,D,di,dj,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP,xjQ,yjQ]=fcn_Visibility_clearAndBlockedPoints(polytopes,start,finish);
xings = fcn_Visibility_linePolytopeIntersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,polytopes);

% NOTE - deprecated plotting
fcn_BoundedAStar_plotPolytopes(polytopes,[],'b-',2,[],'square')

for ints = 1:length(D)
 if D(ints)~=0
     plot([xjP(ints) xjQ(ints)],[yjP(ints) yjQ(ints)],'m-','linewidth',2)
 end
end
plot([start(1) finish(1)],[start(2) finish(2)],'k--','linewidth',2)
for xing = 1:length(xings(end).index)
 plot(xings(end).points(xing,1),xings(end).points(xing,2),'kx','linewidth',1)
end