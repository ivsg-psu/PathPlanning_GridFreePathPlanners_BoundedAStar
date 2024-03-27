clear; close all; clc
% script_test_voronoi_planning
% test script of planning along voronoi diagram edges

addpath(strcat(pwd,'\..\..\PathPlanning_PathTools_PathClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'));
addpath(strcat(pwd,'\..\..\Errata_Tutorials_DebugTools\Functions'));

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;
flag_do_plot_slow = 0;

for map_idx =5
    if map_idx == 1 % generic canyon map
        %% load test fixtures for polytope map rather than creating it here
        % load distribution north of canyon
        load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes1.mat'));
        % this test fixture was made with the following block of code using functions from the MapGen repo
        % tiled_polytopes1 = fcn_MapGen_haltonVoronoiTiling([1,20],[2 1]);
        % % remove the edge polytope that extend past the high and low points
        % % shink the polytopes so that they are no longer tiled
        % des_radius = 0.05; % desired average maximum radius
        % sigma_radius = 0.002; % desired standard deviation in maximum radii
        % min_rad = 0.0001; % minimum possible maximum radius for any obstacle
        % [shrunk_polytopes1,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes1,des_radius,sigma_radius,min_rad);

        % load polytopes representing canyon
        load(strcat(pwd,'\..\Test_Fixtures\canyon_polys_without_exterior.mat'));
        % these polytopes were manually defined

        % load distribution south of canyon
        load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes2.mat'));
        % this test fixture was made with the following block of code using functions from the MapGen repo
        % tiled_polytopes2 = fcn_MapGen_haltonVoronoiTiling([1, 20],[2 1]);
        % % remove the edge polytope that extend past the high and low points
        % % shink the polytopes so that they are no longer tiled
        % [shrunk_polytopes2,~,~] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes2,des_radius,sigma_radius,min_rad);
        %% move second polytope field north of canyon
        second_field_vertical_translation = 1.5;
        for i = 1:length(shrunk_polytopes2)
            num_verts_this_poly = length(shrunk_polytopes2(i).yv);
            shrunk_polytopes2(i).yv = shrunk_polytopes2(i).yv + second_field_vertical_translation;
            shrunk_polytopes2(i).vertices = shrunk_polytopes2(i).vertices + [zeros(num_verts_this_poly+1,1) second_field_vertical_translation*ones(num_verts_this_poly+1,1)];
        end

        %% combine two polytope fields and canyon choke point into one field
        shrunk_polytopes = [shrunk_polytopes1, shrunk_polytopes2, polytopes_manual_canyon];
        %% define start and finish
        start_init = [0 1.25];
        finish_init = [2 1.25];
    elseif map_idx == 2 % the lower triangular flood plain
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_1.mat'));
        shrunk_polytopes = flood_plain_1;
        start_init = [-78.3 40.88];
        % finish_init = [-78.1 40.9];
        finish_init = [-78.07 40.82];
    elseif map_idx == 3 % the mustafar mining rig map (the comb)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_2.mat'));
        shrunk_polytopes = flood_plain_2;
        start_init = [-78.02 40.96];
        % finish_init = [-77.86 40.93];
        finish_init = [-77.82 40.97];
    elseif map_idx == 4 % also good for edge deletion case (the long river valleys)
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_3.mat'));
        shrunk_polytopes = flood_plain_3;
        start_init = [-77.49 40.84];
        % finish_init = [-77.58 40.845];
        finish_init = [-77.68 40.85];
    elseif map_idx == 5 % bridge map, good for random edge deletion case
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_4.mat'));
        shrunk_polytopes = flood_plain_4;
        is_nonconvex = 1;
        start_init = [-77.68 40.9];
        finish_init = [-77.5 40.8];
    elseif map_idx == 6 % large map, good for dilation case, nearly fully tiled
        load(strcat(pwd,'\..\Test_Fixtures\flood_plains\flood_plain_5.mat'));
        shrunk_polytopes = flood_plain_5;
        start_init = [-78.01 41.06];
        finish_init = [-77.75 40.93];
    elseif map_idx == 7 % generic polytope map
        is_nonconvex = 0;
        % pull halton set
        halton_points = haltonset(2);
        points_scrambled = scramble(halton_points,'RR2'); % scramble values

        % pick values from halton set
        Halton_range = [1801 1851];
        low_pt = Halton_range(1,1);
        high_pt = Halton_range(1,2);
        seed_points = points_scrambled(low_pt:high_pt,:);

        % fill polytopes from tiling
        AABB = [0 0 1 1];
        stretch = AABB(3:4);
        tiled_polytopes = fcn_MapGen_generatePolysFromVoronoiAABBWithTiling(seed_points,AABB, stretch);

        % stretch polytopes to cover more area
        new_stretch = [30 40];
        stretched_polytopes = [];
        for poly = 1:length(tiled_polytopes) % pull each cell from the voronoi diagram
            stretched_polytopes(poly).vertices  = tiled_polytopes(poly).vertices.*new_stretch;
        end % Ends for loop for stretch
        stretched_polytopes = fcn_MapGen_fillPolytopeFieldsFromVertices(stretched_polytopes);

        % shrink polytopes to desired radius
        des_rad = 2; sigma_radius = 0.4; min_rad = 0.1;
        [shrunk_polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(stretched_polytopes,des_rad,sigma_radius,min_rad);

        clear Halton_range
        clear halton_points
        clear points_scrambled

        start_init = [-2 20];
        finish_init = [32 20];
        % tile field to hedgerow by making a set above and a set below
    end % if conditions for different map test fixtures
end

% [vx,vy,h] = fcn_MapGen_generateVoronoiDiagramBetweenPolytopes(shrunk_polytopes,is_nonconvex)
% hold on; box on;
% for j = 1:length(shrunk_polytopes)
%      fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
% end
% xlabel('x [km]')
% ylabel('y [km]')

close all; clc;
my_poly = shrunk_polytopes;
figure; hold on; box on;
j = 2;fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',0.5)
% boundary.vertices = [-77.7 40.87; -77.7 40.915; -77.63 40.914; -77.63 40.87];
boundary.vertices = [-77.7 40.78; -77.7 40.92; -77.45 40.92; -77.45 40.78];
boundary.vertices = [boundary.vertices; boundary.vertices(1,:)];
boundary = fcn_MapGen_fillPolytopeFieldsFromVertices(boundary);
boundary.parent_poly_id = nan;
% boundary = fcn_MapGen_increasePolytopeVertexCount(boundary, 0.02);
shrunk_polytopes = [boundary, my_poly];

distances = diff([[shrunk_polytopes.xv]',[shrunk_polytopes.yv]']);
min_distance_between_verts = min(sqrt(sum(distances.*distances,2)));
% poly_map_stats = fcn_MapGen_polytopesStatistics(polytopes);
% % want to ensure that a side with length of 2 std dev below mean is still interpolated at least in half
% resolution = (poly_map_stats.average_side_length - 2*poly_map_stats.std_side_length)/2;
resolution = min_distance_between_verts/2;
shrunk_polytopes = fcn_MapGen_increasePolytopeVertexCount(shrunk_polytopes, 10*resolution);
C = [];
P = [];
largest_idx = 0;
for poly_idx = 1:length(shrunk_polytopes)
    P = [P; shrunk_polytopes(poly_idx).vertices(1:end-1,:)];
    num_verts = size(shrunk_polytopes(poly_idx).vertices,1)-1;
    C1 = [1:num_verts]';
    C2 = [C1(2:end);C1(1)];
    Ccomb = [C1 C2];
    Ccomb = Ccomb + largest_idx;
    C = [C; Ccomb];
    largest_idx = max(max(C));
end
x = P(:,1)
y = P(:,2)
DT = delaunayTriangulation(P,C)

figure; triplot(DT); title('triangulation')
inside = isInterior(DT);
tr = triangulation(DT(inside,:),DT.Points);
figure; triplot(tr); title('triangulation, no interior')
numt = size(tr,1);
T = (1:numt)';
neigh = neighbors(tr);
cc = circumcenter(tr);
% cc = incenter(tr);
nodes = find(~isnan(sum(neigh, 2)));
xcc = cc(:,1);
ycc = cc(:,2);
idx1 = T < neigh(:,1);
idx2 = T < neigh(:,2);
idx3 = T < neigh(:,3);
neigh = [T(idx1) neigh(idx1,1); T(idx2) neigh(idx2,2); T(idx3) neigh(idx3,3)]';
figure; hold on; box on;
triplot(tr,'g')
hold on
plot(xcc(neigh), ycc(neigh), '-r','LineWidth',1.5)
plot(xcc(nodes), ycc(nodes), '.k','MarkerSize',30)
plot(x(C'),y(C'),'-b','LineWidth',1.5)
xlabel('Medial Axis of Polygonal Domain','FontWeight','b')
% TODO @sjharnett
% for each triangle, get each side length, keep max - data structure of tri max sides

% for any two three connected nodes, get all tris in between - neigh()
% recurse through all two connected nodes, pick any direction except where you came from
% cont. until you've reached a 3 connected node noting node and all traversed triangles

% for any set of tris, keep min of max edges
return
close all; clear all; clc;
load trimesh3d
trisurf(tri,x,y,z)
dt = delaunayTriangulation(x,y,z)
tr = triangulation(dt(:,:),dt.Points)
trisurf(tri,x,y,z)
numt = size(tr,1);
T = (1:numt)';
neigh = neighbors(tr);
cc = circumcenter(tr);
% cc = incenter(tr);
nodes = find(~isnan(sum(neigh, 2)));
xcc = cc(:,1);
ycc = cc(:,2);
zcc = cc(:,3);
idx1 = T < neigh(:,1);
idx2 = T < neigh(:,2);
idx3 = T < neigh(:,3);
neigh = [T(idx1) neigh(idx1,1); T(idx2) neigh(idx2,2); T(idx3) neigh(idx3,3)]';
figure(1); hold on; box on;
trisurf(tri,x,y,z)
hold on
% plot3(xcc(neigh), ycc(neigh), zcc(neigh), '-r','LineWidth',1.5)
plot3(xcc(nodes), ycc(nodes), zcc(nodes), '.k','MarkerSize',30)
xlabel('Medial Axis of Polygonal Domain','FontWeight','b')

all_pts = [xcc, ycc, [1:length(xcc)]', -1*ones(length(xcc),1), zeros(length(xcc),1)];
vgraph = zeros(length(xcc));
neigh_orig = neighbors(tr);
for i = 1:size(neigh_orig,1)
    neigh_list = neigh_orig(i,:);
    neigh_list = neigh_list(~isnan(neigh_list));
    if ~(length(neigh_list) == 2)
        continue
    end
    vgraph(neigh_list(1),neigh_list(2)) = 1;
    vgraph(neigh_list(2),neigh_list(1)) = 1;
end
start = all_pts(end-1,:);
start(end) = 1;
finish = all_pts(end,:);
finish(end) = 1;
all_pts = all_pts(1:end-2,:);

start_for_reachability = start;
start_for_reachability(4) = start(3);
finish_for_reachability = finish;
finish_for_reachability(4) = finish(3);

[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start_for_reachability,finish_for_reachability);
if ~is_reachable
    error('initial mission, prior to edge deletion, is not possible')
end

mode = "xy spatial only";
% mode = 'time or z only';
% mode = "xyz or xyt";
[cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts, start, finish, mode);


[init_cost, init_route] = fcn_algorithm_Astar(vgraph, cgraph, hvec, all_pts, start, finish);


figure; hold on; box on;
xlabel('x [km]');
ylabel('y [km]');
plot(start(1),start(2),'xg','MarkerSize',6);
plot(finish(1),finish(2),'xr','MarkerSize',6);
leg_str = {'start','finish'};
plot(init_route(:,1),init_route(:,2),'LineWidth',2);
leg_str{end+1} = sprintf('route');
for j = 2:length(shrunk_polytopes)
    fill(shrunk_polytopes(j).vertices(:,1)',shrunk_polytopes(j).vertices(:,2),[0 0 1],'FaceAlpha',1)
end
leg_str{end+1} = 'obstacles';
for i = 1:length(shrunk_polytopes)-1
    leg_str{end+1} = '';
end
legend(leg_str,'Location','best');
