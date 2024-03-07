clear; close all; clc
% script_test_3d_polytope_multiple
% defacto example of 3D path planning scenario in timespace
% typical field of polytopes, each with a random velocity, that the planner routes around

addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'

addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\Desktop\gif\gif'

addpath 'C:\Users\sjh6473\Desktop\gif\gif'
addpath 'C:\Users\sjh6473\Desktop\TriangleRayIntersection'

%% load test fixtures for polytope map rather than creating it here
% load distribution north of canyon
load(strcat(pwd,'\..\Test_Fixtures\shrunk_polytopes.mat'));
% this test fixture was made with the following block of code using functions from the MapGen repo
% tiled_polytopes = fcn_MapGen_haltonVoronoiTiling([1,20],[1 1]);
% % remove the edge polytope that extend past the high and low points
% % shink the polytopes so that they are no longer tiled
% des_radius = 0.05; % desired average maximum radius
% sigma_radius = 0.002; % desired standard deviation in maximum radii
% min_rad = 0.0001; % minimum possible maximum radius for any obstacle
% [shrunk_polytopes,mu_final,sigma_final] = fcn_MapGen_polytopesShrinkToRadius(tiled_polytopes,des_radius,sigma_radius,min_rad);

flag_do_plot = 1;
flag_do_slow_plot = 0;
flag_do_animation = 0;

if flag_do_plot
    %% plot the map
    figure; hold on; box on;
    xlabel('x [km]');
    ylabel('y [km]');
    title('polytope map in x-y plane at time 0')
    for i = 1:length(shrunk_polytopes)
         fill(shrunk_polytopes(i).vertices(:,1)',shrunk_polytopes(i).vertices(:,2),[0 0 1],'FaceAlpha',0.3)
    end
end
number_of_time_samples = 10;
times1 = nan(1,number_of_time_samples);
times2 = nan(1,number_of_time_samples);
for time_sample_iter = 1:number_of_time_samples
rng(number_of_time_samples)
outer_time = tic;
%% make 2D spatial polytopes into 3D timespace polytopes with velocities, then break into triangular surfels
max_translation_distance = 0.15;
final_time = 20;
time_space_polytopes = fcn_make_timespace_polyhedra_from_polygons(shrunk_polytopes, max_translation_distance, final_time);

time_space_polytopes = fcn_make_facets_from_verts(time_space_polytopes);

all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes);

if flag_do_plot
    figure; hold on; box on; title('polytopes in timespace')
    fig = gcf;
    for i = 1:size(all_surfels,1)
        fill3([all_surfels(i,1) all_surfels(i,4) all_surfels(i,7)], [all_surfels(i,2) all_surfels(i,5) all_surfels(i,8)], [all_surfels(i,3) all_surfels(i,6) all_surfels(i,9)],rand(1,3),'FaceAlpha',0.3);
    end
    INTERNAL_fcn_format_timespace_plot();
end

%% define start and finish
start = [0 0.5 0];
finish = [1 0.5 0; 1 0.2 20]; % moving finish
dt = 5;
finish = fcn_interpolate_route_in_time(finish,dt);
num_finish_pts = size(finish,1);
starts = [start(1)*ones(num_finish_pts,1) start(2)*ones(num_finish_pts,1) start(3)*ones(num_finish_pts,1)];

%% interpolate vertices in time and form all_pts matrix
[verts, time_space_polytopes] = fcn_interpolate_polytopes_in_time(time_space_polytopes,dt);

verts = verts(:,1:3);
all_pts = [verts; start; finish];

num_verts = size(verts,1);

num_pts = size(all_pts,1);
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts

if flag_do_plot
    figure; hold on; box on; title('all vertices and start and finish')
    INTERNAL_fcn_format_timespace_plot();
    plot3(start(1),start(2),start(3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    plot3(verts(:,1),verts(:,2),verts(:,3),'cx')
end

%% set speed limit and form visibility graph
speed_limit = 0.12;
vgraph = fcn_visibility_graph_3d_global(verts, start, finish, all_surfels, speed_limit, time_space_polytopes, dt);

num_starts = size(start,1);
num_finishes = size(finish,1);


start_with_ids = all_pts(num_verts+1:num_verts+num_starts,:);
finish_with_ids = all_pts(num_verts+num_starts+1:num_verts+num_starts+num_finishes,:);
all_pts_with_ids_no_start_and_fin = all_pts(1:num_verts,:);

%% form reachability graph
[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph, start_with_ids, finish_with_ids);

%% make cgraph
mode = "xy spatial only";
% mode = 'time or z only';
% mode = "xyz or xyt";
[cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts_with_ids_no_start_and_fin, start_with_ids, finish_with_ids, mode);

%% plan route
% [vgraph_phantom, cgraph_phantom, hvec_phantom, finish_phantom, all_pts_with_ids_no_start_and_fin_phantom] = fcn_algorithm_create_phantom_goal(vgraph, cgraph, hvec, finish_with_ids, all_pts_with_ids_no_start_and_fin);
inner_time = tic;
% [cost, route] = fcn_algorithm_Astar3d(vgraph_phantom, cgraph_phantom, 0*hvec_phantom, all_pts_with_ids_no_start_and_fin_phantom, start_with_ids, finish_phantom);
[cost, route] = fcn_algorithm_Astar3d(vgraph, cgraph, hvec, all_pts_with_ids_no_start_and_fin, start_with_ids, finish_with_ids);
time1 = toc(inner_time);
times1(time_sample_iter) = time1;
% assert(isnan(route(end,1)))
% route = route(1:end-1,:);
% route metrics follow
total_time = max(route(:,3));
route_x = route(:,1);
route_y = route(:,2);
route_t = route(:,3);
lengths = diff([route_x(:) route_y(:)]);
total_length = sum(sqrt(sum(lengths.*lengths,2)));
lengths_3d = diff([route_x(:) route_y(:) route_t(:)]);
total_length_3d = sum(sqrt(sum(lengths_3d.*lengths_3d,2)));
metrics_title = sprintf('route duration [min]: %.3f \n route length [km]: %.3f \n route length 3D: %.3f',total_time,total_length,total_length_3d);

%% plot path on surfels
if flag_do_plot
    figure; hold on; box on; title(metrics_title);
    plot3(route(:,1),route(:,2),route(:,3),'-b','LineWidth',3);
    fig = gcf;
    for i = 1:size(all_surfels,1)
        X = [all_surfels(i,1), all_surfels(i,4), all_surfels(i,7)];
        Y = [all_surfels(i,2), all_surfels(i,5), all_surfels(i,8)];
        Z = [all_surfels(i,3), all_surfels(i,6), all_surfels(i,9)];
        fill3(X,Y,Z,rand(3,1),'FaceAlpha',0.3);
    end
    plot3(start(1),start(2),start(3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    INTERNAL_fcn_format_timespace_plot();
end

if flag_do_slow_plot
%% vgraph plot
    figure; hold on; box on; title('visibility graph');
    INTERNAL_fcn_format_timespace_plot();
    % start with surfels plot
    for i = 1:size(all_surfels,1)
        X = [all_surfels(i,1), all_surfels(i,4), all_surfels(i,7)];
        Y = [all_surfels(i,2), all_surfels(i,5), all_surfels(i,8)];
        Z = [all_surfels(i,3), all_surfels(i,6), all_surfels(i,9)];
        fill3(X,Y,Z,rand(3,1),'FaceAlpha',0.3);
    end
    plot3(start(1),start(2),start(3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    for beg = 1:size(vgraph,1)
        example_vgraph_row = vgraph(beg,:);
        for term = 1:1:length(example_vgraph_row)
            if example_vgraph_row(term)
                color = 'g';
            else
                color = 'r';
            end
            plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],color,'LineWidth',2)
        end
        view([1 0 0])
    end
end

if flag_do_slow_plot
%% example of speed limit inforcement
    my_title = sprintf('example of speed limit enforcement,\n speed limit %0.1f km/min',speed_limit);
    figure; hold on; box on; title(my_title);
    INTERNAL_fcn_format_timespace_plot();
    fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b','FaceAlpha',0.3);
    fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'b','FaceAlpha',0.3);
    beg = 20;
    example_vgraph_row = vgraph(beg,:);
    cone_apex = [0 0 0];
    cone_apex = [all_pts(beg,1) all_pts(beg,2) all_pts(beg,3)];
    speed_lim_time_change = 3;
    speed_lim_dist_change = speed_limit*speed_lim_time_change;
    [X,Y,Z]=cylinder((0:0.2:1)*speed_lim_dist_change,1000);
    M=makehgtform('translate',cone_apex);
    h=surf(X,Y,speed_lim_time_change*Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.3);
    for term = 1:1:length(example_vgraph_row)
        if example_vgraph_row(term)
            color = 'g';
        else
            color = 'k';
        end
        plot3([all_pts(beg,1), all_pts(term,1)],[all_pts(beg,2), all_pts(term,2)],[all_pts(beg,3), all_pts(term,3)],color,'LineWidth',2)
    end
    view([1 0 0])
end

route_dense = fcn_interpolate_route_in_time(route,dt);
time2 = toc(outer_time)
times2(time_sample_iter) = time2;
end
if flag_do_animation
    fcn_animate_timespace_path_plan(start, finish, time_space_polytopes, route_dense, dt, [0 1], [0 1]);
end

function INTERNAL_fcn_format_timespace_plot()
    % define figure properties
    opts.width      = 8.8;
    opts.height     = 6;
    opts.fontType   = 'Times New Roman';
    opts.fontSize   = 8;
    fig = gcf;
    % scaling
    fig.Units               = 'centimeters';
    fig.Position(3)         = opts.width;
    fig.Position(4)         = opts.height;

    % set text properties
    set(fig.Children, ...
        'FontName',     'Times New Roman', ...
        'FontSize',     8);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [km]')
    ylabel('y [km]')
    zlabel('t [min]')
    view([36 30])
end
