clear; close all; clc
% script_test_3d_polytope
% a basic test of a 3D path planning scenario in timespace
% there is are three basic manually defined polytopes between the start and the goal that the planner routes around

addpath 'C:\Users\sjhar\OneDrive\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\OneDrive\Desktop\gif\gif'

addpath 'C:\Users\sjhar\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjhar\Desktop\gif\gif'

addpath 'C:\Users\sjh6473\Desktop\TriangleRayIntersection'
addpath 'C:\Users\sjh473\Desktop\gif\gif'
tic

flag_do_plot = 1;
flag_do_slow_plot = 1;
flag_do_animation = 0;

%% manually define three polytopes
verts = [1 1 0 1; 1 2 0 2; 2 2 0 3; 2 1 0 4; 1 1 10 1; 1 2 10 2; 2 2 10 3; 2 1 10 4]; % a line that translates its length in x over the course of 20 seconds
time_space_polytopes(1).vertices = verts;

%% turn polytopes from vertices into facets
time_space_polytopes = fcn_make_facets_from_verts(time_space_polytopes);

%% turn polytopes into triangular surfels for intersection checking
all_surfels = fcn_make_triangular_surfels_from_facets(time_space_polytopes);
if flag_do_plot
    figure; hold on; box on; title('triangular surfels of polytopes in timespace')
    fig = gcf;
    for i = 1:size(all_surfels,1)
        fill3([all_surfels(i,1) all_surfels(i,4) all_surfels(i,7)], [all_surfels(i,2) all_surfels(i,5) all_surfels(i,8)], [all_surfels(i,3) all_surfels(i,6) all_surfels(i,9)],rand(1,3),'FaceAlpha',0.3);
    end
    INTERNAL_fcn_format_timespace_plot();
end

%% manually define start and finish
start = [2.5 7 0];
% finish = [1.5*ones(6,1) -1*ones(6,1) (21:2:31)']; % multiple time static finish
finish = [1.5 -1 21; 2 -1 31]; % moving finish
dt = 10;
finish = fcn_interpolate_route_in_time(finish,dt);
num_finish_pts = size(finish,1);
starts = [2*ones(num_finish_pts,1) 2*ones(num_finish_pts,1) zeros(num_finish_pts,1)];

%% interpolate polytopes in time
[verts, time_space_polytopes] = fcn_interpolate_polytopes_in_time(time_space_polytopes,dt);

%% make all_pts matrix
verts = verts(:,1:3);
all_pts = [verts; start; finish];
num_verts = size(verts,1);
num_pts = size(all_pts,1);
all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts

num_starts = size(start,1);
num_finishes = size(finish,1);

if flag_do_plot
    figure; hold on; box on; title('all vertices and start and finish')
    INTERNAL_fcn_format_timespace_plot();
    plot3(start(1),start(2),start(3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    plot3(verts(:,1),verts(:,2),verts(:,3),'cx')
end
verts_with_ids = all_pts(1:num_verts,:);
start_with_ids = all_pts(num_verts+1:num_verts+num_starts,:);
finish_with_ids = all_pts(num_verts+num_starts+1:num_verts+num_starts+num_finishes,:);

%% make vgraph
speed_limit = 1/1.25;
vgraph = fcn_visibility_graph_3d_global(verts, start, finish, all_surfels, speed_limit, time_space_polytopes, dt);

%% make rgraph
[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph, start_with_ids, finish_with_ids);

%% plan route
[cost, route] = fcn_algorithm_Astar3d(vgraph, verts_with_ids, start_with_ids, finish_with_ids, rgraph);
% route metrics follow
total_time = max(route(:,3));
route_x = route(:,1);
route_y = route(:,2);
route_t = route(:,3);
lengths = diff([route_x(:) route_y(:)]);
total_length = sum(sqrt(sum(lengths.*lengths,2)));
lengths_3d = diff([route_x(:) route_y(:) route_t(:)]);
total_length_3d = sum(sqrt(sum(lengths_3d.*lengths_3d,2)));
metrics_title = sprintf('route duration [s]: %.3f \n route length [m]: %.3f \n route length 3D: %.3f',total_time,total_length,total_length_3d);
title(metrics_title);

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

%% vgraph plot
if flag_do_slow_plot
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
test_pt_1 = 2;
test_pt_2 = 16;
all_pts(2,:);
all_pts(16,:);
figure; hold on; box on; title('visibility graph test internal edge');
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
example_vgraph_val = vgraph(test_pt_1,test_pt_2);
if example_vgraph_val
    color = 'g';
else
    color = 'r';
end
plot3([all_pts(test_pt_1,1), all_pts(test_pt_2,1)],[all_pts(test_pt_1,2), all_pts(test_pt_2,2)],[all_pts(test_pt_1,3), all_pts(test_pt_2,3)],color,'LineWidth',2)
view([1 0 0])

assert(~vgraph(test_pt_2,test_pt_1))
assert(~vgraph(test_pt_1,test_pt_2))

return
%% example of speed limit inforcement
if flag_do_plot
    my_title = sprintf('example of speed limit enforcement,\n speed limit %0.1f m/s',speed_limit);
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
toc

if flag_do_animation
    fcn_animate_timespace_path_plan(start, finish, time_space_polytopes, route_dense, dt, [1 3], [-2 8]);
end

function INTERNAL_fcn_format_timespace_plot()
    % define figure properties
    opts.width      = 8;
    opts.height     = 6;
    opts.fontType   = 'Times';
    opts.fontSize   = 9;
    fig = gcf;
    % scaling
    fig.Units               = 'centimeters';
    fig.Position(3)         = opts.width;
    fig.Position(4)         = opts.height;

    % set text properties
    set(fig.Children, ...
        'FontName',     'Times', ...
        'FontSize',     9);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('t [s]')
    view([36 30])
end
