% script_demo_fcn_BoundedAStar_Astar3d.m
% a basic test of a 3D path planning scenario in timespace
% there is a single moving wall between the start and the goal that the planner routes around

% Revision history
% 2025_07_03 - S. Brennan, sbrennan@psu.edu
% -- created code from script_test_3d.m written by S. Harnett
% -- standardized headers on all test scripts

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
% 
% %% DEMO case: call the function to show it operating on the 9th data set
% fig_num = 10001;
% titleString = sprintf('DEMO case: call the function to show it operating on the 9th data set');
% fprintf(1,'Figure %.0f: %s\n',fig_num, titleString);
% figure(fig_num); clf;
% 


clear; close all; clc


flag_do_plot = 1;
flag_do_animation = 0;

facets = [];
verts = [1 1 0; 2 1 0;  3 1 20; 2 1 20]; % a line that translates its length in x over the course of 20 seconds
% verts = [1 1 0; 2 1 0;  3 1 20; 2 1 20; 1 1.2 0; 2 1.2 0;  3 1.2 20; 2 1.2 20]; % make the line a box
start = [2 0 0];
finish = [2*ones(6,1) 2*ones(6,1) (11:2:21)']; % multiple time static finish
% finish = [2 2 11; 1.25 1.25 21]; % moving finish
dt = 1;
finish = fcn_interpolate_route_in_time(finish,dt);
num_finish_pts = size(finish,1);
starts = [2*ones(num_finish_pts,1) 2*ones(num_finish_pts,1) zeros(num_finish_pts,1)];

%% demonstrate TriangleRayIntersction function, the core of the visibility graph intersection checking
if flag_do_plot
    figure; hold on; box on; title('surfels and line from start to goal')
    fig = gcf;
    fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b');
    fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'r');
    plot3(start(1),start(2),start(3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    plot3([start(1) finish(1,1)],[start(2) finish(2,2)],[start(3) finish(3,3)])
    % finish = start + dir
    % thus dir = finish-start
    [intersect, t, u, v, xcoor] = TriangleRayIntersection (starts, finish-starts, verts(1,:),verts(2,:),verts(3,:),'lineType','segment','border','exclusive');
    [intersect2, t2, u2, v2, xcoor2] = TriangleRayIntersection (starts, finish-starts, verts(1,:),verts(3,:),verts(4,:),'lineType','segment','border','exclusive');
    plot3(xcoor(1),xcoor(2),xcoor(3),'cx')
    legend('tri 1','tri 2','start','goal','','intersection')
    INTERNAL_fcn_format_timespace_plot();
end


verts_orig = verts;
verts = [1 1 0 1; 2 1 0 2;  3 1 20 2; 2 1 20 1]; % add IDs to vertices
time_space_polytopes(1).vertices = verts; % create a single polytope for the line
[verts, time_space_polytopes] = fcn_interpolate_polytopes_in_time(time_space_polytopes,dt);

%% this code is required to vectorize the edge, triangle intersection checking
verts = verts(:,1:3);
all_pts = [verts; start; finish];

num_verts = size(verts,1);
num_pts = size(all_pts,1);

all_pts_idx = 1:1:num_pts; % array of all possible pt idx
all_pts = [all_pts all_pts_idx']; % add pt ID column to all_pts
all_surfels = [verts_orig(1,:),verts_orig(2,:),verts_orig(3,:);verts_orig(1,:),verts_orig(3,:),verts_orig(4,:)]; % manual triangution of facet since there is only one, normally this is performed with fcn_make_triangular_surfels_from_facets

%% plot pre-interpolated vertices to show density in time
if flag_do_plot
    figure; hold on; box on; title('all vertices and start and finish')
    INTERNAL_fcn_format_timespace_plot();
    plot3(start(1),start(2),start(3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    plot3(verts_orig(:,1),verts_orig(:,2),verts_orig(:,3),'cx')
end

%% create visibility graph
speed_limit = 100;
vgraph = fcn_visibility_graph_3d_global(verts, start, finish, all_surfels, speed_limit,time_space_polytopes, dt);
start = all_pts(num_verts+1,:);
finish = all_pts(num_verts+2:end,:);
[is_reachable, num_steps, rgraph] = fcn_check_reachability(vgraph,start(:,4),finish(:,4));

% mode = 'time or z only';
mode = 'xyz or xyt';
% mode = 'xy spatial only';
[cgraph, hvec] = fcn_algorithm_generate_cost_graph(all_pts(1:num_verts,:), all_pts(num_verts+1,:), all_pts(num_verts+2:end,:), mode);

%% plan route
[cost, route] = fcn_BoundedAStar_Astar3d(vgraph, cgraph, hvec, all_pts(1:num_verts,:), all_pts(num_verts+1,:), all_pts(num_verts+2:end,:));
% route metrics follow
total_time = max(route(:,3));
route_x = route(:,1);
route_y = route(:,2);
route_t = route(:,3);
lengths = diff([route_x(:) route_y(:)]);
total_length = sum(sqrt(sum(lengths.*lengths,2)));
lengths_3d = diff([route_x(:) route_y(:) route_t(:)]);
total_length_3d = sum(sqrt(sum(lengths_3d.*lengths_3d,2)));

%% plot route and metrics
if flag_do_plot
    metrics_title = sprintf('route duration [s]: %.3f \n route length [m]: %.3f \n route length 3D: %.3f',total_time,total_length,total_length_3d);
    title(metrics_title);
    fill3(verts_orig(1:3,1),verts_orig(1:3,2),verts_orig(1:3,3),'b','FaceAlpha',0.3);
    fill3(verts_orig([1,3,4],1),verts_orig([1,3,4],2),verts_orig([1,3,4],3),'b','FaceAlpha',0.3);
    plot3(route(:,1),route(:,2),route(:,3),'-b','LineWidth',3);
    plot3(start(:,1),start(:,2),start(:,3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
end

%% interpolated vertices
if flag_do_plot
    figure; hold on; box on; title('all vertices, interpolated, and start and finish')
    INTERNAL_fcn_format_timespace_plot();
    plot3(start(:,1),start(:,2),start(:,3),'gx');
    plot3(finish(:,1),finish(:,2),finish(:,3),'rx');
    plot3(verts(:,1),verts(:,2),verts(:,3),'cx')
end

if flag_do_plot
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

%% example of speed limit inforcement
if flag_do_plot
    my_title = sprintf('example of speed limit enforcement,\n speed limit %0.1f m/s',speed_limit);
    figure; hold on; box on; title(my_title);
    INTERNAL_fcn_format_timespace_plot();
    fill3(verts_orig(1:3,1),verts_orig(1:3,2),verts_orig(1:3,3),'b','FaceAlpha',0.3);
    fill3(verts_orig([1,3,4],1),verts_orig([1,3,4],2),verts_orig([1,3,4],3),'b','FaceAlpha',0.3);
    beg = 10;
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
    view([154 12])
end

if flag_do_animation
    %% change some things for the animation...
    dt = 0.25; % use denser interpolation for more frames
    route_dense = fcn_interpolate_route_in_time(route,dt); % interpolate the route
    [verts, time_space_polytopes] = fcn_interpolate_polytopes_in_time(time_space_polytopes,dt); % interpolate the polytopes
    finish = [2*ones(11,1) 2*ones(11,1) (0:2:21)']; % show the finish at all times
    finish = fcn_interpolate_route_in_time(finish,dt); % interpolate the finish
    fcn_animate_timespace_path_plan(start, finish, time_space_polytopes, route_dense, dt,[0 4], [-1 3]);
end


function INTERNAL_fcn_format_timespace_plot()
    % define figure properties
    opts.width      = 12;
    opts.height     = 9;
    opts.fontType   = 'Times New Roman';
    opts.fontSize   = 12;
    fig = gcf;
    % scaling
    fig.Units               = 'centimeters';
    fig.Position(3)         = opts.width;
    fig.Position(4)         = opts.height;
    set(gcf,'color','white')
    % set text properties
    set(fig.Children, ...
        'FontName',     'Times New Roman', ...
        'FontSize',     12);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [km]')
    ylabel('y [km]')
    zlabel('t [min]')
    view([36 30])
end
