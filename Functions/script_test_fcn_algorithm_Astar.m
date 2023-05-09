function [cost, route] = fcn_algorithm_Astar(cgraph, all_pts, start, finish)
clear
clc
close all

%% add necessary directories
addpath([pwd '\..\Example_Map_Generation_Code'])
addpath([pwd '\..\PathPlanning_MapTools_MapGenClassLibrary\Functions'])


%% map generation control
% repetition controls and storage
repetitions = 10;
% final information can be stored in one variable (not suggested to save
% variables of the structure type, as it will take a lot of memory)
final_info(repetitions) = struct('polytopes',[],'start',[],'finish',[],'path_x',[],'path_y',[],'appex1_x',[],'appex1_y',[],'appex2_x',[],'appex2_y',[]);
% generate Voronoi tiling from Halton points
pt_density = 100; % point density used for generation
low_pts = 1:pt_density:(pt_density*(repetitions-1)+1); % lower bound of Halton set range
high_pts = pt_density:pt_density:pt_density*repetitions; % upper bound of Halton set range
% remove the edge polytope that extend past the high and low points
xlow = 0; xhigh = 1; ylow = 0; yhigh = 1;
% shink the polytopes so that they are no longer tiled
des_radius = 0.05; % desired average maximum radius
sigma_radius = 0.002; % desired standard deviation in maximum radii
min_rad = 0.0001; % minimum possible maximum radius for any obstacle
shrink_seed = 1111; % seed used for randomizing the shrinking process
des_cost = 0; % polytope traversal cost
% starting (A) and finish (B) coordinates
A.x = 0.0; A.y = 0.5; B.x = 1; B.y = 0.5;
% uncomment below to start in a polytope
% A.x = 0.15; A.y = 0.54; B.x = 1; B.y = 0.5;

%% plotting control
flag_do_plot = 1; % 1 if you would like to see plots, anything else if not

for rep = 1:repetitions
    %% generate map
    % generate Voronoi tiling from Halton points
    low_pt = low_pts(rep); high_pt = high_pts(rep); % range of Halton points to use to generate the tiling
    tiled_polytopes = fcn_polytope_generation_halton_voronoi_tiling(low_pt,high_pt);
    % remove the edge polytope that extend past the high and low points
    trim_polytopes = fcn_polytope_editing_remove_edge_polytopes(tiled_polytopes,xlow,xhigh,ylow,yhigh);
    % shink the polytopes so that they are no longer tiled
    rng(shrink_seed) % set the random number generator with the shrink seed
    shrunk_polytopes = fcn_polytope_editing_shrink_to_average_max_radius_with_variance(trim_polytopes,des_radius,sigma_radius,min_rad);
    shrunk_polytopes = fcn_polytope_editing_set_all_costs(shrunk_polytopes,des_cost);
    % if starting in a polytope, at 0.15, 0.45 per above, this controls its cost
    % shrunk_polytopes(31).cost = 0.1;
    % plot the map
    if flag_do_plot
        fig = 99; % figure to plot on
        line_spec = 'b-'; % edge line plotting
        line_width = 2; % linewidth of the edge
        axes_limits = [0 1 0 1]; % x and y axes limits
        axis_style = 'square'; % plot axes style
        fcn_plot_polytopes(shrunk_polytopes,fig,line_spec,line_width,axes_limits,axis_style);
    end
    %% plan path
    cgraph = [];
    all_pts = [];
    start = [];
    finish = [];
    % TODO @sjharnett create a setup function for new Astar?
%     [path,cost,err] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(shrunk_polytopes,A,B,'legacy');
    [cost, path] = fcn_algorithm_Astar(cgraph, all_pts, start, finish)
    % path: series of points [x y point_id obs_id beg_end]
    % cost: path length
    % err: marker indicating if there was an error in setup (1) or not (0)

    % plot path
    if flag_do_plot
        plot(path(:,1),path(:,2),'k-','linewidth',2)
        plot(A.x, A.y, 'gx','linewidth',2)
        plot(B.x, B.y, 'rx','linewidth',2)
    end


    if flag_do_plot
        my_title = sprintf('Path length [m]: %.4f',cost)
        title(my_title)
        box on
    end
end
