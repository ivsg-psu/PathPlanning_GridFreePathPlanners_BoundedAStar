function [fig] = fcn_plot_polytopes(polytopes,fig_num,line_spec,line_width,varargin)
% FCN_PLOT_POLYTOPES plot the polytopes as specified
%
% [FIG]=FCN_PLOT_POLYTOPES(POLYTOPES,FIG_NUM,LINE_SPEC,LINE_WIDTH)
% returns:
% a figure with the polytopes plotted as specified
%
% with inputs:
% POLYTOPES: a 1-by-n seven field structure, where n <= number of polytopes
%   with fields:
%   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%     the number of the individual polytope vertices
%   xv: a 1-by-m vector of vertice x-coordinates
%   yv: a 1-by-m vector of vertice y-coordinates
%   distances: a 1-by-m vector of perimeter distances from one point to the
%     next point, distances(i) = distance from vertices(i) to vertices(i+1)
%   mean: average xy coordinate of the polytope
%   area: area of the polytope
%   max_radius: distance from the mean to the farthest vertex
% FIG_NUM: figure number to plot the values on
% LINE_SPEC: a string, line specifications such as color, 'r', and line or
% point type, '--'
% LINE_WIDTH: width of the line to be plotted
% By default the axes will be determined by the plot function
%
% [FIG]=FCN_PLOT_POLYTOPES(POLYTOPES,FIG_NUM,LINE_SPEC,LINE_WIDTH,COLOR)
% allows the user to specify the input:
% COLOR: a 1-by-3 vector to specify the RGB plot colors [Red Green Blue],
% where 0 <= Red,Green,Blue <= 1
%
% [FIG]=FCN_PLOT_POLYTOPES(POLYTOPES,FIG_NUM,LINE_SPEC,LINE_WIDTH,AXIS_LIMITS)
% allows the user to specify the input:
% AXIS_LIMTS: a 1-by-4 vector to specify the start and end of the x and y
% axes, [xstart xend ystart yend]
%
% [FIG]=FCN_PLOT_POLYTOPES(POLYTOPES,FIG_NUM,LINE_SPEC,LINE_WIDTH,AXIS_STYLE)
% allows the user to specify the input:
% AXIS_STYLE: controls the style of the axis, such as square or equal
%
% [FIG]=FCN_PLOT_POLYTOPES(POLYTOPES,FIG_NUM,LINE_SPEC,LINE_WIDTH,FILL_INFO)
% allows the user to specify the input:
% FILL_INFO: a 1-by-5 vector to specify wether or not there is fill, the
% color of fill, and the opacity of the fill [Y/N, R, G, B, alpha]
%
% [FIG]=FCN_PLOT_POLYTOPES(POLYTOPES,FIG_NUM,LINE_SPEC,LINE_WIDTH,COLOR,AXIS_LIMITS,AXIS_STYLE,FILL_INFO)
% allows the user to specify any combination of all four inputs in any
% order after LINE_WIDTH
%
% Examples:
%      
%      mapx = 1;
%      mapy = 1;
%      low_pt = 1;
%      high_pt = 100;
%      [polytopes] = fcn_polytope_generation_halton_voronoi_tiling(low_pt,high_pt);
%      fig1 = fcn_plot_polytopes(polytopes,[],'-',2,[0.5 0 0]);
%      fig2 = fcn_plot_polytopes(polytopes,998,'-',2,[0 0 0.5],[0 mapx 0 mapy]);
%      fig3 = fcn_plot_polytopes(polytopes,999,'-',2,[0 0.5 0],[0 mapx 0 mapy],'square');
%      fig4 = fcn_plot_polytopes(polytopes,1000,'-',2,[0 0 0],[0 mapx 0 mapy],'square',[1 0 0 0 0.5]);
%      fig5 = fcn_plot_polytopes(polytopes([7 10 35]),123,'m--',2,[0 mapx 0 mapy],'square');
%      fig5 = fcn_plot_polytopes(polytopes([5 20 83]),fig5,'k-',2,[1 0.5 0.5 0.5 0.5]);
% 
% This function was written on 2018_12_10 by Seth Tau
% Added comments on 2021_02_23 by Seth Tau
% Removed old add path stuff and adjusted example on 2021_03_05 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

%% chec input arguments
if nargin < 4 || nargin > 8
    error('Incorrect number of input arguments')
end

%% open figures
if isempty(fig_num)
    fig = figure; % create new figure with next default index
else
    fig = figure(fig_num); % open specific figure
end
hold on % allow multiple plot calls

%% determine color and axis values
plots = 1; % basic plot with only polytopes, figure, line_spec, and line_width
color = []; axis_limits = []; axis_style = []; fill_info = [0 0 0 0 0]; % initialize empty values    
if nargin > 4 % variable arguments used
    for arg = 1:nargin-4 % check each variable argument
        argument = varargin{arg};
        arg_size = length(argument);
        if ischar(argument) % if the argument is a character string
            axis_style = argument; % axis style (ie square or equal)
        elseif arg_size == 3 % if the argument is a 3 value array
            color = argument; % color to plot polytopes
            plots = 2; % specific color plot
        elseif arg_size == 4 % if the argument is a 4 value array
            axis_limits = argument; % limits of x and y axes
        elseif arg_size == 5 % if the argument is a 5 value array
            fill_info = argument; % all the fill information [Y/N R G B alpha]
        else % if the argument does not fall within one of these categories
            warning('Invalid argument. Argument ignored.')
        end
    end
end
            

%% plot polytopes
if fill_info(1) == 1 % if fill is specified 
    for polys = 1:size(polytopes,2) % fill each polytope with the specified color and transparence
        filler = fill(polytopes(polys).vertices(:,1)',polytopes(polys).vertices(:,2)',fill_info(2:4));
        filler.FaceAlpha = fill_info(5);
    end
end
if plots == 1 % basic plot with only polytopes, figure, line_spec, and line_width
    for polys = 1:size(polytopes,2) % plot each polytope
        plot(polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2),line_spec,'linewidth',line_width)
    end
else % plot with the specific color
    for polys = 1:size(polytopes,2) % plot each polytope
        plot(polytopes(polys).vertices(:,1),polytopes(polys).vertices(:,2),line_spec,'Color',color,'linewidth',line_width)
    end
end

%% Change axis if specified
axis(axis_limits);
axis(axis_style);


