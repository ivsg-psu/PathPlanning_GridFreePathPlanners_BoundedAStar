function [polytopes,fig]=fcn_Map_Generation_map_name_to_map(map_name,plot_flag,disp_name,varargin)
% FCN_MAP_GENERATION_MAP_NAME_TO_MAP script to generate map based on 
% map_name which specifies map characteristics
% 
% [POLYTOPES,fig]=FCN_MAP_GENERATION_MAP_NAME_TO_MAP(MAP_NAME,PLOT_FLAG,FIG_NUM,LINE_SPEC,LINE_WIDTH,COLOR,AXIS_LIMITS,AXIS_STYLE,FILL_INFO)
% returns: 
% POLYTOPES: a 1-by-n seven field structure, where n <= number of 
%   the map polytopes with fields:
%   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%     the number of the individual polytope vertices
%   xv: a 1-by-m vector of vertice x-coordinates
%   yv: a 1-by-m vector of vertice y-coordinates
%   distances: a 1-by-m vector of perimeter distances from one point to the
%     next point, distances(i) = distance from vertices(i) to vertices(i+1)
%   mean: centroid xy coordinate of the polytope
%   area: area of the polytope
%   max_radius: distance from the mean to the farthest vertex
% FIG: variable containing the figure number of the plot if plot_flag is 1
% 
% with inputs:
% MAP_NAME: string with map characteristics numbers seperated by indicating 
%   letters
% PLOT_FLAG: variable determining whether the map should be plotted (1=Yes)
% DISP_NAME: variable determining whether the name is displayed on the plot
%   (1=Yes), where the name goes (x,y), what color ([r g b]), and the font
%   size (e.g. 12) 
%   Example [Yes, origin, red, size 12] = [1, 0 0, 1 0 0, 12]
%   If not desired set the first value to anything but 1 and the rest will
%   be ignored
%   Example [No, NA, NA, NA] = [0, 1 2, 3 4 5, 6], or 0, or 999  
% FIG_NUM: figure number to plot the values on
% LINE_SPEC: a string, line specifications such as color, 'r', and line or
% point type, '--'
% LINE_WIDTH: width of the line to be plotted
% COLOR: a 1-by-3 vector to specify the RGB plot colors [Red Green Blue],
% where 0 <= Red,Green,Blue <= 1
% AXIS_LIMTS: a 1-by-4 vector to specify the start and end of the x and y
% axes, [xstart xend ystart yend]
% AXIS_STYLE: controls the style of the axis, such as square or equal
% FILL_INFO: a 1-by-5 vector to specify wether or not there is fill, the
% color of fill, and the opacity of the fill [Y/N, R, G, B, alpha]
% 
% %% List of Map Characteristics:
% %%% Generation:
%       HST: Halton set tiling
%           requires: minimum value and maximum value of Halton set
%           example: HST 1 1000
% %%% Trimming
%       SQT: Square trimming 
%           requires: low x value, high x value, low y value, high y value
%           example: SQT 0 1 0 1
% %%% Shrinking
%       SMV: Shrink to mean radius with specified variance
%           requires: mean radius, standard deviation of radius, minimum
%           radius to shrink to, shrink seed
%           example: SMV 0.02 0.005 1e-6 1234
% 
% Basic Example:
%   map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.01 0.001 1e-6 1111"; 
%   plot_flag = 1; disp_name = 0; fig_num = []; line_style = 'r-'; 
%   line_width = 2;
%   [polytopes,fig]=fcn_Map_Generation_map_name_to_map(map_name,plot_flag,disp_name,fig_num,line_style,line_width);
%   
% Advanced Example
%   map_name = "HST 30 450 SQT 0 1 0 1 SMV 0.02 0.005 1e-6 1234";
%   plot_flag = 1; disp_name = [1, 0.05 -0.05, 0.5 0.5 0.5, 10]; 
%   fig_num = 999; line_style = '-'; line_width = 2; color = [0 0 1]; 
%   axis_limits = [0 1 -0.1 1]; axis_style = 'square'; 
%   fill_info = [1 1 0 1 0.5];
%   [polytopes,fig]=fcn_Map_Generation_map_name_to_map(map_name,plot_flag,disp_name,fig_num,line_style,line_width,color,axis_limits,axis_style,fill_info);
% 
% This function was written on 2020_07_02 by Seth Tau
% Comments added on 2021_02_23 by Seth Tau
% Questions or comments? sat5340@psu.edu 
% 

%% Input Checking
%  not enough   plotting but not enough   too many
if (nargin<3)||((nargin>3)&&(nargin<6))||(nargin>10)
    error('Incorrect number of arguments.')
end

if ~isstring(map_name) % input must be a string
    if ischar(map_name) % convert to string if a character
        map_name=convertCharsToStrings(map_name);
    else
        error('map_name must be a string.')
    end
end

%% Split map_name
split_name = split(map_name); % split name string at each space

%% Base Map Generation
if sum(split_name=="HST")>0 % Check for Halton set tiling (HST) 
    HST_index = find(split_name=="HST"); % index of the HST string
    if length(HST_index)>1 % more than one instance of HST
        error('HST is repeated in map_name.')
    end
    % generate base map based on the values following HST
    base_polytopes = fcn_polytope_generation_halton_voronoi_tiling(str2double(split_name(HST_index+1)),str2double(split_name(HST_index+2)));
% elseif % check for other generating methods
    % generate using other method
else % no map generation method specified
    error('No map generation method specified.')
end

%% Trim map if necessary
if sum(split_name=="SQT")>0 % check for square triming (SQT)
    SQT_index = find(split_name=="SQT"); % index of the SQT string
    if length(SQT_index)>1 % more than one instance of SQT
        error('SQT is repeated in map_name')
    end
    % trim the base polytopes based on the values following SQT
    trim_polytopes = fcn_polytope_editing_remove_edge_polytopes(base_polytopes,str2double(split_name(SQT_index+1)),str2double(split_name(SQT_index+2)),str2double(split_name(SQT_index+3)),str2double(split_name(SQT_index+4)));
% elseif % check for other trim methods
    % trim polytopes
else % no trim method specified
    trim_polytopes = base_polytopes; % no trimming so trim_polytopes is the same
end

%% Shrink map if necessary
if sum(split_name=="SMV")>0 % check for shrinking to mean and variance (SMV)
    SMV_index = find(split_name=="SMV"); % index of the SMV string
    if length(SMV_index)>1 % more than 1 instance of SMV
        error('SMV is repeated in map_name')
    end
    % shrink based on the values following SMV
    rng(str2double(split_name(SMV_index+4))) % set the rng to the shrink seed
    polytopes = fcn_polytope_editing_shrink_to_average_max_radius_with_variance(trim_polytopes,str2double(split_name(SMV_index+1)),str2double(split_name(SMV_index+2)),str2double(split_name(SMV_index+3)));
% elseif % check for other shrink methods
    % trim polytopes
else % no trim method specified
    polytopes = trim_polytopes; % no shrinking so polytopes is the same
end


%% Plot map if plot_flag == 1
if plot_flag == 1
    if length(varargin)>3 % basic plotting
        [fig] = fcn_plot_polytopes(polytopes,varargin{1},varargin{2},varargin{3},varargin{4:end});
    else % extra inputs to plot function
        [fig] = fcn_plot_polytopes(polytopes,varargin{1},varargin{2},varargin{3});
    end
    if disp_name(1) == 1 % add map_name to plot
        text(disp_name(2),disp_name(3),map_name,'color',disp_name(4:6),'FontSize',disp_name(7));
    end
else % do not plot
    fig = []; % set value empty to return
end