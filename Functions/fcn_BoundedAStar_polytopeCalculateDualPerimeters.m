function [perimeter1,perimeter2] = fcn_BoundedAStar_polytopeCalculateDualPerimeters(polytope,xing1,xing2,varargin) %,path1,path2]
% fcn_BoundedAStar_polytopeCalculateDualPerimeters calculate perimeters in both 
% directions around the polytope when extering at xing1 and exiting at
% xing2. When xing1 and xing2 are not directly across from e/o on a
% polytope, the numbers perimeter1 and perimeter2 will differ. This fcn
% helps determine if it is more efficient to go in or around the polytope
%
% FORMAT:
%
% [perimeter1, perimeter2]=fcn_BoundedAStar_polytopeCalculateDualPerimeters(polytope, xing1, xing2)
% OUTPUTS:
%
% perimeter1: the distance around the obstacle in direction 1
%
% perimeter2: the distance around the obstacle in direction 2
%
% INPUTS:
%
% polytope: a seven field structure of a polytope, where n = number of 
%   polytopes, with fields:
%       vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%       the number of the individual polytope vertices
%       xv: a 1-by-m vector of vertice x-coordinates
%       yv: a 1-by-m vector of vertice y-coordinates
%       distances: a 1-by-m vector of perimeter distances from one point to the
%       next point, distances(i) = distance from vertices(i) to vertices(i+1)
%       mean: average xy coordinate of the polytope
%       area: area of the polytope
%       max_radius: distance from mean to the furthest vertex
%
% xing1: first crossing location on the obstacle [x y]
%
% xing2: second crossing location on the obstacle [x y]
% 
% (optional inputs)
%
% fig_num: a figure number to plot results. If set to -1, skips any
% input checking or debugging, no figures will be generated, and sets
% up code to maximize speed. As well, if given, this forces the
% variable types to be displayed as output and as well makes the input
% check process verbose
%
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% DEPENDENCIES:
%
% fcn_MapGen_plotPolytopes
%
% REVISION HISTORY: 
% 2025_07_09 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
% 2025_07_17 - K. Hayes
% -- copied to new function from fcn_polytope_calculation_dual_perimeters to
%    follow library convention
% 2025_08_25 - K. Hayes
% -- updated fcn header and formatting
%
% TO DO:
% -- fix 'perimeter path' calculation

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 4; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS");
    MATLABFLAG_MAPGEN_FLAG_DO_DEBUG = getenv("MATLABFLAG_MAPGEN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
end

%% check input arguments?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if 0==flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(3,MAX_NARGIN);

        % Check the polytope input, make sure it is a struct
        assert(isstruct(polytope));

    end
end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1;
    end
end

%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%See: http://patorjk.com/software/taag/#p=display&f=Big&t=Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§


%% repeat the first values of vertices and distances if it does not close itself
if sum(polytope.vertices(1,:)==polytope.vertices(end,:)) ~= 2
    check.v = [polytope.vertices; polytope.vertices(1,:)];
    check.d = [polytope.distances; sum((check.v(end-1) - check.v(end)).^2,2).^0.5];
else
    check.v = polytope.vertices;
    check.d = polytope.distances;
end
check.p = sum(check.d);

%% check location of xing1 and xing2
% gap1 = fcn_find_gap_location(xing1,check.v);
% gap2 = fcn_find_gap_location(xing2,check.v);     
gap1 = fcn_BoundedAStar_polytopePointGapLocation(xing1,check.v);
gap2 = fcn_BoundedAStar_polytopePointGapLocation(xing2,check.v);

%% create perimeters & paths
[perimeter1,perimeter2] = fcn_create_perimeter_paths(check,xing1,xing2,gap1,gap2);
%,path1,path2]

        
%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

if flag_do_plots
   
    figure(fig_num)

    hold on
    box on
    
    plotFormat.LineWidth = 3;
    plotFormat.Color = 'black';
    plotFormat.DisplayName = 'Polytope';
    fcn_MapGen_plotPolytopes(polytope, plotFormat, [1 0 0 1 1],fig_num);
    % fcn_BoundedAStar_plotPolytopes(polytope,[],'k-',4,[-3 3 -3 3],'square');
    % plot(path1(:,1),path1(:,2),'g--','linewidth',2)
    % plot(path2(:,1),path2(:,2),'r--','linewidth',2)
    plot(xing1(1),xing1(2),'gx','linewidth',2,'DisplayName','Entry point');
    plot(xing2(1),xing2(2),'rx','linewidth',2, 'DisplayName', 'Exit point');

    legend

end % end function

end

%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§
        
function [perim1,perim2] = fcn_create_perimeter_paths(check,xing1,xing2,gap1,gap2) %,path1,path2]
% FCN_CREATE_PERIMETER_PATHS find the perimeter distance and paths
%
% [PERIM1,PERIM2,PATH1,PATH2]=FCN_CREATE_PERIMETER_PATHS(CHECK,XING1,XING2,GAP1,GAP2)
% returns:
% PERIM1: the distance around the obstacle in direction 1
% PERIM2: the distance around the obstacle in direction 2
% PATH1: m-by-2 matrix of xy points on the route in direction 1, where m is
%   the number of positions in direction 1
% PATH2: n-by-2 matrix of xy points on the route in direction 1, where n is
%   the number of positions in direction 1
%
% with inputs:
% CHECK: a 2 field structure with fields
%   v: vertices of the obstacle [x1 y1;...; xn yn]
%   d: distances between vertices [d1;...;dn]
% XING1: first crossing location on the obstacle [x y]
% XING2: second crossing location on the obstacle [x y]
% GAP1: gap location of xing1
% GAP2: gap location of xing2
%
% Examples:
%      
%      % Example
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\General_Calculation'])
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_calculation'])
%      polytope.xv=[2 1 -1 -2 -2 -1 1 2];
%      polytope.yv=[1 2 2 1 -1 -2 -2 -1];
%      polytope.vertices=[[polytope.xv polytope.xv(1)]' [polytope.yv polytope.yv(1)]'];
%      polytope.distances = fcn_general_calculation_euclidean_point_to_point_distance(polytope.vertices(1:end-1,:),polytope.vertices(2:end,:));
%      [Cx,Cy,polytope.area] = fcn_MapGen_polytopeCentroidAndArea ([[xv xv(1)]',[yv yv(1)]']);
%      polytope.mean = [Cx, Cy];
%      polytope.max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytope.vertices(1:end-1,:),ones(length(xv),1)*polytope.mean));
%      xing1 = [-2 0];
%      xing2 = [0 -2];
%      if sum(polytope.vertices(1,:)==polytope.vertices(end,:)) ~= 2
%          check.v = [polytope.vertices; polytope.vertices(1,:)];
%          check.d = [polytope.distances; fcn_general_calculation_euclidean_point_to_point_distance(check.v(end-1),check.v(end))];
%      else
%          check.v = polytope.vertices;
%          check.d = polytope.distances;
%      end
%      gap1 = fcn_find_gap_location(xing1,check.v);
%      gap2 = fcn_find_gap_location(xing2,check.v);
%      [perim1,perim2,path1,path2] = fcn_create_perimeter_paths(check,xing1,xing2,gap1,gap2)
%      fcn_BoundedAStar_plotPolytopes(polytope,[],'k-',4,[-3 3 -3 3],'square')
%      plot([xing1(1) xing2(1)],[xing1(2) xing2(2)],'kx','linewidth',2)
%      plot(path1(:,1),path1(:,2),'r--','linewidth',2)
%      plot(path2(:,1),path2(:,2),'g--','linewidth',2)
%      
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% REVISION HISTORY:
% 2025_07_09 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 

verts = size(check.v,1); % number of vertices
% find paths and perimeters
if gap1 < gap2
%     path1 = [xing1; check.v(gap1+1:gap2,:); xing2];
    verts1 = [xing1;check.v(gap2,:)];
    verts2 = [check.v(gap1+1,:);xing2];
    sums = sum(check.d(gap1+1:gap2-1));
    perim1 = sum(sum((verts1 - verts2).^2,2).^0.5) + sums;
    if gap2 == verts
%         path2 = [xing2; check.v(1:gap1,:); xing1];
%         perim2 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing2;check.v(gap1,:)],[check.v(1,:);xing1])) + sum(check.d(1:gap1-1));
        perim2 = check.p-perim1;
    else
%         path2 = [xing2; check.v(gap2+1:verts,:); check.v(1:gap1,:); xing1];
%         perim2 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing2;check.v(gap1,:)],[check.v(gap2+1,:);xing1])) + sum(check.d(gap2+1:end)) + sum(check.d(1:gap1-1));
        perim2 = check.p-perim1;
    end
elseif gap1 > gap2
%     path2 = [xing2; check.v(gap2+1:gap1,:); xing1];
    verts2 = [xing2;check.v(gap1,:)];
    verts1 = [check.v(gap2+1,:);xing1];
    sums = sum(check.d(gap2+1:gap1-1));
    perim2 = sum(sum((verts2 - verts1).^2,2).^0.5) + sums;
    if gap1 == verts
%         path1 = [xing1; check.v(1:gap2,:); xing2];
%         perim1 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing1;check.v(gap2,:)],[check.v(1,:);xing2])) + sum(check.d(1:gap2-1));
        perim1 = check.p-perim2;
    else
%         path1 = [xing1; check.v(gap1+1:verts,:); check.v(1:gap2,:); xing2];
%         perim1 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing1;check.v(gap2,:)],[check.v(gap1+1,:);xing2])) + sum(check.d(gap1+1:end)) + sum(check.d(1:gap2-1));
        perim1 = check.p-perim2;
    end
else % gap1 == gap2 % not sure this will actually happen
    gap_dist = sum(([xing1;xing2] - [check.v(gap1,:);check.v(gap1,:)]).^2,2).^0.5;
    if gap1 == verts
        if gap_dist(1) < gap_dist(2)
%             path2 = [xing2; check.v; xing1];
%             perim2 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing2;check.v(verts,:)],[check.v(1,:);xing1])) + sum(check.d);
%             path1 = [xing1; xing2];
            perim1 = sum((xing1 - xing2).^2,2).^0.5;
            perim2 = check.p-perim1;
        else
%             path1 = [xing1; check.v; xing2];
%             perim1 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing1;check.v(verts,:)],[check.v(1,:);xing2])) + sum(check.d);
%             path2 = [xing2; xing1];
            perim2 = sum((xing2 - xing1).^2,2).^0.5;
            perim1 = check.p-perim2;
        end
    else
        if gap_dist(1) < gap_dist(2)
%             path2 = [xing2; check.v(gap1+1:verts,:); check.v(1:gap1,:); xing1];
%             perim2 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing2;check.v(gap1,:)],[check.v(gap1+1,:);xing1])) + sum(check.d(gap1+1:end)) + sum(check.d(1:gap1-1));
%             path1 = [xing1; xing2];
            perim1 = sum((xing1 - xing2).^2,2).^0.5;
            perim2 = check.p-perim1;
        else
%             path1 = [xing1; check.v(gap1+1:verts,:); check.v(1:gap1,:); xing2];
%             perim1 = sum(fcn_general_calculation_euclidean_point_to_point_distance([xing1;check.v(gap1,:)],[check.v(gap1+1,:);xing2])) + sum(check.d(gap1+1:end)) + sum(check.d(1:gap1-1));
%             path2 = [xing2; xing1];
            perim2 = sum((xing2 - xing1).^2,2).^0.5;
            perim1 = check.p-perim2;
        end
    end
end
%path2 = flipud(path2); % always start paths from xing1

end

