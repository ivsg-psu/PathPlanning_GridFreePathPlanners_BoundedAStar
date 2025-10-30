function [perimeter1,perimeter2] = fcn_polytope_calculation_dual_perimeters(polytope,xing1,xing2) %,path1,path2]
warning('on','backtrace');
warning(['fcn_polytope_calculation_dual_perimeters is being deprecated. ' ...
    'Use fcn_BoundedAStar_polytopeCalculateDualPerimeters instead.']);
% FCN_POLYTOPE_CALCULATION_DUAL_PERIMETERS calculate perimeters in both 
% directions around the polytope
%
% [PERIMETER1,PERIMETER2]=FCN_POLYTOPE_CALCULATION_DUAL_PERIMETERS(POLYTOPE,XING1,XING2)
% returns:
% PERIMETER1: the distance around the obstacle in direction 1
% PERIMETER2: the distance around the obstacle in direction 2
%
% with inputs:
% POLYTOPE: a seven field structure of a polytope, where n = number of 
%   polytopes, with fields:
% vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%   the number of the individual polytope vertices
% xv: a 1-by-m vector of vertice x-coordinates
% yv: a 1-by-m vector of vertice y-coordinates
% distances: a 1-by-m vector of perimeter distances from one point to the
%   next point, distances(i) = distance from vertices(i) to vertices(i+1)
% mean: average xy coordinate of the polytope
% area: area of the polytope
% max_radius: distance from mean to the furthest vertex
% XING1: first crossing location on the obstacle [x y]
% XING2: second crossing location on the obstacle [x y]
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
%      polytope.distances = sum((polytope.vertices(1:end-1,:) - polytope.vertices(2:end,:)).^2,2).^0.5;
%      [Cx,Cy,polytope.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
%      polytope.mean = [Cx, Cy];
%      polytope.max_radius = max(sum((polytope.vertices(1:end-1) - ones(length(xv),1)*polytope.mean).^2,2).^0.5);
%      xing1 = [-2 0];
%      xing2 = [2 0];
%      [perimeter1,perimeter2]=fcn_polytope_calculation_dual_perimeters(polytope,xing1,xing2)
%      fcn_plot_polytopes(polytope,[],'k-',4,[-3 3 -3 3],'square')
%      plot(path1(:,1),path1(:,2),'g--','linewidth',2)
%      plot(path2(:,1),path2(:,2),'r--','linewidth',2)
%      plot(xing1(1),xing1(2),'yx','linewidth',2)
%      plot(xing2(1),xing2(2),'cx','linewidth',2)
%      xing1 = [-2 0.75];
%      xing2 = [-2 -0.75];
%      [perimeter1,perimeter2]=fcn_polytope_calculation_dual_perimeters(polytope,xing1,xing2)
%      fcn_plot_polytopes(polytope,[],'k-',4,[-3 3 -3 3],'square')
%      plot(path1(:,1),path1(:,2),'g--','linewidth',2)
%      plot(path2(:,1),path2(:,2),'r--','linewidth',2)
%      plot(xing1(1),xing1(2),'yx','linewidth',2)
%      plot(xing2(1),xing2(2),'cx','linewidth',2)
%      xing1 = [-2 0];
%      xing2 = [2 1];
%      [perimeter1,perimeter2]=fcn_polytope_calculation_dual_perimeters(polytope,xing1,xing2)
%      fcn_plot_polytopes(polytope,[],'k-',4,[-3 3 -3 3],'square')
%      plot(path1(:,1),path1(:,2),'g--','linewidth',2)
%      plot(path2(:,1),path2(:,2),'r--','linewidth',2)
%      plot(xing1(1),xing1(2),'yx','linewidth',2)
%      plot(xing2(1),xing2(2),'cx','linewidth',2)
%
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% REVISION HISTORY: 
% 2025_07_09 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 


%% check input arguments
if nargin ~= 3
    error('Incorrect number of arguments');
end

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
gap1 = fcn_polytope_calculation_point_gap_location(xing1,check.v);
gap2 = fcn_polytope_calculation_point_gap_location(xing2,check.v);

%% create perimeters & paths
[perimeter1,perimeter2] = fcn_create_perimeter_paths(check,xing1,xing2,gap1,gap2);
%,path1,path2]
end

        
%% supporting functions

% function [gap] = fcn_find_gap_location(xing,vertices)
% % FCN_FIND_GAP_LOCATION find which gap the point is between
% %
% % [GAP]=FCN_FIND_GAP_LOCATION(CROSS,VERTICES)
% % returns:
% % GAP: the gap the point of interest is between, 
% %   ex: gap = 1 means vert1 <= point < vert2
% %       gap = 4 means vert4 <= point < vert5
% %
% % with inputs:
% % CROSS: crossing location on the obstacle [x y]
% % VERTICES: vertices of the obstacle [x1 y1;...; xn+1 yn+1], where n is the
% %   number of vertices in an obstacle
% %
% % Examples:
% %      
% %      % BASIC example
% %      vertices = [2 1; 1 2; -1 2; -2 1; -2 -1; -1 -2; 1 -2; 2 -1; 2 1];
% %      cross1 = [-2 0];
% %      gap1 = fcn_find_gap_location(cross1,vertices)
% %      cross2 = [2 0];
% %      gap2 = fcn_find_gap_location(cross2,vertices)
% %      cross3 = [-2 0.75];
% %      gap3 = fcn_find_gap_location(cross3,vertices)
% %      cross4 = [2 1];
% %      gap4 = fcn_find_gap_location(cross4,vertices)
% %      figure
% %      plot(vertices(:,1),vertices(:,2),'k-','linewidth',1)
% %      hold on
% %      plot(cross1(1),cross1(2),'bx','linewidth',1)
% %      plot(cross2(1),cross2(2),'rx','linewidth',1)
% %      plot(cross3(1),cross3(2),'gx','linewidth',1)
% %      plot(cross4(1),cross4(2),'mx','linewidth',1)
% %      axis([-3 3 -3 3])
% %      axis square
% % 
% % This function was written on 2018_12_21 by Seth Tau
% % Questions or comments? sat5340@psu.edu 
% %
% 
% % xy-vertices and crossing point
% x1 = vertices(1:end-1,1);
% y1 = vertices(1:end-1,2);
% x2 = vertices(2:end,1);
% y2 = vertices(2:end,2);
% xi = xing(1);
% yi = xing(2);
% 
% % check if the point is on the line between two points
% acc = 1e-8;
% TF = fcn_general_calculation_points_on_lines(x1,y1,x2,y2,xi,yi,acc); % row vector of 1 or 0 if on lines between (x1,y1) and (x2,y2)
% gap = find(TF,1);
% 
% % ensure gap is defined
% while isempty(gap)
%     acc = acc*10; % decrease accuracy to account for changes in variable types
%     TF = fcn_general_calculation_points_on_lines(x1,y1,x2,y2,xi,yi,acc); % row vector of 1 or 0 if on lines between (x1,y1) and (x2,y2)
%     gap = find(TF,1);
%     if acc > 100 % if point too far stop searching
%         error('Intersecting point does not appear to be on the line')
%     end
% end
% if acc > 1e-8 % warn if the point is too far away
%     warning('Gap assignment may be incorrect, accuracy decreased to %.0e',acc)
% end
% 
% end
        
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
%      [Cx,Cy,polytope.area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)],[yv yv(1)]);
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
%      fcn_plot_polytopes(polytope,[],'k-',4,[-3 3 -3 3],'square')
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

