function [gap] = fcn_BoundedAStar_polytopePointGapLocation(point,vertices)
% fcn_BoundedAStar_polytopePointGapLocation find which gap the point is in
%
% [GAP]=fcn_BoundedAStar_polytopePointGapLocation(CROSS,VERTICES)
% returns:
% GAP: the gap the point of interest is between, 
%   ex: gap = 1 means vert1 <= point < vert2
%       gap = 4 means vert4 <= point < vert5
%
% with inputs:
% CROSS: crossing location on the obstacle [x y]
% VERTICES: vertices of the obstacle [x1 y1;...; xn+1 yn+1], where n is the
%   number of vertices in an obstacle
%
% Examples:
%      
%      % BASIC example
%      vertices = [2 1; 1 2; -1 2; -2 1; -2 -1; -1 -2; 1 -2; 2 -1; 2 1];
%      cross1 = [-2 0];
%      gap1 = fcn_BoundedAStar_polytopePointGapLocation(cross1,vertices)
%      cross2 = [2 0];
%      gap2 = fcn_BoundedAStar_polytopePointGapLocation(cross2,vertices)
%      cross3 = [-2 0.75];
%      gap3 = fcn_BoundedAStar_polytopePointGapLocation(cross3,vertices)
%      cross4 = [2 1];
%      gap4 = fcn_BoundedAStar_polytopePointGapLocation(cross4,vertices)
%      figure
%      plot(vertices(:,1),vertices(:,2),'k-','linewidth',1)
%      hold on
%      plot(cross1(1),cross1(2),'bx','linewidth',1)
%      plot(cross2(1),cross2(2),'rx','linewidth',1)
%      plot(cross3(1),cross3(2),'gx','linewidth',1)
%      plot(cross4(1),cross4(2),'mx','linewidth',1)
%      axis([-3 3 -3 3])
%      axis square
% 
% This function was written on 2018_12_21 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from
%    fcn_polytope_calculation_point_gap_location to follow library convention

% xy-vertices and crossing point
x1 = vertices(1:end-1,1);
y1 = vertices(1:end-1,2);
x2 = vertices(2:end,1);
y2 = vertices(2:end,2);
xi = point(1);
yi = point(2);

% check if the point is on the line between two points
acc = 1e-8;
TF = fcn_BoundedAStar_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc); % row vector of 1 or 0 if on lines between (x1,y1) and (x2,y2)
gap = find(TF,1);

% ensure gap is defined
while isempty(gap)
    acc = acc*10; % decrease accuracy to account for changes in variable types
    TF = fcn_BoundedAStar_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc); % row vector of 1 or 0 if on lines between (x1,y1) and (x2,y2)
    gap = find(TF,1);
    if acc > 100 % if point too far stop searching
        error('Intersecting point does not appear to be on the line')
    end
end
if acc > 1e-8 % warn if the point is too far away
    warning('Gap assignment may be incorrect, accuracy decreased to %.0e',acc)
end

end