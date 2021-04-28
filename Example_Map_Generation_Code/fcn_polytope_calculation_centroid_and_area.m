function [Cx,Cy,Area] = fcn_polytope_calculation_centroid_and_area(x,y)
% FCN_POLYTOPE_CALCULATION_CENTROID_AND_AREA calculates the centroid and area of a closed 
% polytope
%
% [CX,CY,AREA]=FCN_POLYTOPE_CALCULATION_CENTROID_AND_AREA(X,Y)
% returns:
% CX: the centroid x coordinate
% CY: the centroid y coordinate
% AREA: the unsigned area enclosed by the polytope
%
% with inputs:
% X: x coordinates of the polytope (with the same first and last point)
% Y: y coordinates of the polytope (with the same first and last point)
% The function outputs:
%
% Example:
% x = [3; 4; 2; -1; -2; -3; -4; -2; 1; 2; 3];
% y = [1; 2; 2; 3; 2; -1; -2; -3; -3; -2; 1];
% [Cx,Cy,Area] = fcn_polytope_calculation_centroid_and_area(x,y)
% plot(x,y,'g-','linewidth',2)
% hold on
% plot(Cx,Cy,'kx','linewidth',1)
%
% This function was written on 2019_04_02 by Seth Tau
% Added comments on 2021_02_23 by Seth Tau
% Removed old add path stuff on 2021_03_02 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Centroid & Area equations https://en.wikipedia.org/wiki/Centroid


% % troubleshooting
% figure
% plot(x,y,'b-','linewidth',2)

% current points
xi = x(1:end-1); 
yi = y(1:end-1);
% next points
xip1 = x(2:end); 
yip1 = y(2:end);

A = sum(xi.*yip1 - xip1.*yi)/2; % signed area
Cx = sum((xi+xip1).*(xi.*yip1 - xip1.*yi))/(6*A); % centroid x coordinate
Cy = sum((yi+yip1).*(xi.*yip1 - xip1.*yi))/(6*A); % centroid x coordinate

Area = abs(A); % unsigned area

% % troubleshooting
% hold on
% plot(Cx,Cy,'gx','linewidth',2)