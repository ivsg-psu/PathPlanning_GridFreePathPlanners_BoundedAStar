function [dist] = fcn_general_calculation_euclidean_point_to_point_distance(pt1,pt2)
% FCN_GENERAL_CALCULATION_EUCLIDEAN_POINT_TO_POINT_DISTANCE calculates the 
% distance(s) between pt1 and pt2 
%
% [DIST] = FCN_GENERAL_CALCULATION_EUCLIDEAN_POINT_TO_POINT_DISTANCE(PT1,PT2)
% returns:
% DIST: an n-by-1 vector of distances [d1; d2; ... ; dn], where n is the
% number of point sets
%
% with inputs:
% PT1: an n-by-2or3 series of xy or xyz points [x1 y1 z1; x2 y2 z2; ... ; xn yn zn]
% PT2: an n-by-2or3 series of xy or xyz points [x1 y1 z1; x2 y2 z2; ... ; xn yn zn]
%
% Example:
% pt1 = [1 1 5; 5 3 64; 7 2 -2];
% pt2 = [0 -3 -6; 34 1 17; 18 7 0];
% dist=fcn_general_calculation_euclidean_point_to_point_distance(pt1,pt2);
% 
% This function was written on 2018_11_17 by Seth Tau
% Added comments on 2021_02_23 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

%% check for input errors
[row1,col1] = size(pt1);
if col1 < 2 || col1 > 3 % too few or too many columns (this method is meant only for 2D or 3D geometry
    error('pts can only have 2 or 3 dimensions [x y z]')
elseif min([row1,col1]==size(pt2))==0 % different size inputs
    error('pt1 and pt2 must have the same number of points')
end

%% calculate dist using Pythagorean Theorem
if col1 == 2 % 2D geometry
    dist = sqrt((pt1(:,1)-pt2(:,1)).^2 + (pt1(:,2)-pt2(:,2)).^2);
else % 3D geometry
    dist = sqrt((pt1(:,1)-pt2(:,1)).^2 + (pt1(:,2)-pt2(:,2)).^2 + (pt1(:,3)-pt2(:,3)).^2);
end
