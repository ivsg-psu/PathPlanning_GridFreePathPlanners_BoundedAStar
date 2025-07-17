function [dist_sq,cross_sign] = fcn_BoundedAStar_calculatePointToLineDistSquared(pt,vert1,vert2)
% fcn_BoundAStar_calculatePointToLineDistSquared calculate distance to 
% point from a line between vert1 and vert2
%
% [DIST_SQ]=fcn_BoundAStar_calculatePointToLineDistSquared(PT,VERT1,VERT2)
% returns:
% DIST_SQ: a n-by-1 vector with the orthogonal distances to the line, where
%   n = number of points
%
% with inputs:
% PT: n-by-3 matrix of xyz coordinates of points around the line
% VERT1: 1-by-3 vector of xyz coordinates of one line end point
% VERT2: 1-by-3 vector of xyz coordinates of one line end point
%
% Examples:
%      
%      % Example
%      pt = [1 1 1; 10 10 0; 1 8 3];
%      vert1 = [0 0 0];
%      vert2 = [10 10 10];
%      dist_sq = fcn_point_to_line_distances_squared(pt,vert1,vert2)
% 
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
% 
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from
%    fcn_general_calculation_point_to_line_distances_squared to follow library
%    convention

%% check input arguments
if nargin ~= 3
    error('Incorrect number of arguments');
end

%% make sure vert1 and vert2 are 1-by-3
[row,col] = size(vert1);
if col == 1
    if row == 3 % sideways
        vert1 = vert1';
    elseif row == 2 % sideways with 2 coordinates
        vert1 = [vert1' 0];
    else
        error('vert1 should have 2 to 3 coordinates')
    end
elseif col == 2 
    if row == 1 % 2 coordinates
        vert1 = [vert1 0];
    else
        error('vert1 should have 2 to 3 coordinates')
    end
elseif col == 3
    if row > 1
        error('vert1 should have 2 to 3 coordinates')
    end
else
    error('vert1 should have 2 to 3 coordinates')
end

[row,col] = size(vert2);
if col == 1
    if row == 3 % sideways
        vert2 = vert2';
    elseif row == 2 % sideways with 2 coordinates
        vert2 = [vert2' 0];
    else
        error('vert2 should have 2 to 3 coordinates')
    end
elseif col == 2 
    if row == 1 % 2 coordinates
        vert2 = [vert2 0];
    else
        error('vert2 should have 2 to 3 coordinates')
    end
elseif col == 3
    if row > 1
        error('vert2 should have 2 to 3 coordinates')
    end
else
    error('vert2 should have 2 to 3 coordinates')
end

%% make sure pt is n-by-3
[row,col] = size(pt);
if col == 1
    if row == 3 % sideways
        pt = pt';
    elseif row == 2 % sideways with 2 coordinates
        pt = [pt' 0];
    else
        error('pt should have 2 to 3 coordinates per point')
    end
elseif col == 2 
    pt = [pt zeros(row,1)];
elseif col > 3
    error('pt should have 2 to 3 coordinates per point')
end

%% calculate the distance squared
vert1 = repmat(vert1,size(pt,1),1);
vert2 = repmat(vert2,size(pt,1),1);
ref_vec = vert2 - vert1;
end_vec = pt - vert1;
cross_pdct = cross(ref_vec,end_vec,2);
dist_sq = sum(cross_pdct.^2,2)./sum(ref_vec.^2,2);

cross_sign = sign(cross_pdct);

end