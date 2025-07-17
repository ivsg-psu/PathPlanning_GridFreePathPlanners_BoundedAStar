function [TF] = fcn_BoundedAStar_calculatePointsOnLines(x1,y1,x2,y2,xi,yi,acc)
% fcn_BoundedAStar_calculatePointsOnLines determines whether the points are
% on lines between point set 1 and point set 2
%
% [TF]=fcn_BoundedAStar_calculatePointsOnLines(X1,Y1,X2,Y2,XI,YI,ACC)
% returns:
% TF: n-by-1 vector of True(1) or False(0) values determining whether the
% point is on the line between the corresponding points in [X1 Y1] and
% [X2 Y2]
%
% with inputs:
% X1: x values for the polytope
% Y1: y values for the polytope
% X2: x values from X1 with the first value moved to the end
% Y2: y values from Y1 with the first value moved to the end
% XI: the x values for the points of interest
% YI: the y values for the points of interest
% ACC: variable for determining how close to the line the point of interest
% must be to be considered "on" the line (accounts for calculation
% rounding)
%
% Examples:
%      
%      x1 = [-2 -1 1 2 2 1 -1 -2];
%      y1 = [-1 -2 -2 -1 1 2 2 1];
%      x2 = [-1 1 2 2 1 -1 -2 -2];
%      y2 = [-2 -2 -1 1 2 2 1 -1];
%      acc = 1e-8;
%      figure
%      plot([x1 x1(1)],[y1 y1(1)],'k-','linewidth',1)
%      hold on
%      xi = 2;
%      yi = 0;
%      TF1 = fcn_general_calculation_points_on_lines(x1,y1,x2,y2,xi,yi,acc)
%      plot(xi,yi,'bx','linewidth',1)
%      xi = 2+1e-8;
%      yi = 0;
%      TF2 = fcn_general_calculation_points_on_lines(x1,y1,x2,y2,xi,yi,acc)
%      plot(xi,yi,'rx','linewidth',1)
%      xi = 2+2e-8;
%      yi = 0;
%      TF3 = fcn_general_calculation_points_on_lines(x1,y1,x2,y2,xi,yi,acc)
%      plot(xi,yi,'gx','linewidth',1)
%      xi = -2;
%      yi = -1;
%      TF4 = fcn_general_calculation_points_on_lines(x1,y1,x2,y2,xi,yi,acc)
%      plot(xi,yi,'mx','linewidth',1)
%      axis([-3 3 -3 3])
%      axis square
%      
%
% 
% This function was written on 2018_11_28 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- copied to new function from fcn_general_calculation_points_on_lines
%    to follow library convention


% rows of TF correspond to points in xi,yi
% columns of TF correspond to lines composed from x1,y1,x2,y2

x1s = size(x1);
x2s = size(x2);
y1s = size(y1);
y2s = size(y2);
xis = size(xi);
yis = size(yi);
if (sum(x1s==x2s)+sum(y1s==y2s)+sum(x1s==y1s)) ~= 6
    error('x1,y1,x2,y2 must all have the same dimensions')
end
if sum(xis==yis) ~= 2
    error('xi and yi must have the same dimensions')
end

row = xis(1);
col = xis(2);
if col ~= 1 % 0 or more than 1 column
    if row ~= 1 % xi and yi are matrices or empty
        error('xi and yi must be vectors')
    else % need to transpose xi and yi
        xi = xi';
        yi = yi';
        row = col;
    end
else % 1 column vector
    if row == 0
        error('xi and yi are empty vectors')
    end
end
col = length(x1);

tf = zeros(row,col); % numerical true or false matrix
dx = x2-x1;
dy = y2-y1;
for val = 1:length(dx) % check each dx dy value
    if abs(dx(val)) <= acc
%         tf(:,val) = (abs(xi - x1(val))<acc).*((((y1(val)-yi)<acc).*((yi-y2(val))<acc))+(((y2(val)-yi)<acc).*((yi-y1(val))<acc)));
        tf(:,val) = (abs(xi - x1(val))<acc).*(sign(dy(val)*(yi-y1(val))) + sign(dy(val)*(y2(val)-yi)) > 0);
    elseif abs(dy(val)) <= acc
%         tf(:,val) = (abs(yi - y1(val))<acc).*((((x1(val)-xi)<acc).*((xi-x2(val))<acc))+(((x2(val)-xi)<acc).*((xi-x1(val))<acc)));
        tf(:,val) = (abs(yi - y1(val))<acc).*(sign(dx(val)*(xi-x1(val))) + sign(dx(val)*(x2(val)-xi)) > 0);
    else
%         tf(:,val) = (abs(yi - ((dy(val)/dx(val)).*(xi-x1(val)) + y1(val)))<acc).*((((x1(val)-xi)<acc).*((xi-x2(val))<acc))+(((x2(val)-xi)<acc).*((xi-x1(val))<acc))); 
        tf(:,val) = (abs(yi - ((dy(val)/dx(val)).*(xi-x1(val)) + y1(val)))<acc).*(sign(dx(val)*(xi-x1(val))) + sign(dx(val)*(x2(val)-xi)) > 0); 
    end
end

TF = tf==1; % logical true or false matrix
