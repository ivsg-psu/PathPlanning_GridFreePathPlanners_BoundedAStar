function [polytopes] = fcn_polytope_generation_halton_voronoi_tiling(low_pt,high_pt,varargin)
% FCN_POLYTOPE_GENERATION_HALTON_VORONOI_TILING generate a map with
% obstacles perfectly tiled together using the Voronoi cells generated from
% the Halton sequence
% 
% [POLYTOPES]=FCN_POLYTOPE_GENERATION_HALTON_VORONOI_TILING(LOW_PT,HIGH_PT)
% returns:
% POLYTOPES: a 1-by-n seven field structure, where n <= number of polytopes
%   with fields:
%   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%     the number of the individual polytope vertices
%   xv: a 1-by-m vector of vertice x-coordinates
%   yv: a 1-by-m vector of vertice y-coordinates
%   distances: a 1-by-m vector of perimeter distances from one point to the
%     next point, distances(i) = distance from vertices(i) to vertices(i+1)
%   mean: centroid xy coordinate of the polytope
%   area: area of the polytope
% 
% with inputs:
% LOW_PT: the lowest point index to use in the halton sequence
% HIGH_PT: the highest point index to use in the halton sequence
% 
% [POLYTOPES]=FCN_POLYTOPE_GENERATION_HALTON_VORONOI_TILING(LOW_PT,HIGH_PT,STRETCH)
% with input:
% STRETCH: [x,y] scaling factors to allow the values from the halton set to
%   be scaled to fit maps shapes other than square  
% 
% Basic Example:
%   polytopes = fcn_polytope_generation_halton_voronoi_tiling(1,1000);
%   fcn_plot_polytopes(polytopes,[],'b',2,[0 1 0 1]);
% 
% Stretch Example:
%   stretch = [1000, 500] % stretch in the x and y directions
%   polytopes = fcn_polytope_generation_halton_voronoi_tiling(1,1000,stretch);
%   fcn_plot_polytopes(polytopes,[],'b',2,[0 1000 -250 750],'square');
% 
% This function was written on 2019_06_13 by Seth Tau
% Example modification to remove addpath 2020_07_16 by Seth Tau
% Added comments on 2021_02_23 by Seth Tau
% Questions or comments? sat5340@psu.edu 
% 

%% check variable argument
if nargin == 3 % if stretch is specified
    stretch = varargin{1};
else % no stretch
    stretch = [1 1];
end

%% pull halton set
p = haltonset(2);  % pull a 2 dimensional halton sequence of points
ps = scramble(p,'RR2'); % scramble values

%% pick values from halton set
X = ps(low_pt:high_pt,:);
X = X.*stretch; % stretch the values if specified

%% generate Voronoi diagram based on the Halton set
[V,C] = voronoin(X);

%% create tiling
num_poly = size(X,1); % number of polytopes created is equal to the number of Halton set points
polytopes(num_poly) = struct('vertices',[],'xv',[],'yv',[],'distances',[],'mean',[],'area',[],'max_radius',[]);

%     %%%%%%%%%% trouble shooting
%     figure
%     hold on
%     %%%%%%%%%%%%%%%%%%%%%%%%%%

remove = 0; % keep track of how many cells needed removed for having 2 or fewer points
for poly = 1:num_poly % pull each cell from the voronoi diagram
    % x and y values from this cell
    xv = V(C{poly},1)'; 
    yv = V(C{poly},2)';

    % change infinite values to be finite
    change = (xv>(2*stretch(1)))+(xv<(-1*stretch(1)))+(yv>(2*stretch(2)))+(yv<(-1*stretch(2))); % find values more that extend beyond the strectch value
    xv(change>0) = (2*stretch(1))*sign(mean(xv(change==0))); % set value to twice stretch (removed in trim polytopes function)
    yv(change>0) = (2*stretch(2))*sign(mean(yv(change==0))); % set value to twice stretch (removed in trim polytopes function)
    
    if length(xv)>2 % at least 3 points in cell
        % make sure points are clockwise
        vec1 = [xv(2)-xv(1),yv(2)-yv(1),0]; % vector leading into point
        vec2 = [xv(3)-xv(2),yv(3)-yv(2),0]; % vector leading out of point
        xing = cross(vec1,vec2); % cross product of two vectors
        if sign(xing(3)) == -1 % points ordered in wrong direction
            % flip point order
            xv = fliplr(xv);
            yv = fliplr(yv);
        end
        % enter info into polytope structure
        polytopes(poly-remove).xv = xv;
        polytopes(poly-remove).yv = yv;
        polytopes(poly-remove).vertices = [[xv xv(1)]' [yv yv(1)]']; % repeat first vertice for easy plotting
        [Cx,Cy,polytopes(poly-remove).area] = fcn_polytope_calculation_centroid_and_area([xv xv(1)]',[yv yv(1)]');
        polytopes(poly-remove).mean = [Cx, Cy]; % enter polytope centroid
        % calculate perimeter distances around the polytope
        polytopes(poly-remove).distances = fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly-remove).vertices(1:end-1,:),polytopes(poly-remove).vertices(2:end,:));
        % calculate the maximum distance from center to a vertex
        polytopes(poly-remove).max_radius = max(fcn_general_calculation_euclidean_point_to_point_distance(polytopes(poly-remove).vertices(1:end-1,:),ones(length(xv),1)*polytopes(poly-remove).mean));

%         %%%%%%%%%% trouble shooting
%         plot([xv xv(1)],[yv yv(1)],'linewidth',2)
%         axis([0 1 0 1])
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%

    else % if 2 or less points in cell 
        remove = remove+1; % skip cell and remove later
    end
end

%% remove extra empty polytopes from structure
for polys = 1:remove
    polytopes(num_poly+1-remove) = [];
end