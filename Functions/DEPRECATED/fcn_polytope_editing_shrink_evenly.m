function [shrunk_polytopes,point_polys] = fcn_polytope_editing_shrink_evenly(polytopes,dist)

warning(['fcn_polytope_editing_shrink_evenly is being deprecated.' ...
    ' Use fcn_MapGen_polytopesShrinkEvenly instead.']);

% FCN_POLYTOPE_EDITING_SHRINK_EVENLY shrinks each polytope in poltyopes 
% specified evenly on all sides by dist
%
% [SHRUNK_POLYTOPES,POINT_POLYS]=FCN_POLYTOPE_EDITING_SHRINK_EVENLY(POLYTOPES,DIST)
% returns:
% SHRUNK_POLYTOPES: a 1-by-n seven field structure of shrunken polytopes, 
% where n <= number of polytopes with fields:
%   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%     the number of the individual polytope vertices
%   xv: a 1-by-m vector of vertice x-coordinates
%   yv: a 1-by-m vector of vertice y-coordinates
%   distances: a 1-by-m vector of perimeter distances from one point to the
%     next point, distances(i) = distance from vertices(i) to vertices(i+1)
%   mean: centroid xy coordinate of the polytope
%   area: area of the polytope
% POINT_POLYS: indices of polytopes that have been shrunk to a single point
%
% with inputs:
% POLYTOPES: the original polytopes with the same fields as
% shrunk_polytopes
% DIST: distance to shrink each polytope by on each side
%
% Examples:
%   cur_path = pwd;
%   main_folder = '!Voronoi Tiling Obstacles - Organized';
%   parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%   addpath([parent_dir '\' main_folder '\Plotting'])
%   addpath([parent_dir '\' main_folder '\Map_Generation\polytope_generation'])
%   addpath([parent_dir '\' main_folder '\Map_Generation\polytope_editing'])
%   polytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 1000],[],[],-1);
%   fig = fcn_plot_polytopes(polytopes,[],'b',2,[0 1 0 1]);
%   shrunk_polytopes = fcn_polytope_editing_shrink_evenly(polytopes,0.01);
%   fcn_plot_polytopes(shrunk_polytopes,fig,'g',2,[0 1 0 1]);
%
% This function was written on 2019_06_13 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%
% Revision History:
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% -- Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method
% -- Added cost field to shrunk polytopes to fix bug in
%    script_Path_Planning_testing

%% Check input arguments
if nargin ~= 2
    error('Incorrect number of arguments');
end

%% shrink the polytopes
num_poly = size(polytopes,2); % number of polytopes
% structure for the polytopes
shrunk_polytopes(num_poly) = struct('vertices',[],'xv',[],'yv',[],'distances',[],'mean',[],'area',[],'max_radius',[], 'cost', []);
point_polys = []; % variable to store indices of single point polytopes
for poly = 1:size(polytopes,2) % shrink each polytope
    shrinkable = 0; % assume the obstacle can't be shrunk by the amount until shown otherwise
    rad = polytopes(poly).max_radius; % starting radius of the polytope
    
    % find shrink angles
    vertices = polytopes(poly).vertices;
    % find vectors into and out of each point
    dx_B = vertices(2:end,1)-vertices(1:end-1,1);
    dy_B = vertices(2:end,2)-vertices(1:end-1,2);
    dx_A = -[dx_B(end); dx_B(1:end-1)];
    dy_A = -[dy_B(end); dy_B(1:end-1)];
    angs = atan2(dy_B,dx_B); % angle of outgoing vectors
    angs(angs<0) = angs(angs<0) + 2*pi; % ensure angles are positive
    A = [dx_A dy_A zeros(length(dx_A),1)]';
    B = [dx_B dy_B zeros(length(dx_B),1)]';
    mag_A = vecnorm(A); % maginitued of the vectors
    mag_B = vecnorm(B);
    theta  = acos(dot(A,B)./(mag_A.*mag_B)); % angle between vectors
    shrink_angs = angs + theta'/2; % angle to move the point
    dist_angs = abs((angs - pi/2) - (shrink_angs - pi)); % used to calculate distance to move the point
    dists = rad./cos(dist_angs); % distance needed to move each point

    xv = vertices(1:end-1,1); % polytope vertices
    yv = vertices(1:end-1,2);
    tot_dist = 0;
    for combos = 1:size(vertices,1)-1-2 % vertices has one repeat point and then we want to leave 3 points
        xp = xv + dists.*cos(shrink_angs); % projection of where points need to go
        yp = yv + dists.*sin(shrink_angs);
        % check if this is feasible by determine if any shrinking vectors
        % overlap (not feasible if this happens)
        dxi = xp-xv; dyi = yp-yv;
        dxj = dxi'; dyj = dyi';
        den = dxj.*dyi - dxi.*dyj;
        % di shows if and where two lines intersect
        di = (dxj.*(yv'-yv) - dyj.*(xv'-xv))./den; % only need ones directly off diagonal and top right/bottom left
        di_size = length(di);
        % mod_mat will be used to pull data for lines we care about
        mod_mat = zeros(di_size);
        mod_mat(1:di_size-1,2:di_size) = eye(di_size-1);
        mod_mat(1,di_size) = 1;
        di_mod = di.*mod_mat; % modified di matrix
        min_di = min(min(di_mod(di_mod>0))); % smallest value of di
%         try
            [p1,p2] = find((di_mod<min_di+1e-15).*(di_mod>0)); % find lines that intersect (this is where points merge)
%         catch
%             hold on
%         end
        
        if tot_dist + min_di*dists(p1(1))*cos(dist_angs(p1(1))) < dist % not able to shrink enough yet
            tot_dist = tot_dist + min_di*dists(p1(1))*cos(dist_angs(p1(1))); % amount that can be shrunk
            if length(xv)>3 % still have points to remove
                % combine 2 points into 1
                remove = max(p1,p2);
                xv = xv + min_di*(xp-xv); % replace one value with the new value
                yv = yv + min_di*(yp-yv);
                xv(remove) = [];
                yv(remove) = [];                
                if length(xv) < 3 % if we removed too many points
                    Cx = mean(xv); % use the centroid as the location for the point polytope
                    Cy = mean(yv);
                    break
                end
                % create new vectors
                dx_B = [xv(2:end); xv(1)] - xv;
                dy_B = [yv(2:end); yv(1)] - yv;
                dx_A = -[dx_B(end); dx_B(1:end-1)];
                dy_A = -[dy_B(end); dy_B(1:end-1)];
                angs = atan2(dy_B,dx_B);
                angs(angs<0) = angs(angs<0) + 2*pi;
                A = [dx_A dy_A zeros(length(dx_A),1)]';
                B = [dx_B dy_B zeros(length(dx_B),1)]';
                mag_A = vecnorm(A);
                mag_B = vecnorm(B);
                theta  = acos(dot(A,B)./(mag_A.*mag_B));
                shrink_angs = angs + theta'/2; % angle to move the point
                dist_angs = abs((angs - pi/2) - (shrink_angs - pi)); % used to calculate distance to move the point
                dists = (rad-tot_dist)./cos(dist_angs); % distance to move each point
            else % no more points can be removed remove
                [Cx,Cy] = fcn_polytope_calculation_centroid_and_area([xv; xv(1)],[yv; yv(1)]);
            end 
        else % able to shrink enough
            % shrink by the appropriate distance
            dists = (dist-tot_dist)./cos(dist_angs);
            xv = xv + dists.*cos(shrink_angs); % final points
            yv = yv + dists.*sin(shrink_angs);
            shrinkable = 1; % the obstacle is fully shrinkable
            break
        end
    
    end
    if shrinkable == 1 % if the obstacle was shrunk add it to shrunk_polytopes
        shrunk_polytopes(poly).xv = xv'; % keep vertices seperate for easier calculations
        shrunk_polytopes(poly).yv = yv';
        shrunk_polytopes(poly).vertices = [[xv; xv(1)] [yv; yv(1)]]; % repeat first vertice for easy plotting
        [centroid,shrunk_polytopes(poly).area] = fcn_polytope_calculation_centroid_and_area([xv; xv(1)],[yv; yv(1)]);
        Cx = centroid(1,1);
        Cy = centroid(1,2);
        shrunk_polytopes(poly).mean = [Cx, Cy]; % calculate the polytope mean
        % calculate perimeter distances around the polytope
        shrunk_polytopes(poly).distances = sum((shrunk_polytopes(poly).vertices(1:end-1,:) - shrunk_polytopes(poly).vertices(2:end,:)).^2,2).^0.5;
        % calculate the maximum distance from center to a vertex
        shrunk_polytopes(poly).max_radius = max(sum((shrunk_polytopes(poly).vertices(1:end-1,:) - ones(length(xv),1)*shrunk_polytopes(poly).mean).^2,2).^0.5);
        % add cost field back onto polytope after shrinking
        shrunk_polytopes(poly).cost = polytopes(poly).cost;
    else % if it was not shrinkable, make it a point polytope in shrunk_polytopes
        shrunk_polytopes(poly).xv = [Cx Cx Cx]; % keep vertices seperate for easier calculations
        shrunk_polytopes(poly).yv = [Cy Cy Cy];
        shrunk_polytopes(poly).vertices = [[Cx Cx Cx]' [Cy Cy Cy]']; % repeat first vertice for easy plotting
        shrunk_polytopes(poly).mean = [Cx, Cy]; % calculate the polytope mean
        shrunk_polytopes(poly).area = 0;
        % calculate perimeter distances around the polytope
        shrunk_polytopes(poly).distances = zeros(3,1);
        % calculate the maximum distance from center to a vertex
        shrunk_polytopes(poly).max_radius = 0;
        % add cost field back onto polytope after shrinking
        shrunk_polytopes(poly).cost = polytopes(poly).cost
        point_polys = [point_polys; poly]; % add its index to the point_polys
    end
end
    


