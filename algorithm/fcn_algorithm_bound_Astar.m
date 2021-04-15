function [cost,route] = fcn_algorithm_bound_Astar(start,finish,polytopes,all_pts,bound_pts,varargin)
% FCN_ALGORITHM_BOUND_ASTAR performs Astar path planning algorithm from
% start to finish around polytopes while constantly reducing boundaries
%
% [COST,ROUTE]=FCN_ALGORITHM_BOUND_ASTAR(START,FINISH,POLYTOPES,ALL_PTS,BOUND_PTS)
% returns:
% COST: distance along the shortest path
% ROUTE: m-by-5 matrix of route points with information for each point
% [x-coordinate, y-coordinate, point id, obstace id, 
%       begginning or end of an obstacle set (1 or 0)]
%
% with inputs:
% START: 1-by-5 vector with the same info as route for the starting point
% FINISH: same as start for the finish point
% POLYTOPES: a 1-by-n seven field structure of shrunken polytopes, 
% where n <= number of polytopes with fields:
%   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
%     the number of the individual polytope vertices
%   xv: a 1-by-m vector of vertice x-coordinates
%   yv: a 1-by-m vector of vertice y-coordinates
%   distances: a 1-by-m vector of perimeter distances from one point to the
%     next point, distances(i) = distance from vertices(i) to vertices(i+1)
%   mean: centroid xy coordinate of the polytope
%   area: area of the polytope
% ALL_PTS: p-by-5 matrix of all the points except start and finish
% BOUND_PTS: subset of valid points that are usable
%
% [COST,ROUTE]=FCN_ALGORITHM_BOUND_ASTAR(START,FINISH,POLYTOPES,ALL_PTS,BOUND_PTS,ELLIPSE_POLYTOPES)
% with input:
% ELLIPSE_POLYTOPES: polytopes used to determine the ellipse used for
%   bounding the points
%
% Examples:
%
%      Example 1:
%      cur_path = pwd;
%      main_folder = '!Voronoi Tiling Obstacles - Organized';
%      parent_dir = cur_path(1:strfind(cur_path,main_folder)-2);
%      addpath([parent_dir '\' main_folder '\Plotting'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_generation'])
%      addpath([parent_dir '\' main_folder '\Map_Generation\polytope_editing'])
%      addpath([parent_dir '\' main_folder '\Path_Planning\algorithm'])
%      polytopes=fcn_polytope_generation_halton_voronoi_tiling(1,100,[100,100]);
%      trim_polytopes=fcn_polytope_editing_remove_edge_polytopes(polytopes,0,100,0,100);
%      shrunk_polytopes=fcn_polytope_editing_shrink_evenly(trim_polytopes,2.5);
%
%      % HAVING ISSUES WITH TILED OBSTACLES
% %      % trim_polytopes path planning
% %      xv = [trim_polytopes.xv];
% %      yv = [trim_polytopes.yv];
% %      point_tot = length(xv);
% %      start = [0 50 point_tot+1 0 0];
% %      finish = [100 50 point_tot+2 -1 0];
% %      beg_end = zeros(point_tot,1);
% %      obs_id = zeros(point_tot,1);
% %      curpt = 0;
% %      for poly = 1:size(trim_polytopes,2) % check each polytope
% %          verts = length(trim_polytopes(poly).xv);
% %          obs_id(curpt+1:curpt+verts) = ones(verts,1)*poly; % obs_id is the same for every vertex on a single polytope
% %          beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
% %          curpt = curpt+verts;
% %      end
% %      all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
% %      bound_pts = all_pts;
% %      ellipse_polytopes = fcn_polytope_editing_tiling_loop_polytopes(trim_polytopes);
% %      [cost,route]=fcn_algorithm_bound_Astar(start,finish,trim_polytopes,all_pts,bound_pts,ellipse_polytopes);
% %      disp(['Path Cost: ' num2str(cost)])
% %      fcn_plot_polytopes(trim_polytopes,99,'b-',2,[0 100 0 100],'square');
% %      plot(route(:,1),route(:,2),'k-','linewidth',2)
% %      plot([start(1) finish(1)],[start(2) finish(2)],'kx','linewidth',2)
%
%      % shrunk_polytopes path planning
%      xv = [shrunk_polytopes.xv];
%      yv = [shrunk_polytopes.yv];
%      point_tot = length(xv);
%      start = [0 50 point_tot+1 0 0];
%      finish = [100 50 point_tot+2 -1 0];
%      beg_end = zeros(point_tot,1);
%      obs_id = zeros(point_tot,1);
%      curpt = 0;
%      for poly = 1:size(shrunk_polytopes,2) % check each polytope
%          verts = length(shrunk_polytopes(poly).xv);
%          obs_id(curpt+1:curpt+verts) = ones(verts,1)*poly; % obs_id is the same for every vertex on a single polytope
%          beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
%          curpt = curpt+verts;
%      end
%      all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
%      bound_pts = all_pts;
%      [cost,route]=fcn_algorithm_bound_Astar(start,finish,shrunk_polytopes,all_pts,bound_pts);
%      disp(['Path Cost: ' num2str(cost)])
%      fcn_plot_polytopes(shrunk_polytopes,100,'b-',2,[0 100 0 100],'square')
%      plot(route(:,1),route(:,2),'k-','linewidth',2)
%      plot([start(1) finish(1)],[start(2) finish(2)],'kx','linewidth',2)
%
%
% This function was written on 2020_02_05 by Seth Tau
% Questions or comments? sat5340@psu.edu 
%

% check variable argument
if nargin == 6
    ellipse_polytopes = varargin{1};
elseif nargin == 5
    ellipse_polytopes = polytopes;
else
    error('incorrect number of iputs')
end


% main code
cost = inf;
route = [];

closed_set = []; % no completed points
open_set = start(3); % point id
parent = ones(size(all_pts,1)+2)*NaN; % no known parents
parent(start(3)) = 0; % set start point with 0 as parent
cost_in = ones(size(parent))*Inf; % cost to reach point from start, all start at infinity
cost_in(start(3)) = 0; % cost from start to start is zero
% find straight line cost from each point to finish
heuristic_costs = fcn_general_calculation_euclidean_point_to_point_distance([all_pts(:,1:2); start(1:2); finish(1:2)],ones(size(all_pts,1)+2,1)*finish(1:2));
cost_tot = cost_in; % total cost, heuristic_cost is used as cost out until real cost found, start all at infinity
cost_tot(start(3)) = cost_in(start(3))+heuristic_costs(start(3)); % guess of cost to reach finish

bound_store(size(all_pts,1)+2).bound_pts = [];
bound_store(size(all_pts,1)+1).bound_pts = bound_pts;

while ~isempty(open_set) % continue until open set is empty
    open_cost_guesses = cost_tot(open_set); % find the costs guesses for the open set
    lowest_guess = min(open_cost_guesses); % find the lowest guess
    low_index = find(open_cost_guesses == lowest_guess); % find the index of the guess with the lowest cost
    % make the lowest cost point the current point
    if open_set(low_index) == start(3)
        cur_pt = start;
    elseif open_set(low_index) == finish(3)
        cur_pt = finish;
        cost = cost_tot(finish(3));
        route = finish;
        ind = parent(finish(3));
        while ind ~= start(3)
            route = [all_pts(ind,:); route];
            ind = parent(all_pts(ind,3));
        end
        route = [start; route];
        break
    else
        cur_pt = all_pts(open_set(low_index(1)),:);
        bound_pts = bound_store(parent(cur_pt(3))).bound_pts;
    end
   
    open_set(low_index(1)) = []; % remove cur_pt from open set
    closed_set = [closed_set; cur_pt(3)]; % add cur_pt to closed set
    
     %%% Step 1: Find intersections from start to finish and intersection points
     
    % find polytopes that could be intersected
    close_polytopes  = fcn_polytope_calculation_polytopes_near_the_line(cur_pt,finish,ellipse_polytopes);
    %%%%%%%%%%%%% polytopes changed to ellipse_polytopes
    
    % find if finish is blocked and any intersection data
    if ~isempty(close_polytopes) % if there's close obstacles
        [~,blocked_pts,D,di,~,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP] = fcn_visibility_clear_and_blocked_points(close_polytopes,cur_pt,finish);
        xings = fcn_visibility_line_polytope_intersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,close_polytopes);
%         blocked_pts = finish;
    else % no close obstacles
        blocked_pts = []; % no blocked points
    end

    % Check which case this is (intersection or not)
    if isempty(blocked_pts) % CASE 1: No intersections
        tentative_cost = cost_in(cur_pt(3)) + fcn_general_calculation_euclidean_point_to_point_distance(cur_pt(1:2),finish(1:2)); % cost to reach current + cost to reach finish
        
        if isempty(find(open_set==finish(3),1)) % not already in open set
            open_set = [open_set; finish(3)];
            parent(finish(3)) = cur_pt(3);
            cost_in(finish(3)) = tentative_cost;
            cost_tot(finish(3)) = tentative_cost+heuristic_costs(finish(3));
        elseif tentative_cost < cost_in(finish(3))
            parent(finish(3)) = cur_pt(3);
            cost_in(finish(3)) = tentative_cost;
            cost_tot(finish(3)) = tentative_cost+heuristic_costs(finish(3));
        end
         
    else % CASE 2: Intersections occur
        
        %%% Step 2: Find intersected polytopes and calculate the sum of minimum perimeters around the polytopes and intersection to intersection distances
        xing_polytopes = close_polytopes(xings(end).obstacles); % polytopes crossed between start and finish
        max_dist = fcn_bounding_ellipse_min_perimeter_path(xing_polytopes,xings,cur_pt,finish);

        %%% Step 3: Bound points with the same or less cost based on triangle inequality (bounding box based on ellipse major & minor axes)
        straight = fcn_general_calculation_euclidean_point_to_point_distance(cur_pt(1:2),finish(1:2)); % straight distance start to finish
        perp_offset = ones(1,2)*sqrt((max_dist/2)^2 - (straight/2)^2); % split large triangle from start to mid point to end in half and calculate height
        para_offset = ones(1,2)*(max_dist - straight)/2; % extra distance traveled if moved directly away from the end point and then directly to it
%         pts = [[close_polytopes.xv]' [close_polytopes.yv]'; cur_pt(1:2); finish(1:2)];
%         [dist_sq,cross_sign] = fcn_general_calculation_point_to_line_distances_squared(pts,cur_pt(1:2),finish(1:2));
%         signed_dist_sq = dist_sq.*cross_sign(:,3);
%         perp_offset = [sqrt(abs(min(signed_dist_sq))) sqrt(abs(max(signed_dist_sq)))];
%         
%         ref_vec = ones(size(pts,1),1)*(finish(1:2) - cur_pt(1:2));
%         para_dists = (dot(ref_vec,(pts-cur_pt(1:2)),2))/straight;
%         para_offset = [abs(min(para_dists)) abs(max(para_dists)-straight)];
        
        if min(para_offset)==0 % sometimes happens when the pependicular offset reaches the computing limit, but isn't actually zero
            % make it look like CASE 1
            tentative_cost = cost_in(cur_pt(3)) + fcn_general_calculation_euclidean_point_to_point_distance(cur_pt(1:2),finish(1:2)); % cost to reach current + cost to reach finish
        
            if isempty(find(open_set==finish(3),1)) % not already in open set
                open_set = [open_set; finish(3)];
                parent(finish(3)) = cur_pt(3);
                cost_in(finish(3)) = tentative_cost;
                cost_tot(finish(3)) = tentative_cost+heuristic_costs(finish(3));
            elseif tentative_cost < cost_in(finish(3))
                parent(finish(3)) = cur_pt(3);
                cost_in(finish(3)) = tentative_cost;
                cost_tot(finish(3)) = tentative_cost+heuristic_costs(finish(3));
            end
            neighbor_pts = []; % we don't need any neighbor points, but the variable must be set
            
        else % there is a non-zero offset
            % find bounding box with bound points and polytopes
            [bound_polytopes,bound_box,bound_pts,~] = fcn_bounding_ellipse_polytope_bounding_box(cur_pt(1:2),finish(1:2),polytopes,all_pts,bound_pts,perp_offset,para_offset);
            bound_pts = bound_pts(bound_pts(:,3) ~= cur_pt(3),:); % remove current point if necessary
            bound_store(cur_pt(3)).bound_pts = bound_pts;
%             %% plot for troubleshooting
%             fig = fcn_plot_polytopes(polytopes,[],'b-',2);
%             fcn_plot_polytopes(close_polytopes,fig,'g-',2);
%             plot([bound_box(:,1); bound_box(1,1)],[bound_box(:,2); bound_box(1,2)],'k--','linewidth',2)
%             plot(cur_pt(1),cur_pt(2),'kx','linewidth',2)
%             plot(finish(1),finish(2),'kx','linewidth',2)
        
            %%% Step 4: For all bound points, find points visible to cur_pt and calculate their costs
            neighbor_pts = fcn_visibility_clear_and_blocked_points(bound_polytopes,cur_pt,[bound_pts; finish]);
        end
        
        %%% testing
%         test_pts = [bound_pts; finish];
%         
%         
%         xtest = test_pts(:,1) - cur_pt(1);
%         ytest = test_pts(:,2) - cur_pt(2);
%         [theta,rho] = cart2pol(xtest,ytest);
%         vis_test = [theta rho test_pts(:,3:end)];
%         [min_rho,irho] = min(rho); % start finding the closest point
%         vis_pt = nan(size(test_pts));
%         vis_pt(1,:) = test_pts(irho,:);
%         min_theta = theta(irho);
%         while something...
%             
%         
%         end
%         vis_pt(isnan(vis_pt(:,1)),:) = []; % remove extra entries
        %%%%%%
        
        if ~isempty(neighbor_pts) % bound and visible points from this point exist
            for neighbor = neighbor_pts(:,3)'
                if isempty(find(closed_set==neighbor,1)) % not already in the closed set
                    tentative_cost = cost_in(cur_pt(3)) + fcn_general_calculation_euclidean_point_to_point_distance(cur_pt(1:2),all_pts(neighbor,1:2)); % cost to reach current + cost to reach neighbor
                    
                    if isempty(find(open_set==neighbor,1)) % not already in the open set
                        open_set = [open_set; neighbor]; % put into open set
                        parent(neighbor) = cur_pt(3); % put cur_pt as parent
                        cost_in(neighbor) = tentative_cost; % use tentative_cost as the cost in
                        cost_tot(neighbor) = tentative_cost+heuristic_costs(neighbor); % add heuristic to cost_in
                    elseif tentative_cost < cost_in(neighbor) % in open set but this cost is lower
                        parent(neighbor) = cur_pt(3);
                        cost_in(neighbor) = tentative_cost;
                        cost_tot(neighbor) = tentative_cost+heuristic_costs(neighbor);
                    end
                    
                end
            end
        end
                         
    end

end