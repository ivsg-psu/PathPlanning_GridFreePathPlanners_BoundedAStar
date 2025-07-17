function [cost,route] = fcn_BoundedAStar_AstarBounded(start,finish,polytopes,all_pts,bound_pts,planner_mode,varargin)
% fcn_BoundedAStar_AStarBounded performs Astar path planning algorithm from
% start to finish around polytopes while constantly reducing boundaries
%
% [COST,ROUTE]=fcn_BoundedAStar_AstarBounded(START,FINISH,POLYTOPES,ALL_PTS,BOUND_PTS)
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
% PLANNER_MODE: string containing option for planner behavior
% indicates the planner mode
% "legacy" only goes around obstacles
% "through at vertices" allows the planner to go through or around each obstacle
% but only entering or exiting at vertices
% "through or around" allows the planner to go through all obstacles or around all
% "straight through" the planner only goes straight from the start to the goal, calculating the cost
%
% [COST,ROUTE]=fcn_BoundedAStar_AstarBounded(START,FINISH,POLYTOPES,ALL_PTS,BOUND_PTS,ELLIPSE_POLYTOPES)
% with input:
% ELLIPSE_POLYTOPES: polytopes used to determine the ellipse used for
%   bounding the points
%
% Examples:
%
%      addpath([pwd '\Example_Map_Generation_Code'])
%
%      map_name = "HST 1 100 SQT 0 1 0 1 SMV 0.04 0.008 1e-6 1111";
%      plot_flag = 0; disp_name = 0; fig_num = 654654;
%      [polytopes,fig]=fcn_MapGen_nameToMap(map_name,plot_flag,disp_name);
%
%      % shrunk_polytopes path planning
%      xv = [polytopes.xv];
%      yv = [polytopes.yv];
%      point_tot = length(xv);
%      start = [0 0.5 point_tot+1 0 0];
%      finish = [1 0.5 point_tot+2 -1 0];
%      beg_end = zeros(point_tot,1);
%      obs_id = zeros(point_tot,1);
%      curpt = 0;
%      for poly = 1:size(polytopes,2) % check each polytope
%          verts = length(polytopes(poly).xv);
%          obs_id(curpt+1:curpt+verts) = ones(verts,1)*poly; % obs_id is the same for every vertex on a single polytope
%          beg_end([curpt+1,curpt+verts]) = 1; % the first and last vertices are marked with 1 and all others are 0
%          curpt = curpt+verts;
%      end
%      all_pts = [xv' yv' [1:length(xv)]' obs_id beg_end];
%      bound_pts = all_pts;
%      [cost,route]=fcn_BoundedAStar_AstarBounded(start,finish,polytopes,all_pts,bound_pts);
%      disp(['Path Cost: ' num2str(cost)])
%      fcn_BoundedAStar_plotPolytopes(polytopes,100,'b-',2,[0 1 0 1],'square')
%      plot(route(:,1),route(:,2),'k-','linewidth',2)
%      plot([start(1) finish(1)],[start(2) finish(2)],'kx','linewidth',2)
%      box on
%      xlabel('X Position')
%      ylabel('Y Position')
%
%
% This function was written on 2020_02_05 by Seth Tau
% Cleaned code for Git and rewrote example on 2021_04_28 by Seth Tau
% Questions or comments? sat5340@psu.edu
%
% Revision history
% 2025_07_17 - K. Hayes, kxh1031@psu.edu
% -- created renamed version of function to match convention from
%    fcn_algorithm_bound_Astar.m

% check variable argument
if nargin == 7
    ellipse_polytopes = varargin{1};
elseif nargin ==6
    ellipse_polytopes = polytopes;
else
    error('incorrect number of inputs')
end


% main code
cost = inf;
route = [];

if planner_mode == "straight through"
    [through_cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = fcn_BoundedAStar_straightPlanner(start,finish,all_pts,polytopes);
    cost = through_cost;
    route = [start; finish];
    return
end

closed_set = []; % no completed points
open_set = start(3); % point id
parent = ones(size(all_pts,1)+2)*NaN; % no known parents
parent(start(3)) = 0; % set start point with 0 as parent
cost_in = ones(size(parent))*Inf; % cost to reach point from start, all start at infinity
cost_in(start(3)) = 0; % cost from start to start is zero
% find straight line cost from each point to finish using Euclidean
% distance
heuristic_costs = sum((ones(size(all_pts,1)+2,1)*finish(1:2) - [all_pts(:,1:2); start(1:2); finish(1:2)]).^2,2).^0.5;
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
    close_polytopes  = fcn_BoundedAStar_polytopesNearLine(cur_pt,finish,ellipse_polytopes);
    %%%%%%%%%%%%% polytopes changed to ellipse_polytopes
    % TODO @sjharnett put boundary call here instead of in while loop
    % rebrand in new function called fast bounded A*
    % find if finish is blocked and any intersection data
    if ~isempty(close_polytopes) % if there's close obstacles
        [~,blocked_pts,D,di,~,num_int,xiP,yiP,xiQ,yiQ,xjP,yjP] = fcn_Visibility_clearAndBlockedPoints(close_polytopes,cur_pt,finish);
        xings = fcn_Visibility_linePolytopeIntersections(xiP,yiP,xiQ,yiQ,xjP,yjP,D,di,num_int,close_polytopes);
    else % no close obstacles
        blocked_pts = []; % no blocked points
    end

    % Check which case this is (intersection or not)
    if isempty(blocked_pts) % CASE 1: No intersections
        tentative_cost = cost_in(cur_pt(3)) + sum((cur_pt(1:2) - finish(1:2)).^2,2).^0.5;

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
        % max_dist = norm(finish(1:2)-start(1:2))*2;
        max_dist = fcn_BoundedAStar_calculateBoundingEllipseMinPerimPath(xing_polytopes,xings,cur_pt,finish);

        %%% Step 3: Bound points with the same or less cost based on triangle inequality (bounding box based on ellipse major & minor axes)
        straight = sum((cur_pt(1:2) - finish(1:2)).^2,2).^0.5;  % straight distance start to finish
        perp_offset = ones(1,2)*sqrt((max_dist/2)^2 - (straight/2)^2); % split large triangle from start to mid point to end in half and calculate height
        para_offset = ones(1,2)*(max_dist - straight)/2; % extra distance traveled if moved directly away from the end point and then directly to it

        if min(para_offset)==0 % sometimes happens when the pependicular offset reaches the computing limit, but isn't actually zero
            % make it look like CASE 1
            tentative_cost = cost_in(cur_pt(3)) + sum((cur_pt(1:2) - finish(1:2)).^2,2).^0.5; % cost to reach current + cost to reach finish

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
            [bound_polytopes,bound_box,bound_pts,~] = fcn_BoundedAStar_calculateBoundingEllipsePolytopeBoundingBox(cur_pt(1:2),finish(1:2),polytopes,all_pts,bound_pts,perp_offset,para_offset);
            bound_pts = bound_pts(bound_pts(:,3) ~= cur_pt(3),:); % remove current point if necessary
            bound_store(cur_pt(3)).bound_pts = bound_pts;
            % %% plot for troubleshooting
            % fig = fcn_BoundedAStar_plotPolytopes(polytopes,[],'b-',2);
            % fcn_BoundedAStar_plotPolytopes(close_polytopes,fig,'g-',2);
            % plot([bound_box(:,1); bound_box(1,1)],[bound_box(:,2); bound_box(1,2)],'k--','linewidth',2)
            % plot(cur_pt(1),cur_pt(2),'kx','linewidth',2)
            % plot(finish(1),finish(2),'kx','linewidth',2)

            %%% Step 4: For all bound points, find points visible to cur_pt and calculate their costs
            % TODO @sjharnett calculate visibility matrix once at beginning then simply remove
            % vertices outisde boundary and squeeze() matrix to remove empty rows & cols
            neighbor_pts = fcn_Visibility_clearAndBlockedPoints(bound_polytopes,cur_pt,[bound_pts; finish]);
        end
        if planner_mode == "through at vertices"
            [cur_obs_id, self_blocked_cost, pts_blocked_by_self] = ...
                fcn_Visibility_selfBlockedPoints(polytopes,cur_pt,all_pts);
            % Steve's hack: if we are inside a polytope, the neighbor points don't have a weight of 0,
            % they have a weight of the cost of the polytope we're in
            cost_of_poly_inside_of = 0;
            for obs = 1:size(polytopes,2)
                [in,on] = inpolygon(cur_pt(1),cur_pt(2),polytopes(obs).vertices(:,1),polytopes(obs).vertices(:,2));
                if in && ~ on
                    cost_of_poly_inside_of = polytopes(obs).cost;
                    break
                end
            end
            % append a cost column of 0 to neighbor points in free space
            neighbor_pts = [neighbor_pts, ones(size(neighbor_pts,1),1).*cost_of_poly_inside_of];
            % append a cost column of the polytope's cost to points blocked by current polytope
            pts_blocked_by_self = ...
                [pts_blocked_by_self, ones(size(pts_blocked_by_self,1),1).*self_blocked_cost];
            % merge self blocked points and free space points since a distinction between them is...
            % no longer necessary
            neighbor_pts = [neighbor_pts; pts_blocked_by_self];
        end
        % pts_blocked_by_self_nx5 = [pts_blocked_by_self, nan, cur_obs_id, zeros, self_blocked_cost];
        % neighbor_pts = [neighbor_pts, zeros(length(neighbor_pts,1))];
        % neighbor_pts = [neighbor_pts; pts_blocked_by_self_nx5];
        % figure
        % hold on
        % plot([5,1],[5,1])
        % plot([5,1],[1,5])
        % [dist,loc,wall] = fcn_MapGen_findIntersectionOfSegments([1,1],[5,5],[5,1],[1,5])
        % TODO(@sjharnett): the following is pseudo code for allowing the planner
        % to route through polytopes anywhere (not only at vertices)
        % the neighbor should be not the next point on a polytope, but the next
        % point on a blocking polytope, passing through traversable polytopes
        % therefore, between the current point and the next point, we need to see how many polytope
        % sides are crossed and note the crossing points and polytopes crossed
        % then the distance between each pair of crossing points is scaled by the cost of that polytope
        % inputs: current point, next visible point, all polytopes
        % for each next visible point:
            % for each poly:
                % for each side: does the line from cur_point to next viz cross the poly side?
                % if so, Is there already a noted crossing?
                    % if so
                        % stop checking sides
                        % calculate distance from new crossing to old one
                        % scale that distance by polytope cost
                        %
                    % if not
                        % note the point of crossing
                        % keep searching
        % for each polytope in between, check each side for crossing.  If a side has a crossing, check
        % other sides until a second crossing is found
        % find distance between those crossings
        if ~isempty(neighbor_pts) % bound and visible points from this point exist
            poly_cost_index = 1;
            for neighbor = neighbor_pts(:,3)'
                if isempty(find(closed_set==neighbor,1)) % not already in the closed set
                    % cost to reach current + cost to reach neighbor scaled by cost of traversing
                    % that distance (which is only >1 for points that require crossing a polytope)
                    % need to scale cost based on cost of polytopes traversed (neightbor_pts(i,6))
                    if planner_mode == "legacy" || planner_mode == "through or around"
                        tentative_cost = cost_in(cur_pt(3)) + sum((cur_pt(1:2) - all_pts(neighbor,1:2)).^2,2).^0.5; % cost to reach current + cost to reach neighbor
                    elseif planner_mode == "through at vertices"
                        tentative_cost = cost_in(cur_pt(3)) + ...
                            (1+neighbor_pts(poly_cost_index,6))*sum((cur_pt(1:2) - all_pts(neighbor,1:2)).^2,2).^0.5;

                        %% experimental hill model - scales cost if going one direction,
                        % but not in other directions
                        % find the path so far
                        hill_penalty_on_flag = 0;
                        if hill_penalty_on_flag
                            if cur_pt(3) ~= start(3)
                                next_pt_x = all_pts(neighbor,1);
                                cur_pt_x = cur_pt(1);
                                next_pt_y = all_pts(neighbor,2);
                                cur_pt_y  = cur_pt(2);
                                if cur_pt_y > next_pt_y % we go right
                                    hill_penalty = 1.2;
                                else
                                    hill_penalty = 1;
                                end
                                tentative_cost = tentative_cost*hill_penalty;
                            end
                        end

                        %% experimental curveyness penalty
                        % applies a scaling to the cost based on portion of
                        % maximum possible turn between segments (180 deg)
                        curve_penalty_on_flag = 0;
                        if curve_penalty_on_flag
                            if cur_pt(3) ~= start(3)
                                % find the path so far
                                route_so_far = cur_pt;
                                ind_so_far = parent(cur_pt(3));
                                while ind_so_far ~= start(3)
                                    route_so_far = [all_pts(ind_so_far,:); route_so_far];
                                    ind_so_far = parent(all_pts(ind_so_far,3));
                                end
                                route_so_far = [start; route_so_far];

                                % calculate angle between previous segment and
                                % next possible segment

                                % next_seg = [next_pt_x - cur_pt_x, next_pt_y - cur_pt_y]
                                next_seg = [all_pts(neighbor,1)-cur_pt(1),all_pts(neighbor,2)-cur_pt(2)];
                                % prev_seg = [cur_pt_x - prev_pt_x, cur_pt_y - prev_pt_y]
                                prev_seg = [cur_pt(1)-route_so_far(end-1,1),cur_pt(2)-route_so_far(end-1,2)];

                                ang = acos(dot(next_seg,prev_seg)/(norm(next_seg)*norm(prev_seg)));

                                % express curveneyness as portion of max turn (180)
                                curveyness_portion = ang/pi;
                                % scale cost by max curveyness
                                tentative_cost = tentative_cost*(1+curveyness_portion);
                            end
                        end
                    end

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
                poly_cost_index = poly_cost_index + 1;
            end
        end

    end
    if planner_mode == "through or around"
        [through_cost,distance_in_polys,distance_outside_polys,num_polys_traversed] = fcn_BoundedAStar_straightPlanner(start,finish,all_pts,polytopes);
        x = route(:,1);
        y = route(:,2);
        d = diff([x(:) y(:)]);
        total_route_length = sum(sqrt(sum(d.*d,2)));
        if through_cost <= total_route_length
            cost = through_cost;
            route = [start; finish];
        end
    end
end