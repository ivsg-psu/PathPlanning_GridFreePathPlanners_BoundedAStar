function [path,cost,ERR,shadow_polytopes,vehicle] = fcn_algorithm_setup_speed_shadow_planning(base_polytopes,vehicle,A,B)
% This function repeatedly generates paths until the path angle is <=
% shadow uncertainty angle
%
% path: final path around polytopes
% cost: distance of path
% shadow_polytopes: final shadow_polytopes used
% vehicle: with modified dtheta
% 
% base_polytopes: polytopes without shadows
% vehicle: structure including gravity (g), friction (f), time delay (tau),
% speed (v), and angular uncertainty (dtheta)
% 
%% generate initial shadow map
shadow_polytopes = fcn_polytope_generation_speed_shadows(base_polytopes,vehicle);
% combine any overlapping shadows to remove any convex shapes
plan_polytopes = fcn_polytope_editing_combine_polytopes(shadow_polytopes);    

% %% check initial path
% [path,cost,ERR] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(plan_polytopes,A,B);
% 
% if size(path,1) == 2 % straight line
%     cost = sqrt((B.x-A.x)^2 + (B.y-A.y)^2);
%     ERR = 99;
% else % add slight angular uncertainty for consistency later on
%     vehicle.dtheta = 1e-6;
%     shadow_polytopes = fcn_polytope_generation_speed_shadows(base_polytopes,vehicle);
%     plan_polytopes = fcn_polytope_editing_combine_polytopes(shadow_polytopes);
% end
    

path_ang = NaN;
speed = sqrt(sum(vehicle.v.^2));
while ((path_ang>(vehicle.dtheta+1e-6))||(isnan(path_ang)))&&(~isinf(speed))
    if ~isnan(path_ang)
        vehicle.dtheta = path_ang;
    end

    %% Find paths through the polytopes
    [path,cost,ERR] = fcn_algorithm_setup_bound_Astar_for_tiled_polytopes(plan_polytopes,A,B);

%     %% Troubleshooting
%     fig = fcn_plot_polytopes(base_polytopes,[],'r',2);
%     fcn_plot_polytopes(shadow_polytopes,fig,'b',2);
%     fcn_plot_polytopes(plan_polytopes,fig,'g',2);
%     plot(path(:,1),path(:,2),'k-','linewidth',1)
%     title(num2str(vehicle.dtheta))


    if ERR~=0 
        cost = inf;
        break % break from path planning loop
    end

    %% Find next dtheta
    if size(path,1) == 2 % if straight line
        break
    else % not straight  line
        % max current path angle
        max_path_ang = max(abs(atan2d(path(2:end,2)-path(1:end-1,2),path(2:end,1)-path(1:end-1,1))));
        
        % increase obstacle size until path points disappear or path intersction
        d_ang = max_path_ang-vehicle.dtheta; % difference between max path ang and shadow angle uncertainty
        if d_ang > 0 
            adjustment = 1;
            n = 0;
            test_ang = vehicle.dtheta + d_ang/2^n;
%             if test_ang > 90
%                 disp('stop')
%             end
        else
            path_ang = max_path_ang;
        end
        test_vehicle = vehicle;
        while adjustment == 1
            % check if an obstacles have merged with dtheta = test_ang
            test_vehicle.dtheta = test_ang;
            shadow_polytopes = fcn_polytope_generation_speed_shadows(base_polytopes,test_vehicle);
            new_plan_polytopes = fcn_polytope_editing_combine_polytopes(shadow_polytopes);
            if size(new_plan_polytopes,2) == size(plan_polytopes,2) % no extra combined polytopes
                path_ang = test_ang;
                new_ang = 0;
                adjustment = 0;
            else
                n = n+1;
                test_ang = vehicle.dtheta + d_ang/2^n;
                if  n == 7 % taking this value no regardless, this allows this to step over intersection eventually
                    path_ang = test_ang;
                    adjustment = 0;
                end
            end
            
                
%             test_vehicle.dtheta = test_ang;
%             shadow_polytopes = fcn_polytope_generation_speed_shadows(base_polytopes,test_vehicle);
%             new_plan_polytopes = fcn_polytope_editing_combine_polytopes(shadow_polytopes);
%             if size(new_plan_polytopes,2) == size(plan_polytopes,2) % no extra combined polytopes
%                 % check for new_plan_polytopes intersecting modified path
%                 xv = [[new_plan_polytopes.xv] A.x B.x]';
%                 yv = [[new_plan_polytopes.yv] A.y B.y]';
%                 ind = path(:,3);
%                 new_path = [xv(ind) yv(ind) ind path(:,4:5)];
%                 good_pts = 0;
%                 for seg = 1:length(ind)-1
%                     start = new_path(seg,:);
%                     finish = new_path(seg+1,:);
%                     clear_pts = fcn_visibility_clear_and_blocked_points(new_plan_polytopes,start,finish);
%                     if isempty(clear_pts) % path is blocked
%                         new_ang = 1; % change test_ang
%                         break                        
%                     else
%                         good_pts = good_pts + 1;
%                     end
%                 end
%                 if good_pts == length(ind) % all clear, path is good for testing
%                     path_ang = test_ang;
%                     new_ang = 0;
%                     adjustment = 0;
%                 end
%             else % combined polytopes
%                 new_ang = 2;
%             end
%             
%             if new_ang > 0
%                 n = n+1;
%                 test_ang = vehicle.dtheta + d_ang/2^n;
%                 if  n == 7 % taking this value no regardless, this allows this to step over intersection eventually
%                     path_ang = test_ang;
%                     adjustment = 0;
%                 end
%             end
        end                 
        
        plan_polytopes = new_plan_polytopes;
        
    end
end