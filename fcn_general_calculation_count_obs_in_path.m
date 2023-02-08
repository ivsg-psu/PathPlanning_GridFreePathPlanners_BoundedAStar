function [obs_around, obs_through] = ...
    fcn_general_calculation_count_obs_in_path(path)
    % fcn_general_calculation_count_obs_in_path
    % counts the number of unique obstacles encountered in a path
    % tallying obstacle routed through separately from obstacles routed around
    % for use wtih a planner that routes through or around obstacles, only at vertices
    %
    % FORMAT:
    %
    % [obs_around, obs_through] = ...
    %     fcn_general_calculation_count_obs_in_path(path)
    %
    % INPUTS:
    %
    % PATH: p-by-5 matrix of all the points in the path
    %
    % OUTPUTS:
    %
    % obs_around - the integer number obstalces that were routed around by the planner
    % obs_through - the integer number of obstacles routed through by the planner
    %
    % DEPENDENCIES:
    %
    % EXAMPLES:
    %
    % For additional examples, see: script_planning_performed_at_multiple_costs.m
    %
    % This function was written in 2022_05 by Steve Harentt
    % Questions or comments? sjh6473@psu.edu
    %

    % Revision History:

    % TO DO

    % extract obstacle IDs from points data
    obs_ids = path(:,4);
    % the start point must have obs_id of -1 (i.e. it's not on an obstacle)
    assert(obs_ids(1)==-1)
    % initialize counters
    obs_around = 0;
    obs_through = 0;
    obs_id_idx = 2;
    while obs_id_idx < length(obs_ids)
        % if the obstacle ID doesn't increment, we are on the same obstacle, i.e.
        % we went through from vertex to vertex
        if obs_ids(obs_id_idx) == obs_ids(obs_id_idx+1)
            obs_through = obs_through + 1;
            obs_id_idx = obs_id_idx + 2;
        % if the obstacle ID increments, we are on a new obstacle adn therefore
        % went around the obstacle
        elseif obs_ids(obs_id_idx) ~= obs_ids(obs_id_idx+1)
            obs_around = obs_around + 1;
            obs_id_idx = obs_id_idx + 1;
        end
    end
end
