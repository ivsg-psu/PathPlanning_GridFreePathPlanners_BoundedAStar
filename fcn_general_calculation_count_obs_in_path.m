function [obs_around, obs_through] = ...
    fcn_general_calculation_count_obs_in_path(path)
    obs_ids = path(:,4);
    assert(obs_ids(1)==-1)
    obs_around = 0;
    obs_through = 0;
    obs_id_idx = 2;
    while obs_id_idx < length(obs_ids)
        if obs_ids(obs_id_idx) == obs_ids(obs_id_idx+1)
            obs_through = obs_through + 1;
            obs_id_idx = obs_id_idx + 2;
        elseif obs_ids(obs_id_idx) ~= obs_ids(obs_id_idx+1)
            obs_around = obs_around + 1;
            obs_id_idx = obs_id_idx + 1;
        end
    end
end
