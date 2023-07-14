function [is_reachable, num_steps, rgraph_total] = fcn_check_reachability(vgraph,start,finish)
    num_pts = size(vgraph,1);
    start_id = start(4);
    finish_id = finish(:,4);
    start_id_repeated = ones(size(finish_id,1),size(finish_id,2))*start_id;
    is_reachable = 0;
    rgraph_total = zeros(num_pts);
    for num_steps = 1:num_pts
        rgraph = vgraph^num_steps;
        rgraph_total = rgraph_total + rgraph;
        ind = sub2ind([num_pts,num_pts],start_id_repeated,finish_id);
        if sum(rgraph(ind)) > 0
            is_reachable = 1;
        end
    end
    rgraph_total = (rgraph_total>0); % make rgraph binary
end
