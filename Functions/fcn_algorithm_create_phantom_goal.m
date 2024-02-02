function [vgraph, cgraph, hvec, finish, all_pts] = fcn_algorithm_create_phantom_goal(vgraph, cgraph, hvec, finish, all_pts)
    % create a new vgraph column and row of zeros for the imaginary finish
    orig_size = size(vgraph,1);
    new_row = zeros(1,orig_size);
    new_col = new_row';
    vgraph = [[vgraph, new_col]; [new_row, 1]];
    % row can stay zeros (don't need to go from finish to anywhere)
    % column gets a 1 at every finish ID
    finish_idx = finish(:,4);
    vgraph(finish_idx,end) = 1;
    % cgraph gets a at every finish ID
    new_row = inf*ones(1,orig_size);
    new_col = inf*new_row';
    cgraph = [cgraph, new_col; new_row, inf];
    cgraph(finish_idx,end) = 0;
    % HVEC gets a 1 at every finish ID
    hvec(finish_idx) = 1;
    hvec = [hvec 0];
    % append old finish to all_pts
    all_pts = [all_pts; finish];
    % make phantom finish
    finish = [NaN NaN NaN max(finish_idx)+1];
end
